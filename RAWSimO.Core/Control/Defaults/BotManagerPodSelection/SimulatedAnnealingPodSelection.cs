using System;
using System.Collections.Generic;
using System.Linq;
using RAWSimO.Core.Bots;
using RAWSimO.Core.Configurations;
using RAWSimO.Core.Control.Defaults.OrderBatching;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Items;
using RAWSimO.Core.Management;
using RAWSimO.Core.Metrics;
using RAWSimO.Core.Waypoints;
using RAWSimO.MultiAgentPathFinding.DataStructures;
using RAWSimO.Toolbox;

namespace RAWSimO.Core.Control.Defaults.PodSelection
{
    /// <summary>
    /// Pod selection manager that combine Fully-Demand PPS with path planning with simulated annealing.
    /// </summary>
    public class SimulatedAnnealingPodSelectionManager : PodSelectionManager
    {
        /// <summary>
        /// Creates a new instance of this manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public SimulatedAnnealingPodSelectionManager(Instance instance) : base(instance) 
        { 
            _config = instance.ControllerConfig.TaskAllocationConfig.GetPodSelectionConfig() as SimulatedAnnealingPodSelectionConfiguration; 

            // Init
            pendingPods = new Dictionary<OutputStation, List<Pod>>(Instance.OutputStations.Select(s => new KeyValuePair<OutputStation, List<Pod>>(s, null)));
            foreach(var station in Instance.OutputStations)
                pendingPods[station] = new List<Pod>();
        }

        /// <summary>
        /// The config of this controller.
        /// </summary>
        private SimulatedAnnealingPodSelectionConfiguration _config;
        /// <summary>
        /// The time of current update start. To prevent exceed update period in Simulated Annealing process. 
        /// </summary>
        private DateTime startUpdate;
        /// <summary>
        /// Pod sets of stations prepared to be assigned to stations
        /// </summary>
        private Dictionary<OutputStation, List<Pod>> pendingPods = null;
        /// <summary>
        /// Extract requests of pending pods.
        /// </summary>
        private Dictionary<Pod, List<ExtractRequest>> pendingExtracts = new();

        private double nextUpdateTime = 0;

        private BotManager botManager;
        private FullySuppliedOrderManager orderManager;
        private PathManager pathManager;
        double temperature;

        private class BotInfo
        {
            public Bot bot {get; private set;}
            public double startTime {get; private set;}
            public BotTaskType taskType {get; private set;}
            public Waypoints.Waypoint startWaypoint  {get; private set;}
            public BotInfo(Bot bot, double startTime, BotTaskType taskType)
            {
                this.bot = bot;
                this.startTime = startTime;
                this.taskType = taskType;
                if (taskType == BotTaskType.None || taskType == BotTaskType.Rest)
                    startWaypoint = bot.CurrentWaypoint;
                else
                    startWaypoint = bot.TargetWaypoint;
            }
        }

        private class Solution
        {
            public SearchSpace.Point point;
            /// <summary>
            /// Estimated item throughput rate of this station
            /// </summary>
            public double stationRate;
            /// <summary>
            /// Difference between previous station rate (old - new solution). 
            /// </summary>
            public double difference = double.MaxValue;
            /// <summary>
            /// Fully-Fulfilled orders of this solution, should not contain orders in other solution
            /// </summary>
            public List<Order> orders = new();
            /// <summary>
            /// Station extract requests
            /// </summary>
            public Dictionary<Pod, List<ExtractRequest>> stationRequests = new();
            /// <summary>
            /// Pod extract requests
            /// </summary>
            public List<ExtractRequest> podRequests = new();
        }


        private class SearchSpace
        {
            public List<Point> points{get; set;}
            public OutputStation station {get; private set;}
            public Bot bot {get; private set;}
            public double startTime {get; private set;}
            /// <summary>
            /// Indicate there are no fully-fulfilled order,
            /// all the points in the search space are need to fulfilled one order.
            /// </summary>
            public SearchSpace(OutputStation station, Bot bot, double startTime)
            {
                this.station = station;
                this.bot = bot;
                this.startTime = startTime;
                this.points = new();
            }
            /// <summary>
            /// Keep only top n of points
            /// </summary>
            /// <param name="num"></param>
            public void KeepBest(int num)
            {
                points = points.OrderByDescending(p => p.itemNum).Take(num).ToList();
            }
            public void AddPoint(Pod pod, OutputStation station, HashSet<Order> orders, int itemNum)
            {
                points.Add(new Point(pod, station, orders, itemNum, this));
            }

            /// <summary>
            /// Calculate cumulative distribution function of the rate, 
            /// </summary>
            /// <exception cref="Exception"></exception>
            public void calculateCDF(){
                if (points.Count == 0) return;
                    
                // normalize rate
                double total = points.Sum(pt => pt.rate);
                points.ForEach(pt => {pt.rate /= total;});
                
                double cdf = 0.0;
                foreach(var pt in points)
                {
                    cdf += pt.rate;
                    pt.rateCDF = cdf;
                }
                // normalize cdf
                if (cdf != 1)
                    foreach(var pt in points)
                    {
                        pt.rateCDF/= cdf;
                        if (Double.IsNaN(pt.rateCDF))
                            throw new Exception("rateCDF can't be NaN");
                    }
            }
            
            /// <summary>
            /// Pick a point based on cumulative distribution function and weight
            /// </summary>
            /// <param name="weight">number within the interval of [0, 1)</param>
            /// <returns></returns>
            public Point Pick(double weight)
            {
                if (points.Count == 0) throw new Exception("No point in the search space");
                int index = points.BinarySearch(new Point(weight), new PointComparator());
                if (index < 0) // points[~index-1] < weight < points[~index]
                    index = ~index;
                if (index < 0 || index >= points.Count)
                    throw new Exception($"index {index} out of range (0, {points.Count}), weight: {weight}, cdf range: ({points.First().rateCDF},{points.Last().rateCDF})");
                return points[index];
            }
            public class Point
            {
                public SearchSpace searchSpace {get; private set;}
                public Pod pod {get; private set;}
                public OutputStation station {get; private set;}
                /// <summary>
                /// Fully-fulfilled orders
                /// </summary>
                public HashSet<Order> orders {get; private set;}
                /// <summary>
                /// Number of items in fully-fulfilled orders
                /// </summary>
                public int itemNum {get; private set;}
                /// <summary>
                /// Estimated item throughput rate after assigning this pod to station.
                /// </summary>
                public double rate = -1;
                public double rateCDF = -1;
                public Point(Pod pod, OutputStation station, HashSet<Order> orders, int itemNum, SearchSpace s)
                {
                    this.pod = pod;
                    this.station =station;
                    this.orders = orders;
                    this.itemNum = itemNum;
                    this.searchSpace = s;
                }
                public Point(double weight) {this.rateCDF = weight;}

            }
        
            private class PointComparator : IComparer<Point>
            {
                public int Compare(Point x, Point y)
                {
                    return x.rateCDF.CompareTo(y.rateCDF);
                }
            }
        }



        /// <summary>
        /// The next event when this element has to be updated.
        /// </summary>
        /// <param name="currentTime">The current time of the simulation.</param>
        /// <returns>The next time this element has to be updated.</returns>
        public override double GetNextEventTime(double currentTime) { return nextUpdateTime; }

        /// <summary>
        /// 
        /// </summary>
        public override void Update(double lastTime, double currentTime)
        {
            // Measure time for decision
            startUpdate = DateTime.Now;

            doPodSelection(lastTime, currentTime);

            // Calculate decision time
            Instance.Observer.TimePodSelection((DateTime.Now - startUpdate).TotalSeconds);
        }
        /// <summary>
        /// The start of this method. 
        /// </summary>
        private void doPodSelection(double lastTime, double currentTime)
        {
            if (_config.GreedyMethod) return; // do nothing if greedy method selected

            // only update once per update period
            if(nextUpdateTime > currentTime) return;
            nextUpdateTime = currentTime + _config.updatePeriod;

            init();

            // try allocate order with current inbound pods, to eliminate situation of assigning pod without extract task
            // Fully-supplied don't need extra pod movement, the only cost is station capacity, may cause order later ratio increase
            foreach(var station in Instance.OutputStations)
            {
                orderManager.FullySupplied(station, orderManager.pendingLateOrders);
                orderManager.FullySupplied(station, orderManager.pendingNotLateOrders);
            }

            // only deal with station without pending Pods and still has capacity
            var selectedStations = Instance.OutputStations.Where(s => pendingPods[s].Count == 0)
                .Where(s => s.CapacityInUse + s.CapacityReserved < s.Capacity).ToList();
            if (selectedStations.Count() == 0) return;

            // find available bot and the time finishing their job
            var botsInfo = GetBotsInfo(currentTime, selectedStations);
            // no bot found
            if(botsInfo.Keys.Count() == 0) return;
            // Only do pod selection when at least a rest bot exist
            if(!botsInfo.Values.Any(i => i.taskType == BotTaskType.Rest))
                return;

            pathManager.scheduleInit();
            var searchSpaces = createSearchSpaces(botsInfo);
            // check number of search space
            if(searchSpaces.Count == 0) return;

            var solutions = startSimulatedAnnealing(searchSpaces);

            // output results from solution
            var botsTask = new Dictionary<Bot, BotTask>();
            foreach(var sol in solutions.Values)
            {
                var pt = sol.point;
                var s = pt.searchSpace;
                // assign bot
                ExtractTask task = new ExtractTask(Instance, s.bot, pt.pod, s.station, sol.podRequests);
                botManager.EnqueueTask(s.bot, task);
                botsTask[s.bot] = task; // for later schedule priority output
                botManager.logPodAssignment();
                //botManager.RequestNewTask(s.bot);
                // assign station request of inbound pods
                foreach(var d in sol.stationRequests)
                {
                    botManager.AddExtract(d.Key, d.Value);
                }
                // allocate orders
                foreach(var order in sol.orders)
                {
                    orderManager.AllocateOrder(order, s.station);
                }
                // statistic: record estimated station item throughput rate
            }
            pathManager.OutputScheduledPriority(botsTask);
        }
        
        /// <summary>
        /// Initialize managers
        /// </summary>
        private void init()
        {
            if (botManager == null)
                botManager = Instance.Controller.BotManager;
            if (orderManager == null)
                orderManager = Instance.Controller.OrderManager as FullySuppliedOrderManager;
            if (pathManager == null)
                pathManager = Instance.Controller.PathManager;
        }

        /// <summary>
        /// Get bots that are available for stations with information of end time of current task and current task type.
        /// </summary>
        private Dictionary<OutputStation, BotInfo> GetBotsInfo(double CurrentTime, List<OutputStation> selectedStations)
        { 
            Dictionary<OutputStation, BotInfo> botsInfo = new();
            // get next available bot and it's time of each station
            foreach(var station in selectedStations)
            {
                var bots = botManager.stationBots[station]
                    .Where(bot => (bot.CurrentTask.Type == BotTaskType.None)
                        ||(bot.CurrentTask.Type == BotTaskType.Rest)
                        ||(bot.CurrentTask.Type == BotTaskType.ParkPod))
                    .Where(bot => botManager.CountFutureTasks(bot) == 0); 
                if(bots.Count() == 0) continue;
                //　pick rest or None bot first
                var restBots = bots.Where(b => (b.CurrentTask.Type == BotTaskType.None) || (b.CurrentTask.Type == BotTaskType.Rest));
                if (restBots.Count() > 0)
                {
                    botsInfo[station] = new BotInfo(restBots.First(), CurrentTime, BotTaskType.Rest);
                }
                else // bot with park pod task
                {
                    // find ending time of Park pod task from reservation table
                    var botTime = bots.Select(bot =>
                    {
                        var success = pathManager.FindArrivalTime(out double endTime, bot); // may failed because  the path hasn't be reserved
                        return new Tuple<Bot, double, bool>(bot, endTime, success);
                    });
                    // find bot with shortest arrival time
                    var bestBotTime = botTime.Where(t => t.Item3).MinBy(t => t.Item2);
                    if (bestBotTime != null)
                        botsInfo[station] = new BotInfo(bestBotTime.Item1, bestBotTime.Item2, BotTaskType.ParkPod);
                }
            }
            return botsInfo;
        }
        
        /// <summary>
        /// create search space of each station and assign pod-set if no fully-fulfillable order can be found
        /// </summary>
        /// <returns>search space</returns>
        private Dictionary<OutputStation, SearchSpace> createSearchSpaces(Dictionary<OutputStation, BotInfo> botsInfo)
        {
            Dictionary<OutputStation, SearchSpace> searchSpaces = new();
            HashSet<Order> consideredOrders = new();
            List<OutputStation> podSetStation = new(); // stations that need pod-set
            List<Order> podSetOrders = new(); // orders fulfilled by pod-set
            var iteration = new List<HashSet<Order>>(){orderManager.pendingLateOrders, orderManager.pendingNotLateOrders};
            foreach(var (undecidedOrders, i) in iteration.Select((value, i) => (value, i)))
            {
                foreach(var (station, botInfo) in botsInfo.Select(d => (d.Key, d.Value)))
                {
                    // ignore station already assigned with pod set
                    if(podSetStation.Any(s => s == station)) continue;
                    // find fully-fulfilled pods and orders
                    foreach (var pod in Instance.ResourceManager.UnusedPods)
                    {
                        var orders = new HashSet<Order>();
                        // Check all order backlog, where sufficient inventory is still available in the pod set with the new pod
                        foreach (var order in undecidedOrders.Where(o => o.Positions.All(p =>
                                    station.CountAvailable(p.Key) + pod.CountAvailable(p.Key) >= p.Value)))
                        {
                            orders.Add(order);
                            consideredOrders.Add(order);
                        }
                        if (orders.Count > 0)
                        {
                            var itemNum = orders.Sum(o => o.Positions.Sum(p => p.Value));
                            if(!searchSpaces.ContainsKey(station)) 
                                searchSpaces[station] = new(station, botInfo.bot, botInfo.startTime);
                            searchSpaces[station].AddPoint(pod, station, orders, itemNum);
                        }
                    }
                }
                // deal with unconsidered orders with bot without search space
                var unconsideredOrders = undecidedOrders.ExceptWithNew(consideredOrders);
                foreach(var item in botsInfo.Where(i => !searchSpaces.Values.Any(s => s.bot == i.Value.bot)))
                {
                    if (unconsideredOrders.Count == 0) break;
                    var station = item.Key;
                    var botInfo = item.Value;
                    // ignore station already assigned with pod set
                    if(podSetStation.Any(s => s == station)) continue;
                    var success = generatePodSet(out var selectedOrders, unconsideredOrders, station, botInfo);
                    if(success)
                    {
                        podSetOrders.AddRange(selectedOrders);
                        podSetStation.Add(station);
                    }
                }
            }

            // remove pods used by pod-set
            foreach(var (station, space) in searchSpaces.Select(d => (d.Key, d.Value)))
                searchSpaces[station].points.RemoveAll(pt => pendingPods.SelectMany(d => d.Value).Contains(pt.pod));
                
            // assign pod-set
            foreach(var station in podSetStation)
            {
                assignFirstPodSet(station, botsInfo[station]);
            }

            // process points in search spaces
            foreach(var (station, space) in searchSpaces.Select(d => (d.Key, d.Value)))
            {
                // Helpers
                var bot = space.bot;
                var botStartTime = botsInfo[station].startTime;
                // remove orders used by pod-set
                searchSpaces[station].points.RemoveAll(pt => pt.orders.Overlaps(podSetOrders));
                if(searchSpaces[station].points.Count == 0) // no arrival able pod
                {
                    searchSpaces.Remove(station);
                    continue;
                }
                // Take certain amount of pods with most items
                searchSpaces[station].KeepBest(_config.searchPodNum);
                // calculate item throughput rate in search space
                searchSpaces[station].points.ForEach(pt => {
                    pt.rate = 0;
                    // Estimate arrival time of the pod: May be time costly and not accurate, maybe some rough estimation is enough
                    // Then calculate station item throughput rate by arrival time
                    var startWaypoint = bot.TargetWaypoint; // for bot not in rest task (will no be executed unless bot except rest or None is selected)
                    if(bot.CurrentTask.Type == BotTaskType.Rest) startWaypoint = bot.CurrentWaypoint; // since rest task will be canceled
                    double endTime = botStartTime;
                    endTime += Distances.EstimateManhattanTime(startWaypoint, pt.pod.Waypoint, Instance);
                    if(double.IsInfinity(endTime)) return;
                    endTime += Instance.LayoutConfig.PodTransferTime;
                    var connectedPoints = pt.pod.Waypoint.GetInfoConnectedWaypoints().Cast<Waypoint>().ToList();
                    var entryPoint = connectedPoints.Where(p => !p.GetInfoStorageLocation()).First(); // point before entering pod storage location
                    endTime += Distances.EstimateManhattanTime(entryPoint, station.Waypoint, Instance);
                    if(double.IsInfinity(endTime)) return;
                    pt.rate = pt.itemNum / (Math.Max(endTime-Instance.Controller.CurrentTime, station.GetCurrentQueueTime()) 
                                        + pt.itemNum * Instance.LayoutConfig.ItemPickTime); 
                });
                // remove points with 0 rate
                searchSpaces[station].points.RemoveAll(pt => pt.rate == 0);
                // Only take certain amount of points
                //searchSpaces[station].points = searchSpaces[station].points.Take(20).ToList();
                if(searchSpaces[station].points.Count == 0) // no arrival able pod
                {
                    searchSpaces.Remove(station);
                    continue;
                }
                // sort the points
                searchSpaces[station].points = searchSpaces[station].points.OrderByDescending(p => p.rate).ToList();
                // process search space
                searchSpaces[station].calculateCDF();
            }
            return searchSpaces;
        }
        /// <summary>
        /// Generate a pod-set by allocating a order from undecided orders. 
        /// The selected order will be removed from undecided orders. 
        /// </summary>
        /// <returns>true, if the order can be fulfilled.</returns>
        private bool generatePodSet(out HashSet<Order> podSetOrders,HashSet<Order> undecidedOrders, OutputStation station, BotInfo botInfo)
        {
            // helpers
            Order fulfillableOrder = null;
            Dictionary<ItemDescription, int> requiredAmount = null;
            Dictionary<ItemDescription, List<Pod>> possiblePods = null;
            podSetOrders = new();
            // find an fulfillable order
            foreach(var order in undecidedOrders)
            {
                // require amount is total amount in order minus station available
                requiredAmount = new Dictionary<ItemDescription, int>(
                    order.Positions.Where(p => p.Value - station.CountAvailable(p.Key) > 0).Select(p =>
                    new KeyValuePair<ItemDescription, int>(p.Key, p.Value - station.CountAvailable(p.Key))));
                // get possible pods contained with items in the order
                possiblePods = new Dictionary<ItemDescription, List<Pod>>(requiredAmount.Select(
                    r => new KeyValuePair<ItemDescription, List<Pod>>(r.Key, new List<Pod>(
                        Instance.ResourceManager.UnusedPods.Where(pod => pod.CountAvailable(r.Key) > 0)
                    ))
                ));
                // check if the order is fulfillable
                if (!requiredAmount.Any(r => r.Value > possiblePods[r.Key].Sum(pod => pod.CountAvailable(r.Key))))
                {
                    fulfillableOrder = order;
                    podSetOrders.Add(order);
                    break;
                }
            }
            if (fulfillableOrder == null) return false;

            // greedy search from pod that can provided most item
            foreach(var pod in possiblePods.SelectMany(d => d.Value).Distinct().OrderByDescending(pod => 
                requiredAmount.Sum(r => Math.Min(r.Value, pod.CountAvailable(r.Key)))))
            {
                if(requiredAmount.All(r => r.Value <= 0))
                    break;
                // the pod can't supply any remaining required amount
                else if (requiredAmount.Sum(d => Math.Min(d.Value, pod.CountAvailable(d.Key))) == 0)
                    continue;
                pendingPods[station].Add(pod);
                // claim all pending pod to prevent be claimed by replenishment, reposition...
                // pod will be released right after leaving pending pods and become selected pod
                Instance.ResourceManager.ClaimPod(pod, null, BotTaskType.None);
                foreach(var item in requiredAmount.Keys)
                {
                    int takePod = Math.Min(requiredAmount[item], pod.CountAvailable(item));
                    requiredAmount[item] -= takePod;
                }
            }
            // check negative number
            if (requiredAmount.All(r => r.Value < 0))
                throw new Exception("remaining required amount shouldn't smaller than 0.");
            // Fully-Supplied should be done right after any success pod selection
            var request = orderManager.ExtraDecideAboutPendingOrder(station, pendingPods[station], undecidedOrders, fulfillableOrder);
            
            // deal with pending request
            foreach(var pod in pendingPods[station])
            {
                // Check and init
                if(!pendingExtracts.ContainsKey(pod))
                    pendingExtracts.Add(pod, new List<ExtractRequest>());
                else
                    throw new Exception("The pod has remaining pending extract request!");
                pendingExtracts[pod] = request[pod];
                // check empty extract
                if(pendingExtracts[pod].Count == 0)
                {
                    throw new Exception($"New pod {pod.ID} in pod set without assigning any extract request");
                }
                request.Remove(pod);
            }
            if(request.Keys.Count > 0)
                throw new Exception("Remain requests not deal with!");
            return true;
        }

        private void assignFirstPodSet(OutputStation station, BotInfo botInfo)
        {
            // Helper
            var bot = botInfo.bot;
            // output one of the selected pod and extract requests
            var pod = pendingPods[station].First();
            pendingPods[station].Remove(pod);
            // release pod for later extract request
            Instance.ResourceManager.ReleasePod(pod);
            var extractRequests = pendingExtracts[pod];
            botManager.EnqueueTask(bot, new ExtractTask(Instance, bot, pod, station, extractRequests));
            botManager.logPodAssignment();
            // remove pending extracts
            pendingExtracts.Remove(pod);
            orderManager.SignalPodAssigned();

            // try to make reservation of path in solution, prevent affect by other solution
            // but it is ok to failed
            List<ReservationTable.Interval> path = new();
            var success = pathManager.schedulePath(out var endTime, ref path, botInfo.startTime, bot, botInfo.startWaypoint, pod.Waypoint, false);
            endTime += Instance.LayoutConfig.PodTransferTime;
            var successAgain = success && pathManager.schedulePath(out endTime, ref path, endTime, bot, pod.Waypoint, station.Waypoint, true);
            if(success || successAgain) pathManager.OverwriteScheduledPath(bot, path);
            return;
        }

        private Dictionary<OutputStation, Solution> startSimulatedAnnealing(Dictionary<OutputStation, SearchSpace> searchSpaces)
        {
            Dictionary<OutputStation, Solution> solutions = new();
            // initialize the temperature
            temperature = _config.initTemp;

            // get initial solution by selecting one point for each station
            foreach(var pair in searchSpaces.OrderBy(d => d.Value.points.Count)) //
                initSolutions(ref solutions, pair.Value);

            if(_config.BruteForceMethod)
            {
                // brute-force try all pods
                for(int i = 0; i < 5; i++) // only try certain times
                {
                    bool changed = false;
                    foreach(var space in searchSpaces.Values)
                    {
                        var points = new List<SearchSpace.Point>(space.points);
                        points.Reverse();
                        foreach(var point in space.points)
                        {
                            var success = updateSolutions(ref solutions, point, false);
                            changed = changed || success;
                        }
                    }
                    // end iteration if all solution has small difference
                    if(solutions.Values.All(s => Math.Abs(s.difference) == 0)) break;
                    // do until no solution change
                    if(!changed) break;
                }
            }
            else
            {
                // do until temperature is too low, or item throughput rate converge
                while(!_config.InitSolutionMethod && temperature > _config.minTemp 
                      && (DateTime.Now - startUpdate).TotalSeconds < _config.updatePeriod - 0.01) // remain 10ms to output solutions
                {
                    // pick a random station
                    var space = searchSpaces.Values.ToList()[Instance.Randomizer.NextInt(searchSpaces.Values.Count)];
                    // pick a point from the station's search space
                    var point = space.Pick(Instance.Randomizer.NextDouble());

                    updateSolutions(ref solutions, point);

                    // end iteration if all solution has small difference
                    if(solutions.Values.All(s => Math.Abs(s.difference) < _config.minDifference))
                        break;

                    // decrease temperature
                    temperature *= _config.coolingRate;
                }
            }
            return solutions;
        }

        /// <summary>
        /// Try every pod in search space to get a solution.
        /// </summary>
        private bool initSolutions(ref Dictionary<OutputStation, Solution> solutions, SearchSpace space)
        {
            Solution sol = null;
            foreach(var point in space.points)
            {
                // skip exist pod
                if(solutions.Values.Any(s => s.point.pod == point.pod)) continue;
                sol = createSolution(solutions, point, out List<ReservationTable.Interval> path);
                if(sol != null)
                {
                    // store the solution
                    sol.difference =double.MaxValue;
                    solutions.Add(point.station, sol);
                    pathManager.OverwriteScheduledPath(point.searchSpace.bot, path);
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Update the solutions with the point, if its item throughput rate is acceptable
        /// </summary>
        /// <returns>success</returns>
        private bool updateSolutions(ref Dictionary<OutputStation, Solution> solutions, SearchSpace.Point point, bool explore = true)
        {
            // check if pod already exist in other solutions
            foreach(var (station, solution) in solutions.Select(d => (d.Key, d.Value)))
            {
                if(point.station == station)// ignore same station
                {
                    if(point.pod == solution.point.pod) return replanSolution(ref solutions, station, false, explore);
                    else continue;
                }
                if(point.pod == solution.point.pod) // swap point
                {
                    return swapSolution(ref solutions, station, point.searchSpace.station, explore);
                }
            }

            Solution sol = createSolution(solutions, point, out List<ReservationTable.Interval> path);
            if (sol == null) return false;

            // may find solution not in init solution
            if(!solutions.ContainsKey(point.station))
            {
                sol.difference = double.MaxValue;
                solutions[point.station] = sol;
                pathManager.OverwriteScheduledPath(point.searchSpace.bot, path);
                return true;
            }

            sol.difference = solutions[point.station].stationRate - sol.stationRate;
            double rand = Instance.Randomizer.NextDouble();
            if (sol.difference < 0 || (explore && Math.Exp(-sol.difference*10000/temperature) > rand))
            {
                solutions[point.station] = sol;
                pathManager.OverwriteScheduledPath(point.searchSpace.bot, path);
                return true;
            }
            else
                return false;
        }
        /// <summary>
        /// Replan the path of the solution
        /// </summary>
        /// <param name="solutions"></param>
        /// <param name="station"></param>
        /// <param name="force">if true, force update the solution. </param>
        /// <param name="explore">if true and force = false, accept worse solution according to temperature.</param>
        private bool replanSolution(ref Dictionary<OutputStation, Solution> solutions, OutputStation station, bool force = false, bool explore = true)
        {
            var point = solutions[station].point;
            var sol = createSolution(solutions, point, out var path);
            if (sol == null) return false;

            // may find solution not in init solution
            if(!solutions.ContainsKey(point.station))
            {
                sol.difference = double.MaxValue;
                solutions[point.station] = sol;
                pathManager.OverwriteScheduledPath(point.searchSpace.bot, path);
                return true;
            }

            sol.difference = solutions[point.station].stationRate - sol.stationRate;
            double rand = Instance.Randomizer.NextDouble();
            if (force || sol.difference < 0 || (explore && Math.Exp(-sol.difference*10000/temperature) > rand))
            {
                solutions[point.station] = sol;
                pathManager.OverwriteScheduledPath(point.searchSpace.bot, path);
                return true;
            }
            else
                return false;
        }
        /// <summary>
        /// swap the pod of two solution.
        /// </summary>
        private bool swapSolution(ref Dictionary<OutputStation, Solution> solutions, OutputStation station1, OutputStation station2, bool explore = true)
        {
            if(!solutions.ContainsKey(station1) || !solutions.ContainsKey(station2)) return false;
            var point1 = solutions[station1].point;
            var point2 = solutions[station2].point;
            var space1 = point1.searchSpace;
            var space2 = point2.searchSpace;
            var pod1 = point1.pod;
            var pod2 = point2.pod;

            // find point with same pod in each others' search space
            var newPoint1 = space1.points.FirstOrDefault(pt => pt.pod == pod2);
            if(newPoint1 == null) return false;
            var newPoint2 = space2.points.FirstOrDefault(pt => pt.pod == pod1);
            if(newPoint2 == null) return false;

            // try new solutions
            var sol1 = createSolution(solutions, newPoint1, out var path1);
            if(sol1 == null) return false;
            // store old path1
            //var oldPath1 = pathManager.GetSchedulePath(space1.bot);
            pathManager.OverwriteScheduledPath(space1.bot, path1);
            var sol2 = createSolution(solutions, newPoint2, out var path2);
            // add old path back, may failed because path1 may can't be fully removed
            // temporary solution: force replan path
            replanSolution(ref solutions, station1, true);
            //pathManager.OverwriteScheduledPath(space1.bot, oldPath1);
            if(sol2 == null) return false;
            sol1.difference = solutions[station1].stationRate - sol1.stationRate;
            sol2.difference = solutions[station2].stationRate - sol2.stationRate;

            // evaluate new solution
            double rand = Instance.Randomizer.NextDouble();
            if (sol1.difference + sol2.difference < 0 || 
                (explore && Math.Exp(-(sol1.difference + sol2.difference) *10000/temperature) > rand))
            {
                solutions[station1] = sol1;
                solutions[station2] = sol2;
                // temporary solution, need to plan again to prevent collision
                createSolution(solutions, newPoint1, out path1);
                pathManager.OverwriteScheduledPath(space1.bot, path1);
                // temporary solution, need to plan again to prevent collision
                createSolution(solutions, newPoint2, out path2);
                pathManager.OverwriteScheduledPath(space2.bot, path2);
                return true;
            }
            else // recover original path of station 1 by replan
            {

            }
            return false;
        }

        /// <summary>
        /// Find orders in points' order but fulfilled
        /// 1. not in solutions of other station
        /// 2. consider remained item in pod and station
        /// </summary>
        /// <returns>
        /// null, if
        /// 1. no possible order.
        /// 2. can't find path in window.
        /// </returns>
        private Solution createSolution(Dictionary<OutputStation, Solution> solutions, SearchSpace.Point point, out List<ReservationTable.Interval> path)
        {
            // Helpers
            path = null;
            var station = point.station;
            var otherSolution = solutions.Values.Where(s => s.point.pod != point.pod);
            var possibleOrders = point.orders.Except(otherSolution.SelectMany(s => s.orders));

            if (possibleOrders.Count() == 0) return null;

            // Helpers
            Solution sol = new()
            {
                point = point,
                stationRequests = new Dictionary<Pod, List<ExtractRequest>>(
                station.InboundPods.Select(pod => new KeyValuePair<Pod, List<ExtractRequest>>(pod, new List<ExtractRequest>())))
            };
            // remained items in station and pod
            var stationRemain = new Dictionary<Pod, Dictionary<ItemDescription, int>>(
                station.InboundPods.ToDictionary(p => p, 
                p => p.ItemDescriptionsContained.ToDictionary(i => i, i => p.CountAvailable(i))));
            var podRemain = new Dictionary<ItemDescription, int>(
                point.pod.ItemDescriptionsContained.ToDictionary(i =>i, i => point.pod.CountAvailable(i)));
            
            // TODO: pick order priority should not be timed in not late orders
            foreach(var order in possibleOrders) // should be in sequence of time
            {
                if(station.CapacityInUse + station.CapacityReserved + sol.orders.Count >= station.Capacity) break;

                if (order.Positions.Any(p => 
                stationRemain.Values.Sum(d => d.ContainsKey(p.Key)? d[p.Key]: 0)
                + (podRemain.ContainsKey(p.Key)? podRemain[p.Key] : 0) < p.Value))
                    continue; // order can't be fulfilled

                sol.orders.Add(order);
                // calculate extract request
                var requireRequests = Instance.ResourceManager.GetExtractRequestsOfOrder(order).
                    GroupBy(r => r.Item).ToDictionary(g => g.Key, g => g.ToList());
                foreach(var item in requireRequests.Keys)
                {
                    // take from inbound pod of station first
                    foreach(var pod in station.InboundPods)
                    {
                        if (stationRemain[pod].ContainsKey(item))
                        {
                            int takeStation = Math.Min(requireRequests[item].Count(), stationRemain[pod][item]);
                            stationRemain[pod][item] -= takeStation;
                            sol.stationRequests[pod].AddRange(requireRequests[item].GetRange(0, takeStation));
                            requireRequests[item].RemoveRange(0, takeStation);
                        }
                    }
                    // then take from pod
                    if(podRemain.ContainsKey(item))
                    {
                        int takePod = Math.Min(requireRequests[item].Count, podRemain[item]);
                        podRemain[item] -= takePod;
                        sol.podRequests.AddRange(requireRequests[item].GetRange(0, takePod));
                        requireRequests[item].RemoveRange(0, takePod);
                    }
                }
            }
            if(sol.orders.Count == 0) return null;

            // Helpers
            var s = point.searchSpace;
            path = new();
            // estimate time of: bot -> pod
            if(!pathManager.schedulePath(out double endTime, ref path, s.startTime,s.bot, s.bot.TargetWaypoint, point.pod.Waypoint, false))
                return null; // failed to find path in window
            // Add time of lifting pod
            endTime += Instance.LayoutConfig.PodTransferTime; 
            // estimate time of: pod->station
            if(!pathManager.schedulePath(out endTime, ref path, endTime, s.bot, point.pod.Waypoint, s.station.Waypoint, true))
                return null; // / failed to find path in window
            // Estimated Item throughput rate of the station
            var itemNum = sol.orders.Sum(o => o.Positions.Sum(p => p.Value));
            sol.stationRate = itemNum / (Math.Max(endTime-Instance.Controller.CurrentTime, s.station.GetCurrentQueueTime()) 
                                      + itemNum * Instance.LayoutConfig.ItemPickTime); 
            return sol;
        }

        /// <summary>
        /// Get pod selection and extract requests from the pod selection manager
        /// </summary>
        /// <param name="selectedPod"></param>
        /// <param name="extractRequests"></param>
        /// <param name="bot"></param>
        /// <param name="oStation"></param>
        /// <returns></returns>
        public bool GetResult(out Pod selectedPod, out List<ExtractRequest> extractRequests, Bot bot, OutputStation oStation)
        {
            selectedPod = null;
            extractRequests = null;
            if (_config.GreedyMethod)
                return GreedyMethod(out selectedPod, out extractRequests, bot, oStation);
            else
            {
                if(pendingPods[oStation].Count == 0) return false;
                // output the selected pod and extract requests
                selectedPod = pendingPods[oStation].First();
                pendingPods[oStation].Remove(selectedPod);
                // release pod for later extract request
                Instance.ResourceManager.ReleasePod(selectedPod);
                extractRequests = pendingExtracts[selectedPod];
                // remove pending extracts
                pendingExtracts.Remove(selectedPod);
                orderManager.SignalPodAssigned();
                return true;
            }
        }

        /// <summary>
        /// A greedy method of maximizing item throughput rate by considering 
        /// number of items in fully-fulfillable orders and path congestion
        /// </summary>
        public bool GreedyMethod(out Pod selectedPod, out List<ExtractRequest> extractRequests, Bot bot, OutputStation oStation)
        {
            // check order manager
            if (Instance.Controller.OrderManager.GetType() != typeof(FullySuppliedOrderManager))
                throw new ArgumentException("Unknown order manager type for Simulated Annealing Pod Selection: " + Instance.Controller.OrderManager.GetType().ToString());
            init();
            foreach(var undecidedOrders in new List<HashSet<Order>>{orderManager.pendingLateOrders, orderManager.pendingNotLateOrders} )
            {
                // search not late orders if no late order can be fulfilled
                // end search if no not late order can be fulfilled
                if (undecidedOrders.Count == 0)
                {
                    // don't search not late order even if station remain capacity
                    if (orderManager.lateOrdersEnough)
                        break; 
                    else
                        continue;
                }
                // try allocate order with current inbound pods, to eliminate situation of assigning pod without extract task
                orderManager.FullySupplied(oStation, undecidedOrders);

                // check station capacity
                if (oStation.CapacityInUse + oStation.CapacityReserved >= oStation.Capacity)
                    break;

                // pick new pod or pod set if there are none
                if (pendingPods[oStation].Count == 0)
                {
                    // Temporary: Greedy solution of maximizing station's item throughput rate
                    // Determine best pod from all unused pods by maximizing station's item throughput
                    Dictionary<Pod, Tuple<List<Order>, int>> scores = new();
                    foreach (var pod in Instance.ResourceManager.UnusedPods)
                    {
                        int Score = 0;
                        var orders = new List<Order>();
                        // Check all order backlog, where sufficient inventory is still available in the pod set with the new pod
                        foreach (var order in undecidedOrders.Where(o => o.Positions.All(p =>
                            oStation.CountAvailable(p.Key) + pod.CountAvailable(p.Key) >= p.Value)))
                        {
                            Score += order.Positions.Sum(p => p.Value); // number of all items
                            orders.Add(order);
                        }
                        scores.Add(pod, new Tuple<List<Order>, int> (orders, Score));
                    }
                    // only consider top searchPodNum pods of the score
                    var bestPod = scores.OrderByDescending(d => d.Value.Item2).Take(_config.searchPodNum).Select(d => d.Key).
                            OrderByDescending(pod => {
                                // Estimate arrival time of the pod: May be time costly and not accurate, maybe some rough estimation is enough
                                double endTime;
                                if(!pathManager.findPath(out endTime, Instance.Controller.CurrentTime, bot, bot.CurrentWaypoint, pod.Waypoint, false))
                                    return 0; // can't find path
                                endTime += Instance.LayoutConfig.PodTransferTime;
                                if(!pathManager.findPath(out endTime, endTime, bot, pod.Waypoint, oStation.Waypoint, true))
                                    return 0; // can't find path
                                // estimated station item throughput rate, consider item picking time of pods in queue, but ignore other pods not in queue yet
                                return scores[pod].Item2 / (Math.Max(endTime-Instance.Controller.CurrentTime, oStation.GetCurrentQueueTime()) 
                                                            + scores[pod].Item2 * Instance.LayoutConfig.ItemPickTime); 
                            }).ThenByDescending(pod => scores[pod].Item2).First(); // break tie by score
                    var possibleOrders = scores[bestPod].Item1;
                    // make sure at least one pod is assigned, 
                    if (possibleOrders.Count > 0)
                    {
                        pendingPods[oStation].Add(bestPod);
                        Instance.ResourceManager.ClaimPod(bestPod, null, BotTaskType.None);
                        // Init if haven't
                        if (!pendingExtracts.ContainsKey(bestPod)) pendingExtracts.Add(bestPod, new List<ExtractRequest>());
                        // Fully-Supplied should be done right after any success pod selection
                        var request = orderManager.ExtraDecideAboutPendingOrders(oStation, bestPod, undecidedOrders, possibleOrders);
                        // deal with best pod request
                        pendingExtracts[bestPod] = request;
                    }
                    else // by adding pods to an fully fulfillable order with longest stay time in backlog
                    { 
                        bool success = false;
                        // search from order with longest stay time in backlog i.e. first put in backlog
                        foreach(var order in undecidedOrders)
                        {
                            // in original Fully-supplied does not consider inbound pods
                            var requiredAmount = new Dictionary<ItemDescription, int>(
                                order.Positions.Where(p => p.Value - oStation.CountAvailable(p.Key) > 0).Select(p =>
                                new KeyValuePair<ItemDescription, int>(p.Key, p.Value - oStation.CountAvailable(p.Key))));
                            // get possible pods contained with items in the order
                            var possiblePods = new Dictionary<ItemDescription, List<Pod>>(requiredAmount.Select(
                                r => new KeyValuePair<ItemDescription, List<Pod>>(r.Key, new List<Pod>(
                                    Instance.ResourceManager.UnusedPods.Where(pod => pod.CountAvailable(r.Key) > 0)
                                ))
                            ));
                            // check if the order is fulfillable
                            if (requiredAmount.Any(r => r.Value > possiblePods[r.Key].Sum(pod => pod.CountAvailable(r.Key))))
                                continue;

                            // greedy search from pod that can provided most item
                            foreach(var pod in possiblePods.SelectMany(d => d.Value).Distinct().OrderByDescending(pod => 
                                requiredAmount.Sum(r => Math.Min(r.Value, pod.CountAvailable(r.Key)))))
                            {
                                if(requiredAmount.All(r => r.Value <= 0))
                                    break;
                                // the pod can't supply any remaining required amount
                                else if (requiredAmount.Sum(d => Math.Min(d.Value, pod.CountAvailable(d.Key))) == 0)
                                    continue;
                                pendingPods[oStation].Add(pod);
                                // claim all pending pod to prevent be claimed by replenishment, reposition...
                                // pod will be released right after leaving pending pods and become selected pod
                                Instance.ResourceManager.ClaimPod(pod, null, BotTaskType.None);
                                foreach(var item in requiredAmount.Keys)
                                {
                                    int takePod = Math.Min(requiredAmount[item], pod.CountAvailable(item));
                                    requiredAmount[item] -= takePod;
                                }
                                
                            }
                            // check negative number
                            if (requiredAmount.All(r => r.Value < 0))
                                throw new Exception("remaining required amount shouldn't smaller than 0.");
                            else
                            {
                                // Fully-Supplied should be done right after any success pod selection
                                var request = orderManager.ExtraDecideAboutPendingOrder(oStation, pendingPods[oStation], undecidedOrders, order);
                                
                                // deal with pending request
                                foreach(var pod in pendingPods[oStation])
                                {
                                    // Check and init
                                    if(!pendingExtracts.ContainsKey(pod))
                                        pendingExtracts.Add(pod, new List<ExtractRequest>());
                                    else
                                        throw new Exception("The pod has remaining pending extract request!");
                                    pendingExtracts[pod] = request[pod];
                                    // check empty extract
                                    if(pendingExtracts[pod].Count == 0)
                                        throw new Exception($"New pod {pod.ID} in pod set without assigning any extract request");
                                    request.Remove(pod);
                                }
                                if(request.Keys.Count > 0)
                                    throw new Exception("Remain requests not deal with!");
                                success = true;
                            }
                            break; 
                        }
                        if (!success) // failed to find a fulfillable order, try not late order
                            continue;
                    }
                }
                // output the selected pod and extract requests
                selectedPod = pendingPods[oStation].First();
                pendingPods[oStation].Remove(selectedPod);
                // release pod for later extract request
                Instance.ResourceManager.ReleasePod(selectedPod);
                extractRequests = pendingExtracts[selectedPod];
                // remove pending extracts
                pendingExtracts.Remove(selectedPod);
                orderManager.SignalPodAssigned();
                return true;
            }
            // can't assign any pod
            selectedPod = null;
            extractRequests = null;
            return false;
        }

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public override void SignalCurrentTime(double currentTime) { /* Do nothing*/ }
    }
}