using System;
using System.Collections.Generic;
using System.Linq;
using RAWSimO.Core.Bots;
using RAWSimO.Core.Configurations;
using RAWSimO.Core.Control.Defaults.OrderBatching;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Items;
using RAWSimO.Core.Management;

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
        /// <summary>
        /// Search space of each station in simulated annealing.
        /// List of order: all orders that can be fulfilled by the pod
        /// </summary>
        private Dictionary<OutputStation,  SearchSpace> searchSpace = new();

        private Dictionary<OutputStation, Solution> solution = new();

        /// <summary>
        /// Store orders that need pod-set to fulfilled
        /// </summary>
        private HashSet<Order> selectedOrders = new();


        private class Solution
        {
            public SearchSpace.Point point;
            /// <summary>
            /// Estimated item throughput rate of this station
            /// </summary>
            public double stationRate;
            /// <summary>
            /// Absolute difference between previous station rate
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
            public List<Point> points{get; private set;}
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
            public void AddPoint(Pod pod, OutputStation station, List<Order> orders, int itemNum)
            {
                points.Add(new Point(pod, station, orders, itemNum, this));
            }

            /// <summary>
            /// Calculate cumulative distribution function of the rate, 
            /// and remove points with rate smaller than 1% of total rate
            /// </summary>
            /// <exception cref="Exception"></exception>
            public void calculateCDF(){
                if (points.Count == 0) return;
                    
                // normalize rate
                double total = points.Sum(pt => pt.rate);
                points.ForEach(pt => {pt.rate /= total;});
                // remove points with rate smaller than 1%
                points.RemoveAll(pt => pt.rate < 0.01);
                
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
                public SearchSpace searchSpace;
                public Pod pod;
                public OutputStation station;
                /// <summary>
                /// Fully-fulfilled orders
                /// </summary>
                public List<Order> orders;
                /// <summary>
                /// Number of items in fully-fulfilled orders
                /// </summary>
                public int itemNum = -1;
                /// <summary>
                /// Estimated item throughput rate after assigning this pod to station.
                /// </summary>
                public double rate = -1;
                public double rateCDF = -1;
                public Point(Pod pod, OutputStation station, List<Order> orders, int itemNum, SearchSpace s)
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

            // try late orders first, then try not late orders
            foreach(var undecidedOrders in new List<HashSet<Order>>(){orderManager.pendingLateOrders, orderManager.pendingNotLateOrders})
            {
                if (undecidedOrders.Count == 0) continue;

                if(!createSearchSpace(undecidedOrders)) continue;
                Instance.LogVerbose($"{searchSpace.Keys.Count} search space are created.");

                handleEmptySearchSpace(undecidedOrders);

                Instance.LogVerbose($"{searchSpace.Keys.Count} search space are remained.");
                if(searchSpace.Keys.Count == 0) continue; // no search space

                startSimulatedAnnealing();

                Instance.LogVerbose($"{solution.Values.Count()} solutions are found.");
                foreach(var sol in solution.Values)
                {
                    Instance.LogVerbose($"orders: {string.Join(", ", sol.orders.Select(o => string.Join(", ", o.Positions.Select(p => $"{p.Key.ID}({p.Value})"))))}");
                    Instance.LogVerbose($"pod: {string.Join(", ", sol.point.pod.ID)}");
                    Instance.LogVerbose($"bot: {string.Join(", ", sol.point.searchSpace.bot.ID)}");
                    Instance.LogVerbose("---");
                }
                

                // output results from solution
                foreach(var sol in solution.Values)
                {
                    var pt = sol.point;
                    var s = pt.searchSpace;
                    // assign bot
                    ExtractTask task = new ExtractTask(Instance, s.bot, pt.pod, s.station, sol.podRequests);
                    botManager.EnqueueTask(s.bot, task);
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
                        undecidedOrders.Remove(order);
                    }
                    searchSpace.Remove(s.station);
                    // statistic: record estimated station item throughput rate
                }
                solution.Clear();
            }
            searchSpace.Clear();
            Instance.LogVerbose("End of update.");
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
        /// create search space of each station
        /// </summary>
        /// <param name="undecidedOrders"></param>
        /// <returns>false, if no station has capacity left.</returns>
        private bool createSearchSpace(HashSet<Order> undecidedOrders)
        {
            // only deal with station without pending Pods and still has capacity
            var selectedStations = Instance.OutputStations.Where(s => pendingPods[s].Count == 0)
                .Where(s => s.CapacityInUse + s.CapacityReserved < s.Capacity).ToList();
            Instance.LogVerbose($"{selectedStations.Count()} stations are considered in this simulated annealing update.");
            if (selectedStations.Count() == 0) return false;

            pathManager.scheduleInit();

            Instance.LogVerbose($"try to create {selectedStations.Except(searchSpace.Keys).Count()} search space");


            // create search space of station if haven't exist: get next available bot and it's time of each station
            foreach(var station in selectedStations.Except(searchSpace.Keys))
            {
                // TODO: select bot that is not extract task, accept park, rest, none
                // TODO: make sure bot does't has extract task in task queue
                var bots = botManager.stationBots[station]
                    .Where(bot => (bot.CurrentTask.Type == BotTaskType.None)||(bot.CurrentTask.Type == BotTaskType.Rest))
                    .Where(bot => botManager.CountFutureTasks(bot) == 0); 
                if(bots.Count() == 0) continue;
                // Estimated bots' next available time
                var botTime = bots.Select(bot => {
                        var botN = bot as BotNormal; 
                        var success = botN.EstimateFinishedTime(out double endTime);
                        return new Tuple<Bot, double, bool>(bot, endTime, success);
                        });
                System.Console.WriteLine($"station {station.ID}: {string.Join(", ", botTime.Select(t => $"{t.Item1}/{t.Item2}/{t.Item3}"))}");
                var bestBotTime = botTime.Where(t => t.Item3).MinBy(t => t.Item2);
                if(bestBotTime != null)
                {
                    searchSpace[station] = new SearchSpace(station, bestBotTime.Item1, bestBotTime.Item2);
                }
            }

            if (searchSpace.Keys.Count() == 0) return false;
            Instance.LogVerbose($"try to establish {searchSpace.Keys.Count()} search space");

            foreach(var station in searchSpace.Keys)
            {
                var bot = searchSpace[station].bot;
                var botStartTime = searchSpace[station].startTime;
                // find fully-fulfilled pods and orders
                foreach (var pod in Instance.ResourceManager.UnusedPods)
                {
                    var orders = new List<Order>();
                    // Check all order backlog, where sufficient inventory is still available in the pod set with the new pod
                    foreach (var order in undecidedOrders.Where(o => o.Positions.All(p =>
                                station.CountAvailable(p.Key) + pod.CountAvailable(p.Key) >= p.Value)))
                    {
                        orders.Add(order);
                    }
                    if (orders.Count > 0)
                    {
                        var itemNum = orders.Sum(o => o.Positions.Sum(p => p.Value));
                        searchSpace[station].AddPoint(pod, station, orders, itemNum);
                    }
                }
                // Do nothing if no fully-fulfilled orders
                if(searchSpace[station].points.Count > 0)
                {
                    // Take certain amount of pods with most items
                    searchSpace[station].KeepBest(_config.searchPodNum);
                    // calculate item throughput rate in search space
                    searchSpace[station].points.ForEach(pt => {
                        pt.rate = 0;
                        // Estimate arrival time of the pod: May be time costly and not accurate, maybe some rough estimation is enough
                        // Then calculate station item throughput rate by arrival time
                        if(!pathManager.findPath(out double endTime, botStartTime, bot, bot.TargetWaypoint, pt.pod.Waypoint, false))
                            return; // can't find path, thus throughput rate = 0
                        endTime += Instance.LayoutConfig.PodTransferTime;
                        if(!pathManager.findPath(out endTime, endTime, bot,  pt.pod.Waypoint, station.Waypoint, true))
                            return;
                        pt.rate = pt.itemNum / (Math.Max(endTime-Instance.Controller.CurrentTime, station.GetCurrentQueueTime()) 
                                            + pt.itemNum * Instance.LayoutConfig.ItemPickTime); 
                    });
                    // remove points with 0 rate
                    searchSpace[station].points.RemoveAll(pt => pt.rate == 0);
                    // process search space
                    searchSpace[station].calculateCDF();
                }
            }
            
            return true;
        }

        /// <summary>
        /// Select pod-set for stations with empty search space and remove them
        /// </summary>
        /// <param name="undecidedOrders"></param>
        private void handleEmptySearchSpace(HashSet<Order> undecidedOrders)
        {
            foreach(var space in searchSpace.Values.Where(s => s.points.Count == 0))
            {
                // Helpers
                var station = space.station;
                var success = false;
                // init
                selectedOrders.Clear();
                foreach(var order in undecidedOrders) // order by timestamp
                {

                    // require amount is total amount in order minus station available
                    var requiredAmount = new Dictionary<ItemDescription, int>(
                        order.Positions.Where(p => p.Value - station.CountAvailable(p.Key) > 0).Select(p =>
                        new KeyValuePair<ItemDescription, int>(p.Key, p.Value - station.CountAvailable(p.Key))));
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
                        pendingPods[station].Add(pod);
                        // claim all pending pod to prevent be claimed by replenishment, reposition...
                        // pod will be released right after leaving pending pods and become selected pod
                        Instance.ResourceManager.ClaimPod(pod, null, BotTaskType.None);
                        // remove pod from other search space
                        searchSpace.Values.Where(s => s.points.Count > 0).ToList().ForEach(s => {
                            s.points.RemoveAll(pt => pt.pod == pod);});
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
                        var request = orderManager.ExtraDecideAboutPendingOrder(station, pendingPods[station], undecidedOrders, order);
                        
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
                        Instance.LogVerbose($"station {station.ID} select pod-set: {string.Join(", ", pendingPods[station].Select(p => $"pod {p.ID}({string.Join(", ", pendingExtracts[p].Select(r => r.Item.ID))})"))}");
                        selectedOrders.Add(order); // store order of the pod-set to prevent chosen by SA
                        // output one of theselected pod and extract requests
                        var selectedPod = pendingPods[station].First();
                        pendingPods[station].Remove(selectedPod);
                        // release pod for later extract request
                        Instance.ResourceManager.ReleasePod(selectedPod);
                        var extractRequests = pendingExtracts[selectedPod];
                        botManager.EnqueueTask(space.bot, new ExtractTask(Instance, space.bot, selectedPod, station, extractRequests));
                        botManager.logPodAssignment();
                        // remove pending extracts
                        pendingExtracts.Remove(selectedPod);
                        orderManager.SignalPodAssigned();
                        success = true;
                        break;
                    }
                }
                if(!success)
                    Instance.LogVerbose($"Warning! can't select pod-set for station {station.ID}");
            }
            // remove search space without point
            searchSpace = searchSpace.Where(s => s.Value.points.Count > 0).ToDictionary(d => d.Key, d=> d.Value);
        }

        private void startSimulatedAnnealing()
        {
            // initialize the temperature
            temperature = _config.initTemp;

            // get initial solution by selecting one point for each station
            foreach(var pair in searchSpace.OrderBy(d => d.Value.points.Count)) //
            {
                // get a pod from search space
                var point = pair.Value.Pick(Instance.Randomizer.NextDouble());
                updateSolution(point);
            }
            foreach(var sol in solution.Values)
                {
                    Instance.LogVerbose($"orders: {string.Join(", ", sol.orders.Select(o => string.Join(", ", o.Positions.Select(p => $"{p.Key.ID}({p.Value})"))))}");
                    Instance.LogVerbose($"pod: {string.Join(", ", sol.point.pod.ID)}");
                    Instance.LogVerbose($"bot: {string.Join(", ", sol.point.searchSpace.bot.ID)}");
                    Instance.LogVerbose("---temp---");
                }
            
            

            // do until temperature is too low, or (ignore for now)total item throughput rate converge
            int i = 0;
            while(temperature > _config.minTemp && i < _config.maxIteration) 
            {
                i++;
                // generate a new solution by
                // pick a random station TODO: higher probability of choosing station with more point
                var station = searchSpace.Values.ToList()[Instance.Randomizer.NextInt(searchSpace.Values.Count)];
                // pick a point from the station's search space
                var point = station.Pick(Instance.Randomizer.NextDouble());
                // skip if pod exist
                if (solution.Values.Any(s => s.point.pod == point.pod))
                    continue;

                var success = updateSolution(point);

                // end iteration if all solution has small difference
                if(solution.Values.All(s => s.difference < _config.minDifference))
                    break;

                // decrease temperature
                temperature *= _config.coolingRate;
            }
        }


        /// <summary>
        /// Update the solution with the point, if its item throughput rate is acceptable
        /// </summary>
        /// <param name="point"></param>
        /// <returns>success</returns>
        private bool updateSolution(SearchSpace.Point point)
        {
            // only add point when pod doesn't exist in solution or there are no solution
            if (solution.Values.Count > 0 && solution.Values.Any(so => so.point.pod == point.pod)) return false;
            Solution sol = null;
            sol = createSolution(point);
            if (sol == null) return false;

            // accept if there are no solution for the station
            if (!solution.ContainsKey(point.station))
            {
                sol.difference =double.MaxValue;
                solution.Add(point.station, sol);
                return true;
            }
            else // Change the solution of the station when rate is higher or acceptable according to the temperature
            {
                sol.difference = solution[point.station].stationRate - sol.stationRate;
                if (sol.difference < 0 || Math.Exp(-sol.difference/temperature) > Instance.Randomizer.NextDouble())
                {
                    solution[point.station] = sol;
                    return true;
                }
                 return false;
            }
        }

        /// <summary>
        /// Find orders in points' order but fulfilled
        /// 1. not in solution of other station
        /// 2. consider remained item in pod and station
        /// </summary>
        /// <param name="point"></param>
        /// <returns>
        /// null, if
        /// 1. no possible order.
        /// 2. can't find path in window.
        /// </returns>
        private Solution createSolution(SearchSpace.Point point)
        {
            // can't create solution if pod is already in solution
            if(solution.Values.Any(so => so.point.pod == point.pod)) return null;

            // Helpers
            var station = point.station;
            var otherSolution = solution.Values.Where(s => s.point.pod != point.pod);
            var possibleOrders = point.orders.Except(otherSolution.SelectMany(s => s.orders)).Except(selectedOrders);

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
            // estimate time of: bot -> pod
            if(!pathManager.schedulePath(out double endTime, s.startTime,s.bot, s.bot.TargetWaypoint, point.pod.Waypoint, false, true))
                return null; // failed to find path in window

            // Add time of lifting pod
            endTime += Instance.LayoutConfig.PodTransferTime; 
            
            // estimate time of: pod->station
            if(!pathManager.schedulePath(out endTime, endTime, s.bot, point.pod.Waypoint, s.station.Waypoint, true, false))
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
                                    {
                                        System.Console.WriteLine($"order: {string.Join(", ", order.Positions.Select(o=> $"{o.Key.ID}({o.Value})"))}");
                                        foreach(var p in pendingPods[oStation])
                                        {
                                            System.Console.WriteLine($"pod {p.ID}: {string.Join(",", p.ItemDescriptionsContained.Select(i => $"{i.ID}({p.CountAvailable(i)})"))}");
                                        }
                                        foreach(var r in request)
                                        {
                                            System.Console.WriteLine($"request of inbound pod {r.Key.ID}: {string.Join(", ", r.Value.Select(r => r.Item.ID))}");
                                        }
                                        throw new Exception($"New pod {pod.ID} in pod set without assigning any extract request");
                                    }
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