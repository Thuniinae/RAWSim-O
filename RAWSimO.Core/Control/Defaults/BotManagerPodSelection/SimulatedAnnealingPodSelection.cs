using System;
using System.Collections.Generic;
using System.Linq;
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

        private double lastUpdateTime = -1.0;

        /// <summary>
        /// 
        /// </summary>
        public override void Update(double lastTime, double currentTime)
        {
            // only update once per update period
            if (lastUpdateTime + _config.updatePeriod < currentTime) return;
            lastUpdateTime = currentTime;

            // initialize the temperature

            // Assumptions:
            // 1. pod selection with single pod is most case (>90%), thus pod-set assignment is deal with by HADOD

            // Prerequisite:
            // 1. Fully-supplied station with orders using inbound pods
            // 2. triggered situation: 
                // 1. station with capacity, what if there is only one?-> find best priority of the pod
                // 2. need to have unused bot for station or need to know ending time of the bot
                // 3. every second (maybe will be unnecessary if  other trigger situation is well set)
                // 4.  more than ? stations has capacity left or hasn't has bot pre-plan (to increase planning efficiency)
            // 3. need to know the task allocation of bots (or just assume rest possible is at middle, which often happens when there are lots of bots)
            // 4. station need to have at least ond pod that can fully-fulfilled an order
            // 5. penalty time of possible collision
            // 6. need to be able to preplan bot to pod selection to preplan pod selection for each station even when there are no pod available, 
            // can make use of inbound pod, increase pile on , and make use of station capacity, therefore
            // SA should be executed when there are more than ? stations has capacity left or hasn't has bot pre-plan
                // 1. problem is bot may be reallocated -> stop bot from resting
                // 2. need know next bot available for the station

            // Inputs: 
            // 1. location of unused pods
            // 2. order backlog
            // 3. Initial temperature: make sure first few iteration can surely be worse
            // 4. smallest bot available time of each station (to estimate new pod arrival time)

            // For each station with remained capacity, find
            // 1. 50% possible pods by nearest distance
            // 2. 50% possible pods by items in Fully-fulfilled orders, with consideration of inbound pods

            // Initialization:
            // 1. Fill each station with one(?) pod and order, what if no order can be assigned to a station?
            // 2. Fill station not being able to assign an order with longest stay time orders

            // Iterations:
            // 1. Generate new solution (orders allocated, pod selected, pod priority)
                // 1. Decide explore(2.1) or exploit(3.1) (?%)
                // 2.1 random pick a station (station hasn't pick has higher chance, then station with less possible pod)
                // 2.2 random(distribution?, can let pod with higher item throughput rate has higher probability) pick a pod in search space of the station
                // 2.3 if the selected pod is already assigned to other station, then back to 2.2
                // 2.4 skip 3
                    //(new pod has lowest priority, for the ease of calculating path time?)
                // 3.1 go to 2.1 if no pod bypass (don't have shortest path) (is this necessary?)
                // 3.2 swap priority of a pod which bypass at least once with the pod it by pass 
                // (doesn't this only improve efficiency of path planning, which already been researched)
                // 4. replan all new(fit with WHCAn*, which only replan with, should be better assuming old pod movement has higher priority) pod
                // (as the number of output station, should be fast)
                // 4.alternative or estimated pod arrival time by collision (only needed, if 4 has poor result or spend too much time)
            // 2. Calculate new solution with sum of item throughput rate of picking all items of all stations
            // 3. if exp((new - current item throughput) / T) > random(0, 1) , current = new solution
            // 4. Decrease temperature
            // 5. if temperature large enough go to 1.
            // 6. end

            // Outputs:
            // 1. pod selection with extract request (will be executed by allocated bot of the stations)
            // 2. orders allocation
            // 3. pod priority
            // 4. path planning result (will be difficult)
        }
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
                                return scores[pod].Item2 / (Math.Max(endTime-Instance.Controller.CurrentTime, 
                                                                        oStation.InboundPods.Where(p => p.Bot != null && p.Bot.IsQueueing)
                                                                                            .Sum(p => p.CountRegisterItems()) * Instance.LayoutConfig.ItemPickTime) 
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