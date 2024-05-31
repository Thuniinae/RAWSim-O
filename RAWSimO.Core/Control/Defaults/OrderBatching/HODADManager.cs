using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.IO;
using RAWSimO.Core.Items;
using RAWSimO.Core.Management;
using RAWSimO.Core.Metrics;
using RAWSimO.Toolbox;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static RAWSimO.Core.Control.BotManager;

namespace RAWSimO.Core.Control.Defaults.OrderBatching
{

    /// <summary>
    /// Implements a manager that uses information of the backlog to exploit similarities in orders when assigning them.
    /// </summary>
    public class HODADManager : OrderManager
    {
        // --->>> BEST CANDIDATE HELPER FIELDS - USED FOR SELECTING THE NEXT BEST TASK
        /// <summary>
        /// The current pod to assess.
        /// </summary>
        private Pod _currentPod = null;
        /// <summary>
        /// The current output station to assess
        /// </summary>
        private OutputStation _currentOStation = null;
        /// <summary>
        /// 
        /// </summary>
        public double SumofNumberofDuetime = 0.0;
        /// <summary>
        /// 临时存储_pendingOrders1
        /// </summary>
        HashSet<Order> _pendingOrders1 = null;
        /// <summary>
        /// Creates a new instance of this manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public HODADManager(Instance instance) : base(instance) { _config = instance.ControllerConfig.OrderBatchingConfig as HADODConfiguration; }
        /// <summary>
        /// Stores the available counts per SKU for a pod for on-the-fly assessment.
        /// </summary>
        private VolatileIDDictionary<ItemDescription, int> _availableCounts;
        /// <summary>
        /// 开始执行分配决策的工作站空闲容量的阈值
        /// </summary>
        private int _ThresholdValue = 1;
        /// <summary>
        /// Initializes some fields for pod selection.
        /// </summary>
        private void InitPodSelection()
        {
            if (_availableCounts == null)
                _availableCounts = new VolatileIDDictionary<ItemDescription, int>(Instance.ItemDescriptions.Select(i => new VolatileKeyValuePair<ItemDescription, int>(i, 0)).ToList());
        }
        /// <summary>
        /// The config of this controller.
        /// </summary>
        private HADODConfiguration _config;
        /// <summary>
        /// order进入Od的截止时间
        /// </summary>
        private double DueTimeOrderofMP = TimeSpan.FromMinutes(30).TotalSeconds;
        /// <summary>
        /// Checks whether another order is assignable to the given station.
        /// </summary>
        /// <param name="station">The station to check.</param>
        /// <returns><code>true</code> if there is another open slot, <code>false</code> otherwise.</returns>
        private bool IsAssignable(OutputStation station)
        { return station.Active && station.CapacityReserved + station.CapacityInUse < station.Capacity; }
        /// <summary>
        /// Checks whether another order is assignable to the given station.
        /// </summary>
        /// <param name="station">The station to check.</param>
        /// <returns><code>true</code> if there is another open slot and another one reserved for fast-lane, <code>false</code> otherwise.</returns>
        private bool IsAssignableKeepFastLaneSlot(OutputStation station)
        { return station.Active && station.CapacityReserved + station.CapacityInUse < station.Capacity; }

        private BestCandidateSelector _bestCandidateSelectNormal;
        private BestCandidateSelector _bestCandidateSelectFastLane;
        private Order _currentOrder = null;
        private VolatileIDDictionary<OutputStation, Pod> _nearestInboundPod;
        ///// <summary>
        /////临时分配给工作站的Pods
        ///// </summary>
        //Dictionary<OutputStation, HashSet<Pod>> _inboundPodsPerStation1 = null;
        /// <summary>
        /// Initializes this controller.
        /// </summary>
        private void Initialize()
        {
            // Set some values for statistics
            _statPodMatchingScoreIndex = _config.LateBeforeMatch ? 1 : 0;
            // --> Setup normal scorers
            List<Func<double>> normalScorers = new List<Func<double>>();
            // Select late orders first
            if (_config.LateBeforeMatch)
            {
                normalScorers.Add(() =>
                {
                    return _currentOrder.DueTime > Instance.Controller.CurrentTime ? 1 : 0;
                });
            }
            // Select best by match with inbound pods
            normalScorers.Add(() =>
            {
                return -_currentOrder.Timestay;
            });
            // If we run into ties use the oldest order
            normalScorers.Add(() =>
            {
                switch (_config.TieBreaker)
                {
                    case Shared.OrderSelectionTieBreaker.Random: return Instance.Randomizer.NextDouble();
                    case Shared.OrderSelectionTieBreaker.EarliestDueTime: return -_currentOrder.DueTime;
                    case Shared.OrderSelectionTieBreaker.FCFS: return -_currentOrder.TimeStamp;
                    default: throw new ArgumentException("Unknown tie breaker: " + _config.FastLaneTieBreaker);
                }
            });
            // --> Setup fast lane scorers
            List<Func<double>> fastLaneScorers = new List<Func<double>>();
            // If we run into ties use the oldest order
            fastLaneScorers.Add(() =>
            {
                switch (_config.FastLaneTieBreaker)
                {
                    case Shared.FastLaneTieBreaker.Random: return Instance.Randomizer.NextDouble();
                    case Shared.FastLaneTieBreaker.EarliestDueTime: return -_currentOrder.DueTime;
                    case Shared.FastLaneTieBreaker.FCFS: return -_currentOrder.TimeStamp;
                    default: throw new ArgumentException("Unknown tie breaker: " + _config.FastLaneTieBreaker);
                }
            });
            // Init selectors
            _bestCandidateSelectNormal = new BestCandidateSelector(true, normalScorers.ToArray());
            _bestCandidateSelectFastLane = new BestCandidateSelector(true, fastLaneScorers.ToArray());
            if (_config.FastLane)
                _nearestInboundPod = new VolatileIDDictionary<OutputStation, Pod>(Instance.OutputStations.Select(s => new VolatileKeyValuePair<OutputStation, Pod>(s, null)).ToList());
        }
        /// <summary>
        /// Determines a score that can be used to decide about an assignment.
        /// </summary>
        /// <returns>A score that can be used to decide about the best assignment. Minimization / Smaller is better.</returns>
        public double Score()
        {
            // Check picks leading to completed orders
            int completeableAssignedOrders = 0;
            //int NumberofItem = 0;
            Dictionary<ItemDescription, int> _availableCounts1 = new Dictionary<ItemDescription, int>();
            // Get current pod content
            foreach (var pod in _inboundPodsPerStation[_currentOStation])
            {
                foreach (var item in pod.ItemDescriptionsContained)
                {
                    if (_availableCounts1.ContainsKey(item))
                        _availableCounts1[item] += pod.CountAvailable(item);
                    else
                        _availableCounts1.Add(item, pod.CountAvailable(item));
                }
            }
            // Check all assigned orders
            SumofNumberofDuetime = 0.0;
            foreach (var order in _pendingOrders)
            {
                // Get demand for items caused by order
                List<IGrouping<ItemDescription, ExtractRequest>> itemDemands = Instance.ResourceManager.GetExtractRequestsOfOrder(order).GroupBy(r => r.Item).ToList();
                // Check whether sufficient inventory is still available in the pod (also make sure it is was available in the beginning, not all values were updated at the beginning of this function / see above)
                if (itemDemands.All(g => _inboundPodsPerStation[_currentOStation].Any(v => v.IsAvailable(g.Key)) && _availableCounts1[g.Key] >= g.Count()))
                {
                    // Update remaining pod content
                    foreach (var itemDemand in itemDemands)
                    {
                        _availableCounts1[itemDemand.Key] -= itemDemand.Count();
                        //NumberofItem += itemDemand.Count();
                    }
                    // Update number of completeable orders
                    completeableAssignedOrders++;
                    SumofNumberofDuetime += order.sequence;
                }
            }
            return -(100 * completeableAssignedOrders);
        }

        /// <summary>
        /// Determines a score that can be used to decide about an assignment.
        /// </summary>
        /// <param name="pod">The pod.</param>
        /// <returns>A score that can be used to decide about the best assignment. Minimization / Smaller is better.</returns>
        public double NumItemofPod(Pod pod)
        {
            // Check picks leading to completed orders
            int completeableAssignedItems = 0;
            // Get current pod content
            foreach (var item in pod.ItemDescriptionsContained)
                _availableCounts[item] = pod.CountAvailable(item);
            // Check all assigned orders
            foreach (var order in _pendingOrders)
            {
                // Get demand for items caused by order
                List<IGrouping<ItemDescription, ExtractRequest>> itemDemands = Instance.ResourceManager.GetExtractRequestsOfOrder(order).GroupBy(r => r.Item).ToList();
                // Check whether sufficient inventory is still available in the pod (also make sure it is was available in the beginning, not all values were updated at the beginning of this function / see above)
                if (itemDemands.Any(g => pod.IsAvailable(g.Key) && _availableCounts[g.Key] > 0))
                {
                    // Update remaining pod content
                    foreach (var itemDemand in itemDemands.Where(g => pod.IsAvailable(g.Key) && _availableCounts[g.Key] > 0))
                    {
                        completeableAssignedItems += Math.Min(itemDemand.Count(), _availableCounts[itemDemand.Key]);
                        _availableCounts[itemDemand.Key] -= Math.Min(itemDemand.Count(), _availableCounts[itemDemand.Key]);
                    }
                }
            }
            return completeableAssignedItems;
        }
        /// <summary>
        /// Determines a score that can be used to decide about an assignment.
        /// </summary>
        /// <param name="config">The config specifying further parameters.</param>
        /// <param name="pod">The pod.</param>
        /// <param name="station">The station.</param>
        /// <returns>A score that can be used to decide about the best assignment. Minimization / Smaller is better.</returns>
        public double Score(PCScorerPodForOStationBotDemand config, Pod pod, OutputStation station)
        {
            return -pod.ItemDescriptionsContained.Sum(i =>
                Math.Min(
                    // Overall demand
                    Instance.ResourceManager.GetDemandAssigned(i) + Instance.ResourceManager.GetDemandQueued(i) + Instance.ResourceManager.GetDemandBacklog(i),
                    // Stock offered by pod
                    pod.CountContained(i)));
        }
        /// <summary>
        /// Prepares some meta information.
        /// </summary>
        private void PrepareAssessment()
        {
            if (_config.FastLane)
            {
                foreach (var station in Instance.OutputStations.Where(s => IsAssignable(s)))
                {
                    _nearestInboundPod[station] = station.InboundPods.ArgMin(p =>
                    {
                        if (p.Bot != null && p.Bot.CurrentWaypoint != null)
                            // Use the path distance (this should always be possible)
                            return Distances.CalculateShortestPathPodSafe(p.Bot.CurrentWaypoint, station.Waypoint, Instance);
                        else
                            // Use manhattan distance as a fallback
                            return Distances.CalculateManhattan(p, station, Instance.WrongTierPenaltyDistance);
                    });
                }
            }
        }
        internal bool AnyRelevantRequests(Pod pod)
        {
            return _pendingOrders.Any(o => Instance.ResourceManager.GetExtractRequestsOfOrder(o).Any(r => pod.IsAvailable(r.Item)) && Instance.
            ResourceManager.GetExtractRequestsOfOrder(o).GroupBy(r => r.Item).All(i => pod.CountAvailable(i.Key) >= i.Count()));
        }
        internal bool AnyRelevantRequests1(Pod pod)  //pod最少可以向_pendingOrders提供一个item
        {
            return _pendingOrders.Any(o => Instance.ResourceManager.GetExtractRequestsOfOrder(o).Any(r => pod.IsAvailable(r.Item)));
        }
        /// <summary>
        /// 产生Od, Od为快到期order的集合
        /// </summary>
        /// <param name="pendingOrders"></param>
        /// <returns></returns>
        public HashSet<Order> GenerateOd(HashSet<Order> pendingOrders)
        {
            foreach (Order order in pendingOrders)
                order.Timestay = order.DueTime - (Instance.SettingConfig.StartTime.AddSeconds(Convert.ToInt32(Instance.Controller.CurrentTime)) - order.TimePlaced).TotalSeconds;
            int i = 0;
            foreach (Order order in pendingOrders.OrderBy(v => v.Timestay).ThenBy(u => u.DueTime)) //先选剩余的截止时间最短的，再选开始时间最早的
            {
                order.sequence = i;
                i++;
            }
            HashSet<Order> Od = new HashSet<Order>();
            foreach (var order in pendingOrders.Where(v => v.Positions.Sum(line => Math.Min(Instance.ResourceManager.UnusedPods.Sum(pod => pod.CountAvailable(line.Key)), line.Value))
            == v.Positions.Sum(s => s.Value)))//保证Od中的所有order必须能被执行
            {
                if (order.Timestay < DueTimeOrderofMP)
                    Od.Add(order);
            }
            return Od;
        }
        /// <summary>
        /// Instantiates a scoring function from the given config.
        /// </summary>
        /// <param name="scorerConfig">The config.</param>
        /// <returns>The scoring function.</returns>
        private Func<double> GenerateScorerPodForOStationBot(PCScorerPodForOStationBot scorerConfig)
        {
            switch (scorerConfig.Type())
            {
                case PrefPodForOStationBot.Demand:
                    { PCScorerPodForOStationBotDemand tempcfg = scorerConfig as PCScorerPodForOStationBotDemand; return () => { return Score(tempcfg, _currentPod, _currentOStation); }; }
                case PrefPodForOStationBot.Completeable:
                    { PCScorerPodForOStationBotCompleteable tempcfg = scorerConfig as PCScorerPodForOStationBotCompleteable; return () => { return Score(); }; }
                case PrefPodForOStationBot.WorkAmount:
                    { PCScorerPodForOStationBotWorkAmount tempcfg = scorerConfig as PCScorerPodForOStationBotWorkAmount; return () => { return SumofNumberofDuetime; }; }
                default: throw new ArgumentException("Unknown score type: " + scorerConfig.Type());
            }
        }

        /// <summary>
        /// 产生Ns
        /// </summary>
        /// <returns></returns>
        public Dictionary<OutputStation, Dictionary<Pod, int>> GenerateNs()
        {
            Dictionary<OutputStation, Dictionary<Pod, int>> Ns = new Dictionary<OutputStation, Dictionary<Pod, int>>();
            //初始化Ns
            foreach (var station in Instance.OutputStations)
            {
                Dictionary<Pod, int> sequenceofpod = new Dictionary<Pod, int>();
                Ns.Add(station, sequenceofpod);
            }
            foreach (var station in Instance.OutputStations)
            {
                Dictionary<Pod, int> sequuenceofpod = new Dictionary<Pod, int>();
                IEnumerable<Pod> inboundPodsofstation = station.InboundPods;
                Dictionary<Pod, double> distenceofpod = new Dictionary<Pod, double>();
                Dictionary<Pod, double> distenceofpod1 = new Dictionary<Pod, double>();
                //已经分配而未完成拣选的pod  1 计算已经进入工作站缓冲区的pod
                foreach (Pod pod in inboundPodsofstation.Where(v => v.Bot != null && (station.Queues[station.Waypoint].Contains(v.Bot.CurrentWaypoint) || v.Bot.CurrentWaypoint == station.Waypoint)))
                    distenceofpod1.Add(pod, Distances.CalculateShortestPathPodSafe(pod.Bot.CurrentWaypoint, station.Waypoint, Instance));
                //已经分配而未完成拣选的pod  3 将已经分配而在工作站缓冲区外的的pod加入
                foreach (Pod pod in inboundPodsofstation.Where(v => !distenceofpod1.ContainsKey(v)))
                    distenceofpod.Add(pod, Distances.CalculateShortestPathPodSafe(pod.Waypoint == null ? pod.Bot.CurrentWaypoint : pod.Waypoint, station.Waypoint, Instance));

                //已经分配而未完成拣选的pod  2 将已经分配而未指派bot的pod加入
                foreach (var pod in Instance.ResourceManager._availablePodsPerStation[station])
                {
                    if (!distenceofpod.ContainsKey(pod))
                        distenceofpod.Add(pod, Distances.CalculateShortestPathPodSafe(pod.Waypoint == null ? pod.Bot.CurrentWaypoint : pod.Waypoint, station.Waypoint, Instance));
                }
                int i = 0;
                foreach (var pod in distenceofpod1.OrderBy(v => v.Value).Select(v => v.Key))
                {
                    pod.sequence = i;
                    if (!Ns[station].ContainsKey(pod))
                        Ns[station].Add(pod, i);
                    else
                        throw new InvalidOperationException("Could not any request from the selected pod!");
                    i++;
                }
                foreach (var pod in distenceofpod.OrderBy(v => v.Value).Select(v => v.Key))
                {
                    pod.sequence = i;
                    if (!Ns[station].ContainsKey(pod))
                        Ns[station].Add(pod, i);
                    else
                        throw new InvalidOperationException("Could not any request from the selected pod!");
                    i++;
                }
            }
            return Ns;
        }

        /// <summary>
        /// Returns a list of relevant items for the given pod / output-station combination.
        /// </summary>
        /// <param name="pod">The pod in focus.</param>
        /// <param name="itemDemands">The station in focus.</param>
        /// <returns>A list of tuples of items to serve the respective extract-requests.</returns>
        internal List<ExtractRequest> GetPossibleRequests(Pod pod, IEnumerable<ExtractRequest> itemDemands)
        {
            // Init, if necessary
            InitPodSelection();
            // Match fitting items with requests
            List<ExtractRequest> requestsToHandle = new List<ExtractRequest>();
            // Get current content of the pod
            foreach (var item in itemDemands.Select(r => r.Item).Distinct())
                _availableCounts[item] = pod.CountAvailable(item);
            // First handle requests already assigned to the station
            foreach (var itemRequestGroup in itemDemands.GroupBy(r => r.Item))
            {
                // Handle as many requests as possible with the given SKU
                IEnumerable<ExtractRequest> possibleRequests = itemRequestGroup.Take(_availableCounts[itemRequestGroup.Key]);
                requestsToHandle.AddRange(possibleRequests);
                // Update content available in pod for the given SKU
                _availableCounts[itemRequestGroup.Key] -= possibleRequests.Count();
            }
            // Return the result
            return requestsToHandle;
        }
        /// <summary>
        /// 候选pod
        /// </summary> 
        private BestCandidateSelector _bestPodOStationCandidateSelector = null;
        /// <summary>
        /// 已经被选择的pod集合
        /// </summary>
        public HashSet<Pod> SelectedPod = new HashSet<Pod>();
        ///// <summary>
        ///// 所有可能的组合
        ///// </summary>
        //public HashSet<HashSet<Pod>> FindSetofPod(int num)
        //{
        //    HashSet<HashSet<Pod>> Setpods = new HashSet<HashSet<Pod>>();
        //    foreach (var pod1 in Instance.ResourceManager.UnusedPods.Where(p => AnyRelevantRequests1(p) && !SelectedPod.Contains(p)))
        //    {
        //        _inboundPodsPerStation[station].Add(pod1);
        //        if (_bestPodOStationCandidateSelector.Reassess())
        //            bestPod = pod1;
        //        _inboundPodsPerStation[station].Remove(pod1);
        //    }
        //    return Setpods;
        //}
        /// <summary>
        /// POA and PPS
        /// </summary>
        /// <param name="validStationNormalAssignment"></param>
        /// <param name="CsOd"></param>
        public void HeuristicsPOAandPPS(Func<OutputStation, bool> validStationNormalAssignment, bool CsOd)
        {
            // Assign orders while possible
            bool furtherOptions = true;
            while (furtherOptions)
            {
                // Prepare helpers
                SelectedPod = new HashSet<Pod>();
                OutputStation chosenStation = null;
                // Look for next station to assign orders to
                foreach (var station in Instance.OutputStations
                    // Station has to be valid
                    .Where(s => validStationNormalAssignment(s)))
                {
                    _currentOStation = station;
                    HashSet<Pod> BestPodsbyS = new HashSet<Pod>();
                L:
                    //进行POA操作
                    bool furtherOptions1 = true;
                    while (validStationNormalAssignment(station) && furtherOptions1)
                    {
                        _bestCandidateSelectNormal.Recycle();
                        Order chosenOrder = null;
                        // Search for best order for the station in all fulfillable orders        选择的订单必须满足库存的数量约束
                        foreach (var order in _pendingOrders.Where(o => o.Positions.All(p => _inboundPodsPerStation[_currentOStation].Sum(pod => pod.CountAvailable(p.Key)) >= p.Value)))
                        {
                            // Set order
                            _currentOrder = order;
                            // --> Assess combination    可以建立一个集合
                            if (_bestCandidateSelectNormal.Reassess())  //选出可以由当前inbound中的pod分拣的完整order，tie-breaker是order的item的数量
                            {
                                chosenStation = _currentOStation;
                                chosenOrder = _currentOrder;
                            }
                        }
                        // Assign best order if available
                        if (chosenOrder != null)
                        {
                            // Assign the order
                            AllocateOrder(chosenOrder, chosenStation);
                            _pendingOrders1.Remove(chosenOrder);
                            //对pod进行排序
                            Dictionary<OutputStation, Dictionary<Pod, int>> Ns = GenerateNs();
                            //对pod中的item进行标记
                            // Match fitting items with requests
                            List<ExtractRequest> requestsToHandleofAll = new List<ExtractRequest>();
                            IEnumerable<ExtractRequest> itemDemands = Instance.ResourceManager.GetExtractRequestsOfOrder(chosenOrder);
                            Instance.LogVerbose($"Allocate order:  {string.Join(", ", chosenOrder.Positions.Select(p => $"{p.Key.ID}({p.Value})"))} at {chosenOrder.TimeStamp}");
                            int i = 0;
                            // Get current content of the pod
                            if (BestPodsbyS.Count == 0)
                            {
                                Instance.LogVerbose("pod set empty!");
                                foreach (var pod in Ns[chosenStation].OrderBy(v => v.Value).Select(s => s.Key))
                                {
                                    Instance.LogVerbose($"Pod {pod.ID}: {string.Join(", ", pod.ItemDescriptionsContained.Select(i => $"{i.ID}({pod.CountAvailable(i)})"))}");
                                    if (itemDemands.Where(v => !requestsToHandleofAll.Contains(v)).Any(g => pod.IsAvailable(g.Item)))
                                    {
                                        // Get all fitting requests
                                        List<ExtractRequest> fittingRequests = GetPossibleRequests(pod, itemDemands.Where(v => !requestsToHandleofAll.Contains(v)));
                                        requestsToHandleofAll.AddRange(fittingRequests);
                                        // Update remaining pod content
                                        foreach (var fittingRequest in fittingRequests)
                                        {
                                            pod.JustRegisterItem(fittingRequest.Item); //将pod中选中的item进行标记   
                                            i++;
                                        }
                                        if (Instance.ResourceManager._Ziops1[chosenStation].ContainsKey(pod))
                                            Instance.ResourceManager._Ziops1[chosenStation][pod].AddRange(fittingRequests);
                                        else
                                            Instance.ResourceManager._Ziops1[chosenStation].Add(pod, fittingRequests);
                                        Instance.LogVerbose($"Register items for pod {pod.ID} in inbound pods: {string.Join(", ", fittingRequests.Select(r => r.Item.ID))} from {string.Join(", ", chosenOrder.Positions.Select(p => $"{p.Key.ID}({p.Value})"))}");
                                    }
                                }
                            }
                            else
                            {
                                Instance.LogVerbose("pod set not empty");
                                foreach (var pod in BestPodsbyS)
                                {
                                    if (itemDemands.Where(v => !requestsToHandleofAll.Contains(v)).Any(g => pod.IsAvailable(g.Item)))
                                    {
                                        // Get all fitting requests
                                        List<ExtractRequest> fittingRequests = GetPossibleRequests(pod, itemDemands.Where(v => !requestsToHandleofAll.Contains(v)));
                                        requestsToHandleofAll.AddRange(fittingRequests);
                                        // Update remaining pod content
                                        foreach (var fittingRequest in fittingRequests)
                                        {
                                            pod.JustRegisterItem(fittingRequest.Item); //将pod中选中的item进行标记   
                                            i++;
                                        }
                                        if (Instance.ResourceManager._Ziops1[chosenStation].ContainsKey(pod))
                                            Instance.ResourceManager._Ziops1[chosenStation][pod].AddRange(fittingRequests);
                                        else
                                            Instance.ResourceManager._Ziops1[chosenStation].Add(pod, fittingRequests);
                                        Instance.LogVerbose($"Register items for pod {pod.ID} in pod set: {string.Join(", ", fittingRequests.Select(r => r.Item.ID))}");
                                    }
                                }
                                BestPodsbyS.Clear();
                            }

                            if (i != itemDemands.Count())
                            {
                                Instance.LogVerbose("Deal with remained items");
                                foreach (var pod in Ns[chosenStation].OrderBy(v => v.Value).Select(s => s.Key))
                                {
                                    if (itemDemands.Where(v => !requestsToHandleofAll.Contains(v)).Any(g => pod.IsAvailable(g.Item)))
                                    {
                                        // Get all fitting requests
                                        List<ExtractRequest> fittingRequests = GetPossibleRequests(pod, itemDemands.Where(v => !requestsToHandleofAll.Contains(v)));
                                        requestsToHandleofAll.AddRange(fittingRequests);
                                        // Update remaining pod content
                                        foreach (var fittingRequest in fittingRequests)
                                        {
                                            pod.JustRegisterItem(fittingRequest.Item); //将pod中选中的item进行标记   
                                            i++;
                                        }
                                        if (Instance.ResourceManager._Ziops1[chosenStation].ContainsKey(pod))
                                            Instance.ResourceManager._Ziops1[chosenStation][pod].AddRange(fittingRequests);
                                        else
                                            Instance.ResourceManager._Ziops1[chosenStation].Add(pod, fittingRequests);
                                        Instance.LogVerbose($"Register items for pod {pod.ID} in inbound pods: {string.Join(", ", fittingRequests.Select(r => r.Item.ID))}");
                                    }
                                }
                                throw new InvalidOperationException("order 无法满足!");
                            }
                        }
                        else
                            furtherOptions1 = false;
                    }
                    //进行PPS操作
                    if (validStationNormalAssignment(station) && (CsOd || _pendingOrders.Count() > 0))
                    {
                        _bestPodOStationCandidateSelector.Recycle();
                        Pod BestPod = null;
                        foreach (var pod in Instance.ResourceManager.UnusedPods.Where(p => AnyRelevantRequests1(p) && !SelectedPod.Contains(p)))
                        {
                            _inboundPodsPerStation[station].Add(pod);
                            if (_bestPodOStationCandidateSelector.Reassess())
                                BestPod = pod;
                            _inboundPodsPerStation[station].Remove(pod);
                        }
                        if (BestPod == null)
                            _pendingOrders = SimplePOAandPPS(station, ref BestPodsbyS);
                        else
                        {
                            _inboundPodsPerStation[station].Add(BestPod);
                            Instance.ResourceManager._availablePodsPerStation[station].Add(BestPod);
                            SelectedPod.Add(BestPod);
                        }
                        goto L;
                    }
                    else if (validStationNormalAssignment(station) && _pendingOrders.Count == 0 && !CsOd)
                    {
                        _pendingOrders = _pendingOrders1;
                        CsOd = true;
                        goto L;
                    }
                }
                furtherOptions = false;
            }
        }
        /// <summary>
        /// 生成PiSKU
        /// </summary>
        /// <param name="order"></param>
        /// <returns></returns>
        public List<List<HashSet<Pod>>> GeneratePiSKU(Order order)
        {
            Dictionary<ItemDescription, HashSet<Pod>> piSKU = new Dictionary<ItemDescription, HashSet<Pod>>();
            List<List<HashSet<Pod>>> PiSKU = new List<List<HashSet<Pod>>>();
            HashSet<ItemDescription> setofitem = new HashSet<ItemDescription>();
            foreach (var item in order.Positions)
            {
                setofitem.Add(item.Key);
                foreach (var pod in Instance.ResourceManager.UnusedPods.Where(v => v.IsAvailable(item.Key))) //&& !SelectedPod.Contains(v)
                {
                    if (piSKU.ContainsKey(item.Key))
                    {
                        if (!piSKU[item.Key].Contains(pod))
                            piSKU[item.Key].Add(pod);
                    }
                    else
                    {
                        HashSet<Pod> listofpod = new HashSet<Pod>() { pod };
                        piSKU.Add(item.Key, listofpod);
                    }
                }
                //foreach (var station in Instance.OutputStations)
                //{
                //    foreach (var pod in Instance.ResourceManager._Ziops[station].Select(v => v.Key).Distinct().Where(v => v.IsAvailable(item.Key)))
                //    {
                //        if (piSKU.ContainsKey(item.Key))
                //        {
                //            if (!piSKU[item.Key].Contains(pod))
                //                piSKU[item.Key].Add(pod);
                //        }
                //        else
                //        {
                //            HashSet<Pod> listofpod = new HashSet<Pod>() { pod };
                //            piSKU.Add(item.Key, listofpod);
                //        }
                //    }
                //}
                Pod[] arr = piSKU[item.Key].ToArray();
                List<HashSet<Pod>> listofpod1 = new List<HashSet<Pod>>();
                //int numpfpodlist = 1;//任意一个初始值
                //int length = arr.Length;
                for (int i = 1; i < arr.Length + 1; i++)
                {
                    //求组合
                    List<Pod[]> lst_Combination = FindPodSet<Pod>.GetCombination(arr, i); //得到所有的货架组合
                    foreach (var podlist in lst_Combination.Where(v => v.Sum(p => p.CountAvailable(item.Key)) >= _availableCounts[item.Key]).OrderBy(s => s.Length))
                        listofpod1.Add(podlist.ToHashSet());
                    if (listofpod1.Count > 0)
                        break;
                    //{
                    //    if (listofpod1.Count  == 0)
                    //        numpfpodlist = podlist.Length;
                    //    if (podlist.Length > numpfpodlist && listofpod1.Count>0 && !order.Positions.Where(u => !setofitem.Contains(u.Key)).Select(w => w.Key).Any(o => podlist.Any(v => v.IsAvailable(o))))
                    //        continue;
                    //    else
                    //        listofpod1.Add(podlist.ToHashSet());
                    //    if (numpfpodlist == podlist.Length && !order.Positions.Where(u => !setofitem.Contains(u.Key)).Select(w => w.Key).Any(o => podlist.Any(v => v.IsAvailable(o))))
                    //        piSKU[item.Key].RemoveWhere(v => podlist.Contains(v));
                    //}
                    //if (piSKU[item.Key].Count < 2)
                    //    break;
                    //else
                    //    arr= piSKU[item.Key].ToArray();
                }
                PiSKU.Add(listofpod1);
            }
            return PiSKU;
        }
        /// <summary>
        /// 根据确定的order来选择相应的pod
        /// </summary>
        /// <param name="station"></param>
        /// <param name="BestPods"></param>
        /// <returns></returns>
        public HashSet<Order> SimplePOAandPPS(OutputStation station, ref HashSet<Pod> BestPods)
        {
            HashSet<Order> selectedoforder = new HashSet<Order>(); //表示选择的pod集合
            int cs = Cs[station];
            while (cs > 0 && _pendingOrders.Count > 0)
            {
                Order order = _pendingOrders.Where(v => v.Positions.Sum(line => Math.Min(Instance.ResourceManager.UnusedPods.Sum(pod => pod.CountAvailable(line.Key)), line.Value)) ==
                v.Positions.Sum(s => s.Value)).OrderBy(u=>u.sequence).FirstOrDefault();  //选择优先级最高并且满足库存需求的一个order
                _pendingOrders.Remove(order);
                selectedoforder.Add(order);
                cs--;
                //Dictionary<ItemDescription, int> availableCountsoforder = new Dictionary<ItemDescription, int>();
                foreach (var item in order.Positions)
                {
                    _availableCounts[item.Key] = item.Value;
                    //availableCountsoforder.Add(item.Key, item.Value);
                    //if (_inboundPodsPerStation[station].Sum(p => p.CountAvailable(item.Key)) > 0)
                    //{
                    //    _availableCounts[item.Key] -= Math.Min(_inboundPodsPerStation[station].Sum(p => p.CountAvailable(item.Key)), item.Value);//减掉_inboundPodsPerStation中已有的item数量
                    //    availableCountsoforder[item.Key] -= Math.Min(_inboundPodsPerStation[station].Sum(p => p.CountAvailable(item.Key)), item.Value);//减掉_inboundPodsPerStation中已有的item数量
                    //}                       
                }
                List<List<HashSet<Pod>>> PiSKU = new List<List<HashSet<Pod>>>();
                PiSKU = GeneratePiSKU(order); //生成满足order中每个item需求的pod集合
                var result = PiSKU.Skip(1).Aggregate(PiSKU.First().StartCombo(), (serials, current) => serials.Combo(current), x => x).ToList();//生成满足order需求的pod集合
                if (PiSKU.Select(v => v.Count).Aggregate((av, e) => av * e) != result.Count)
                    throw new InvalidOperationException("Could not any request from the selected pod!");
                HashSet<HashSet<Pod>> hashset = new HashSet<HashSet<Pod>>();
                foreach (var sets in result)
                {
                    HashSet<Pod> hashgset = new HashSet<Pod>();
                    foreach (var set in sets)
                    {
                        foreach (var st in set)
                            hashgset.Add(st);
                    }
                    hashset.Add(hashgset.Distinct().ToHashSet());
                }
                BestPods = hashset.Where(u => order.Positions.All(s => u.Sum(p => p.CountAvailable(s.Key)) >= _availableCounts[s.Key])).OrderBy(v => v.Count()).FirstOrDefault();//去重后选择pod数量最少的集合
                if (BestPods.Count == 0)
                    throw new InvalidOperationException("Could not any request from the selected pod!");
                //暂时是随机选择第一个，后续可改为选择可完成order最多的pod集合
                foreach (var pod in BestPods.Where(v => !_inboundPodsPerStation[station].Contains(v)))
                {
                    _inboundPodsPerStation[station].Add(pod);
                    Instance.ResourceManager._availablePodsPerStation[station].Add(pod);
                    SelectedPod.Add(pod);
                }
            }
            return selectedoforder;

        }
        /// <summary>
        /// This is called to decide about potentially pending orders.
        /// This method is being timed for statistical purposes and is also ONLY called when <code>SituationInvestigated</code> is <code>false</code>.
        /// Hence, set the field accordingly to react on events not tracked by this outer skeleton.
        /// </summary>
        protected override void DecideAboutPendingOrders()
        {
            //获取每个station的可用容量
            GenerateCs();
            if (!Cs.Any(v => v.Value > _ThresholdValue - 1))
                return;
            if (Instance.ControllerConfig.TaskAllocationConfig.GetType() != typeof(BalancedTaskAllocationConfiguration))
                throw new Exception("HODAD must be used with Balanced Task Allocation!");
            var balancedTAConfig = Instance.ControllerConfig.TaskAllocationConfig as BalancedTaskAllocationConfiguration;
            if (balancedTAConfig.PodSelectionConfig.GetType() != typeof(HADODPodSelectionConfiguration))
                throw new Exception("HODAD must be used with HADOD Pod Selection!");
            var podSelectConfig = balancedTAConfig.PodSelectionConfig as HADODPodSelectionConfiguration;
                                   
            // If not initialized, do it now
            if (_bestCandidateSelectNormal == null)
                Initialize();
            // Init
            InitPodSelection();
            foreach (var oStation in Instance.OutputStations)
                _inboundPodsPerStation[oStation] = new HashSet<Pod>(oStation.InboundPods);
            if (_bestPodOStationCandidateSelector == null)
            {
                _bestPodOStationCandidateSelector = new BestCandidateSelector(false,
                    GenerateScorerPodForOStationBot(podSelectConfig.OutputPodScorer),
                    GenerateScorerPodForOStationBot(podSelectConfig.OutputPodScorerTieBreaker1));
            }
            // Define filter functions
            Func<OutputStation, bool> validStationNormalAssignment = _config.FastLane ? (Func<OutputStation, bool>)IsAssignableKeepFastLaneSlot : IsAssignable;
            Func<OutputStation, bool> validStationFastLaneAssignment = IsAssignable;
            //Od为快到期order的集合
            HashSet<Order> Od = GenerateOd(_pendingOrders);
            //用于存储_pendingOrders的临时数据
            _pendingOrders1 = new HashSet<Order>(_pendingOrders);
            // Assign fast lane orders while possible
            if (Od.Count == 0)
                HeuristicsPOAandPPS(validStationNormalAssignment, true);
            else if (Cs.Sum(v => v.Value) > Od.Count && Od.Count > 0)
            {
                _pendingOrders = Od;
                HeuristicsPOAandPPS(validStationNormalAssignment, false);
                _pendingOrders = _pendingOrders1;
            }
            else if (Cs.Sum(v => v.Value) <= Od.Count && Od.Count > 0)
            {
                _pendingOrders = Od;
                HeuristicsPOAandPPS(validStationNormalAssignment, true);
                //DeletePod();
                _pendingOrders = _pendingOrders1;
            }
        }

        #region IOptimize Members

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public override void SignalCurrentTime(double currentTime) { /* Ignore since this simple manager is always ready. */ }

        #endregion

        #region Custom stat tracking

        /// <summary>
        /// Contains the aggregated scorer values.
        /// </summary>
        private double[] _statScorerValues = null;
        /// <summary>
        /// Contains the number of assignments done.
        /// </summary>
        private double _statAssignments = 0;
        /// <summary>
        /// The index of the pod matching scorer.
        /// </summary>
        private int _statPodMatchingScoreIndex = -1;
        /// <summary>
        /// The callback indicates a reset of the statistics.
        /// </summary>
        public override void StatReset()
        {
            _statScorerValues = null;
            _statAssignments = 0;
        }
        /// <summary>
        /// The callback that indicates that the simulation is finished and statistics have to submitted to the instance.
        /// </summary>
        public override void StatFinish()
        {
            Instance.StatCustomControllerInfo.CustomLogOBString =
                _statScorerValues == null ? "" :
                string.Join(IOConstants.DELIMITER_CUSTOM_CONTROLLER_FOOTPRINT.ToString(), _statScorerValues.Select(e => e / _statAssignments).Select(e => e.ToString(IOConstants.FORMATTER)));
        }

        #endregion
    }
}
