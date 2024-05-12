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

namespace RAWSimO.Core.Control.Defaults.OrderBatching
{
    /// <summary>
    /// Implements a manager that uses information of the backlog to exploit similarities in orders when assigning them.
    /// </summary>
    public class FullySuppliedOrderManager : OrderManager
    {
        /// <summary>
        /// Creates a new instance of this manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public FullySuppliedOrderManager(Instance instance) : base(instance) 
        { 
            _config = instance.ControllerConfig.OrderBatchingConfig as FullySuppliedOrderBatchingConfiguration; 
        }

        /// <summary>
        /// The config of this controller.
        /// </summary>
        private FullySuppliedOrderBatchingConfiguration _config;

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
        { return station.Active && station.CapacityReserved + station.CapacityInUse < station.Capacity - 1; }

        private BestCandidateSelector _bestCandidateSelectNormal;
        private BestCandidateSelector _bestCandidateSelectFastLane;
        private OutputStation _currentStation = null;
        private Order _currentOrder = null;
        private VolatileIDDictionary<OutputStation, Pod> _nearestInboundPod;

        /// <summary>
        /// Initializes this controller.
        /// </summary>
        private void Initialize()
        {
            // Set some values for statistics
            _statFullySuppliedScoreIndex = 0;
            // --> Setup normal scorers
            List<Func<double>> normalScorers = new List<Func<double>>();
            // Select best by match with inbound pods
            /*normalScorers.Add(() =>
            {
                return _currentOrder.Positions.Sum(line => Math.Min(_currentStation.InboundPods.Sum(pod => pod.CountAvailable(line.Key)), line.Value));
            });*/
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

        /// <summary>
        /// This is called to decide about potentially pending orders.
        /// This method is being timed for statistical purposes and is also ONLY called when <code>SituationInvestigated</code> is <code>false</code>.
        /// Hence, set the field accordingly to react on events not tracked by this outer skeleton.
        /// </summary>
        protected override void DecideAboutPendingOrders()
        {
            // do nothing
        }

        /// <summary>
        /// This is called to decide about potentially pending orders.
        /// This method is being timed for statistical purposes and is also ONLY called when <code>SituationInvestigated</code> is <code>false</code>.
        /// Hence, set the field accordingly to react on events not tracked by this outer skeleton.
        /// </summary>
        public void FullySupplied(OutputStation station, HashSet<Order> undecidedOrders)
        {
            // If not initialized, do it now
            if (_bestCandidateSelectNormal == null)
                Initialize();

            Dictionary<Pod, List<ExtractRequest>> requests = 
                new(station.InboundPods.Select(p => new KeyValuePair<Pod, List<ExtractRequest>>(p, new())));

            // remained items in station and pod
            var stationRemain = new Dictionary<Pod, Dictionary<ItemDescription, int>>(
                station.InboundPods.ToDictionary(p => p, 
                p => p.ItemDescriptionsContained.ToDictionary(i => i, i => p.CountAvailable(i))));

            // Do until can't find any order or station full
            while(station.CapacityInUse + station.CapacityReserved < station.Capacity)
            {
                _bestCandidateSelectNormal.Recycle();
                // Set station
                _currentStation = station;
                // Prepare helpers
                OutputStation chosenStation = null;
                Order chosenOrder = null;
                // Search for best order for the station in all orders that can be fulfilled by the stations inbound pods
                foreach (var order in undecidedOrders)
                {
                    if (order.Positions.Any(p => 
                        stationRemain.Values.Sum(d => d.ContainsKey(p.Key)? d[p.Key]: 0) < p.Value))
                        continue;
                    // Set order
                    _currentOrder = order;
                    // --> Assess combination
                    if (_bestCandidateSelectNormal.Reassess())
                    {
                        chosenStation = _currentStation;
                        chosenOrder = _currentOrder;
                    }
                }
                // Assign best order if available
                if (chosenOrder != null)
                {
                    // Assign the order
                    AllocateOrder(chosenOrder, chosenStation);
                    // remove from order list
                    undecidedOrders.Remove(chosenOrder);
                    // calculate extract request
                    var requireRequests = Instance.ResourceManager.GetExtractRequestsOfOrder(chosenOrder).
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
                                requests[pod].AddRange(requireRequests[item].GetRange(0, takeStation));
                                requireRequests[item].RemoveRange(0, takeStation);
                            }
                        }
                    }
                    // check if the order can be fulfilled
                    if (requireRequests.Values.Any(v => v.Count > 0))
                        throw new Exception("Order selected can't be fulfilled");
                    // Log score statistics
                    if (_statScorerValues == null)
                        _statScorerValues = _bestCandidateSelectNormal.BestScores.ToArray();
                    else
                        for (int i = 0; i < _bestCandidateSelectNormal.BestScores.Length; i++)
                            _statScorerValues[i] += _bestCandidateSelectNormal.BestScores[i];
                    _statAssignments++;
                    Instance.StatCustomControllerInfo.CustomLogOB2 = _statScorerValues[_statFullySuppliedScoreIndex] / _statAssignments;
                }
                else
                    break; // no more assignment for this station.
            }
            // register all items in orders to inbound pods
            foreach(var pod in requests.Keys)
            {
                foreach(var r in requests[pod])
                {
                    pod.RegisterItem(r.Item, r);
                }
                requests.Remove(pod);
            }
            // check if all station requests are registered
            if (requests.Keys.Count > 0)
                throw new Exception("Remained unregister items");
        }

        /// <summary>
        /// Fully-Supplied POA that will allocate orders, by a Fully-Demand pod that are about to assigned to the station.
        /// Notes: 1. orders decided will be remove from undecided orders. 
        /// 2. This method is not being timed, should only be called in other timed function (ex: Pod Selection)
        /// </summary>
        /// <param name="station"></param>
        /// <param name="newPod"></param>
        /// <param name="undecidedOrders"></param>
        /// <returns>A list of extract request for the new pod</returns>
        public List<ExtractRequest> ExtraDecideAboutPendingOrders(OutputStation station, Pod newPod, HashSet<Order> undecidedOrders, List<Order> possibleOrders)
        {
            // If not initialized, do it now
            if (_bestCandidateSelectNormal == null)
                Initialize();

            List<ExtractRequest> requests = new();
            Dictionary<Pod, List<ExtractRequest>> stationRequests = 
                new(station.InboundPods.Select(p => new KeyValuePair<Pod, List<ExtractRequest>>(p, new())));
            // remained items in station and pod
            var stationRemain = new Dictionary<Pod, Dictionary<ItemDescription, int>>(
                station.InboundPods.ToDictionary(p => p, 
                p => p.ItemDescriptionsContained.ToDictionary(i => i, i => p.CountAvailable(i))));
            var podRemain = new Dictionary<ItemDescription, int>(
                newPod.ItemDescriptionsContained.ToDictionary(i =>i, i => newPod.CountAvailable(i)));

            // Do until can't find any order or station full
            while(station.CapacityInUse + station.CapacityReserved < station.Capacity)
            {
                _bestCandidateSelectNormal.Recycle();
                // Set station
                _currentStation = station;
                // Prepare helpers
                OutputStation chosenStation = null;
                Order chosenOrder = null;
                // Search for best order for the station in all orders that can be fulfilled by the stations inbound pods
                foreach (var order in undecidedOrders)
                {
                    if (order.Positions.Any(p => 
                    stationRemain.Values.Sum(d => d.ContainsKey(p.Key)? d[p.Key]: 0)
                    + (podRemain.ContainsKey(p.Key)? podRemain[p.Key] : 0) < p.Value))
                        continue;
                    // Set order
                    _currentOrder = order;
                    // --> Assess combination
                    if (_bestCandidateSelectNormal.Reassess())
                    {
                        chosenStation = _currentStation;
                        chosenOrder = _currentOrder;
                    }
                }
                // Assign best order if available
                if (chosenOrder != null)
                {
                    // Assign the order
                    AllocateOrder(chosenOrder, chosenStation);
                    // remove from order list
                    undecidedOrders.Remove(chosenOrder);
                    // calculate extract request
                    var requireRequests = Instance.ResourceManager.GetExtractRequestsOfOrder(chosenOrder).
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
                                stationRequests[pod].AddRange(requireRequests[item].GetRange(0, takeStation));
                                requireRequests[item].RemoveRange(0, takeStation);
                            }
                        }
                        // then take from pod
                        if(podRemain.ContainsKey(item))
                        {
                            int takePod = Math.Min(requireRequests[item].Count, podRemain[item]);
                            podRemain[item] -= takePod;
                            requests.AddRange(requireRequests[item].GetRange(0, takePod));
                            requireRequests[item].RemoveRange(0, takePod);
                        }
                    }
                    // check if the order can be fulfilled
                    if (requireRequests.Values.Any(v => v.Count > 0))
                        throw new Exception("Order selected can't be fulfilled");
                    // check number of remaining item in pod and station
                    if (podRemain.Values.Any(v => v < 0) || stationRemain.Values.Any(v => v.Values.Any(v1 => v1 < 0)))
                        throw new Exception("Remain negative number of item in pod or station.");
                    // Log score statistics
                    if (_statScorerValues == null)
                        _statScorerValues = _bestCandidateSelectNormal.BestScores.ToArray();
                    else
                        for (int i = 0; i < _bestCandidateSelectNormal.BestScores.Length; i++)
                            _statScorerValues[i] += _bestCandidateSelectNormal.BestScores[i];
                    _statAssignments++;
                    Instance.StatCustomControllerInfo.CustomLogOB2 = _statScorerValues[_statFullySuppliedScoreIndex] / _statAssignments;
                }
                else
                    break; // no more assignment for this station.
            }
            // register all items in orders to inbound pods
            foreach(var pod in station.InboundPods)
            {
                foreach(var r in stationRequests[pod])
                {
                    pod.RegisterItem(r.Item, r);
                }
                //stationRequests.Remove(pod);
            }
            if (requests.Count == 0){
                foreach(var order in possibleOrders)
                    System.Console.WriteLine($"order: {string.Join(", ", order.Positions.Select(o=> $"{o.Key.ID}({o.Value})"))}");
                foreach(var pod in station.InboundPods)
                {
                    System.Console.WriteLine($"request: {string.Join(", ", stationRequests[pod].Select(r => r.Item.ID))}");
                }
                System.Console.WriteLine($"pod {newPod.ID}: {string.Join(",", newPod.ItemDescriptionsContained.Select(i => $"{i.ID}({newPod.CountAvailable(i)})"))}");
                throw new Exception($"No order can be assigned to pod {newPod.ID}!");
            }
            return requests;
        }
        /// <summary>
        /// Fully-Supplied POA when pod-set is needed to fulfilled a single order.
        /// Notes: 1. necessary order will be remove from undecided orders. 
        /// 2. This method is not being timed, should only be called in other timed function (ex: Pod Selection)
        /// </summary>
        /// <param name="station"></param>
        /// <param name="newPods"></param>
        /// <param name="undecidedOrders"></param>
        /// <param name="necessaryOrder"></param>
        /// <returns>Dictionary of pods' extract Request.</returns>
        public Dictionary<Pod, List<ExtractRequest>> ExtraDecideAboutPendingOrder(OutputStation station, List<Pod> newPods, HashSet<Order> undecidedOrders, Order necessaryOrder)
        {
            // If not initialized, do it now
            if (_bestCandidateSelectNormal == null)
                Initialize();
            // Get some meta info
            PrepareAssessment();
            // only for score statistics
            _currentOrder = necessaryOrder;
            _currentStation = station;
            _bestCandidateSelectNormal.Reassess();

            // Init
            Dictionary<Pod, List<ExtractRequest>> requests = 
                new(newPods.Select(p => new KeyValuePair<Pod, List<ExtractRequest>>(p, new())));

            if (necessaryOrder != null)
            {
                // Assign the order
                AllocateOrder(necessaryOrder, station);
                // remove from order list
                undecidedOrders.Remove(necessaryOrder);
                // calculate extract request
                var requireRequests = Instance.ResourceManager.GetExtractRequestsOfOrder(necessaryOrder).
                    GroupBy(r => r.Item).ToDictionary(g => g.Key, g => g.ToList());
                
                foreach(var item in requireRequests.Keys)
                {
                    // take from station first
                    foreach(var pod in station.InboundPods)
                    {
                        int takeStation = Math.Min(requireRequests[item].Count, pod.CountAvailable(item));
                        // register items in the order to the inbound pod
                        foreach(var r in requireRequests[item].GetRange(0, takeStation))
                            pod.RegisterItem(r.Item, r);
                        requireRequests[item].RemoveRange(0, takeStation);
                    }
                    // then take from new pods
                    foreach(var pod in newPods)
                    {
                        int takePod = Math.Min(requireRequests[item].Count, pod.CountAvailable(item));
                        requests[pod].AddRange(requireRequests[item].GetRange(0, takePod));
                        requireRequests[item].RemoveRange(0, takePod);
                    }
                }
                
                // check if there are remain
                if (requireRequests.Any(d => d.Value.Count > 0))
                    throw new Exception("necessary order can't be fulfilled by new pod set and available item in inbound pod to the station");
                
                // Log score statistics
                if (_statScorerValues == null)
                    _statScorerValues = _bestCandidateSelectNormal.BestScores.ToArray();
                else
                    for (int i = 0; i < _bestCandidateSelectNormal.BestScores.Length; i++)
                        _statScorerValues[i] += _bestCandidateSelectNormal.BestScores[i];
                _statAssignments++;
                Instance.StatCustomControllerInfo.CustomLogOB2 = _statScorerValues[_statFullySuppliedScoreIndex] / _statAssignments;
            }
            else
                throw new Exception("necessary order is null!");
            return requests;
        }
        /// <summary>
        /// Immediately submits the order to the station.
        /// </summary>
        /// <param name="order">The order that is going to be allocated.</param>
        /// <param name="station">The station the order is assigned to.</param>
        public void ExtraAllocateOrder(Order order, OutputStation station)
        {
            AllocateOrder(order, station);
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
        private int _statFullySuppliedScoreIndex = -1;
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
