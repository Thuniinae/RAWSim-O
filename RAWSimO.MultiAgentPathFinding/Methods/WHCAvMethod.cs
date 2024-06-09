using RAWSimO.MultiAgentPathFinding.Algorithms.AStar;
using RAWSimO.MultiAgentPathFinding.DataStructures;
using RAWSimO.MultiAgentPathFinding.Elements;
using RAWSimO.MultiAgentPathFinding.Physic;
using RAWSimO.MultiAgentPathFinding.Toolbox;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;

namespace RAWSimO.MultiAgentPathFinding.Methods
{
    /// <summary>
    /// A* Approach for the Multi Agent Path Finding Problem
    /// </summary>
    public class WHCAvStarMethod : PathFinder
    {

        /// <summary>
        /// The length of a wait step
        /// </summary>
        public double LengthOfAWindow = 20.0;

        /// <summary>
        /// Abort the search at first conflict
        /// </summary>
        public bool AbortAtFirstConflict = true;

        /// <summary>
        /// Indicates whether the method uses a deadlock handler.
        /// </summary>
        public bool UseDeadlockHandler = true;

        /// <summary>
        /// The RRA* Searches
        /// </summary>
        public Dictionary<int, ReverseResumableAStar> rraStars;

        /// <summary>
        /// Reservation Table
        /// </summary>
        public ReservationTable _reservationTable;
        public ReservationTable scheduledTable {get; private set;}

        public Dictionary<int, List<ReservationTable.Interval>> scheduledPath {get; private set;}

        /// <summary>
        /// Sequence (latest to earliest) of the scheduled path.
        /// </summary>
        public List<int> scheduleSequence {get; private set;}
        public Dictionary<int, int> AgentPriorities {get; private set;}

        /// <summary>
        /// The calculated reservations
        /// </summary>
        private Dictionary<int, List<ReservationTable.Interval>> _calculatedReservations;

        /// <summary>
        /// The deadlock handler
        /// </summary>
        private DeadlockHandler _deadlockHandler;

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="graph">graph</param>
        /// <param name="seed">The seed to use for the randomizer.</param>
        /// <param name="logger">The logger to use.</param>
        public WHCAvStarMethod(Graph graph, int seed, PathPlanningCommunicator logger)
            : base(graph, seed, logger)
        {
            if (graph.BackwardEdges == null)
                graph.GenerateBackwardEgdes();
            rraStars = new Dictionary<int, ReverseResumableAStar>();
            _reservationTable = new ReservationTable(graph);
            if (UseDeadlockHandler)
                _deadlockHandler = new DeadlockHandler(graph, seed);
            _calculatedReservations = new();
            AgentPriorities = new();
        }

        /// <summary>
        /// Find the path for all the agents.
        /// </summary>
        /// <param name="agents">agents</param>
        /// <param name="queues">The queues for the destination, starting with the destination point.</param>
        /// <param name="nextReoptimization">The next re-optimization time.</param>
        /// <returns>
        /// paths
        /// </returns>
        /// <exception cref="System.Exception">Here I have to do something</exception>
        public override void FindPaths(double currentTime, List<Agent> agents)
        {
            Stopwatch.Restart();

            // Priorities of the agents
            var agentPrios = agents.ToDictionary(agent => agent.ID, agent => 0);

            for (int retry = 1; true; retry++)
            {
                var found = _findPaths(currentTime, agents, agentPrios, Stopwatch.ElapsedMilliseconds / 1000.0 > RuntimeLimitPerAgent * agents.Count * 0.7, Math.Min(RuntimeLimitPerAgent * agents.Count, RunTimeLimitOverall));
                if (found)
                    break;

                if (Stopwatch.ElapsedMilliseconds / 1000.0 > RuntimeLimitPerAgent * agents.Count * 0.9 || Stopwatch.ElapsedMilliseconds / 1000.0 > RunTimeLimitOverall)
                {
                    Communicator.SignalTimeout();
                    return;
                }
            }

        }

        /// <summary>
        /// Find the path for all the agents.
        /// </summary>
        /// <param name="agents">agents</param>
        /// <param name="obstacleWaypoints">The way points of the obstacles.</param>
        /// <param name="queues">The queues for the destination, starting with the destination point.</param>
        /// <param name="nextReoptimization">The next re-optimization time.</param>
        /// <returns>
        /// paths
        /// </returns>
        /// <exception cref="System.Exception">Here I have to do something</exception>
        private bool _findPaths(double currentTime, List<Agent> agents, Dictionary<int, int> agentPrios, bool lastRun, double runtimeLimit)
        {
            var conflictFree = true;

            //Reservation Table
            _reservationTable.Clear();
            var fixedBlockage = AgentInfoExtractor.getStartBlockage(agents, currentTime);

            SortAgents(ref agents, agentPrios);

            //set fixed blockage
            foreach (var interval in fixedBlockage.Values.SelectMany(d => d))
                _reservationTable.Add(interval);

            //deadlock handling
            if (UseDeadlockHandler)
            {
                _deadlockHandler.LengthOfAWaitStep = LengthOfAWaitStep;
                _deadlockHandler.MaximumWaitTime = 30;
                _deadlockHandler.Update(agents, currentTime);
            }

            //optimize Path
            foreach (var agent in agents.Where(a => !a.FixedPosition))
            {

                if (Stopwatch.ElapsedMilliseconds / 1000.0 > runtimeLimit * 0.9)
                {
                    Communicator.SignalTimeout();
                    return true;
                }

                //Create RRA* search if necessary.
                //Necessary if the agent has none or the agents destination has changed
                ReverseResumableAStar rraStar;
                if (!rraStars.TryGetValue(agent.ID, out rraStar) || rraStar.StartNode != agent.DestinationNode ||
                    UseDeadlockHandler && _deadlockHandler.IsInDeadlock(agent, currentTime)) // TODO this last expression is used to set back the state of the RRA* in case of a deadlock - this is only a hotfix
                {
                    rraStars[agent.ID] = new ReverseResumableAStar(Graph, agent, agent.Physics, agent.DestinationNode);
                }

                //search my path to the goal
                var aStar = new SpaceTimeAStar(Graph, LengthOfAWaitStep, currentTime + LengthOfAWindow, _reservationTable, agent, rraStars[agent.ID]);

                //the agent with a higher priority has to wait so that the others can go out of the way
                aStar.WaitStepsBeforeStart = (int)(Math.Pow(2, agentPrios[agent.ID]) / 2.0);

                //execute
                var found = aStar.Search();

                if (!found)
                {
                    conflictFree = false;

                    //fall back => ignore the collisions
                    agentPrios[agent.ID]++;
                    if (!lastRun)
                    {
                        if (!AbortAtFirstConflict)
                            continue;
                        else
                            return false;
                    }
                }

                //+ WHCA* Nodes
                List<ReservationTable.Interval> reservations;
                if (found)
                {
                    aStar.GetPathAndReservations(ref agent.Path, out reservations);
                    // only for scheduling
                    _calculatedReservations[agent.ID] = reservations;
                    foreach (var reservation in reservations)
                        _reservationTable.Add(reservation);
                    //add reservation to infinity
                    var lastNode = (_calculatedReservations[agent.ID].Count > 0) ? _calculatedReservations[agent.ID][_calculatedReservations[agent.ID].Count - 1].Node : agent.NextNode;
                    var lastTime = (_calculatedReservations[agent.ID].Count > 0) ? _calculatedReservations[agent.ID][_calculatedReservations[agent.ID].Count - 1].End : currentTime;
                    _calculatedReservations[agent.ID].Add(new ReservationTable.Interval(lastNode, lastTime, double.PositiveInfinity));
                    try
                    {
                        _reservationTable.Add(lastNode, lastTime, double.PositiveInfinity);
                    }
                    catch (DisjointIntervalTree.IntervalIntersectionException)
                    {
                        //This could technically fail => especially when they come from a station
                    }
                }

                //+ RRA* Nodes
                if (aStar.GoalNode >= 0)
                    rraStars[agent.ID].addPath(agent.Path, aStar.NodeTo2D(aStar.GoalNode));

                //add the next node again
                if (fixedBlockage.Count > 0 && (agent.Path.Count == 0 || agent.Path.NextAction.Node != agent.NextNode || agent.Path.NextAction.StopAtNode == false))
                    agent.Path.AddFirst(agent.NextNode, true, 0);

                //next time ready?
                if (agent.Path.Count == 0)
                    rraStars[agent.ID] = null;

                //deadlock?
                if (UseDeadlockHandler)
                    if (_deadlockHandler.IsInDeadlock(agent, currentTime))
                        _deadlockHandler.RandomHop(agent);
            }

            return conflictFree;
        }
        /// <summary>
        /// Find single path to the goal within the window and ending time of a bot, using current reservation table.
        /// </summary>
        /// <param name="endTime">Ending time of the bot reach the goal or reach the end of the window.</param>
        /// <param name="currentTime">The time when the path finding start from.</param>
        /// <param name="agent">Agent.Path is the path within window.</param>
        /// <returns>false, if no path can be found.</returns>
        public bool findPath(out double endTime, double currentTime, Agent agent)
        {
            if (agent != null) 
            {
                //Create RRA* search if necessary.
                //Necessary if the agent has none or the agents destination has changed
                ReverseResumableAStar rraStar;
                if (!rraStars.TryGetValue(agent.ID, out rraStar) || rraStar.StartNode != agent.DestinationNode ||
                    UseDeadlockHandler && _deadlockHandler.IsInDeadlock(agent, currentTime)) // TODO this last expression is used to set back the state of the RRA* in case of a deadlock - this is only a hotfix
                {
                    rraStars[agent.ID] = new ReverseResumableAStar(Graph, agent, agent.Physics, agent.DestinationNode);
                }

                //search my path to the goal (within time window)
                var aStar = new SpaceTimeAStar(Graph, LengthOfAWaitStep, currentTime + LengthOfAWindow, _reservationTable, agent, rraStars[agent.ID]);
                var found = aStar.Search();
                if(found) 
                {
                    List<ReservationTable.Interval> reservations;
                    aStar.GetPathAndReservations(ref agent.Path, out reservations);
                    endTime = reservations.Last().End;
                    return true;
                }
            }
            // failed to find path
            endTime = double.MaxValue;
            return false;
        }

        /// <summary>
        /// Sorts the agents.
        /// </summary>
        /// <param name="agents">The agents.</param>
        /// <param name="queues">The queues.</param>
        private void SortAgents(ref List<Agent> agents, Dictionary<int, int> agentPrios)
        {
            agents = agents.OrderByDescending(a => agentPrios[a.ID]).ThenBy(a => a.CanGoThroughObstacles ? 1 : 0).ThenBy(a => Graph.getDistance(a.NextNode, a.DestinationNode)).ToList();
        }
                /// <summary>
        /// Initialization of scheduling paths based on current reservation table. 
        /// Scheduled paths will not affect real reservation table.
        /// </summary>
        public void scheduleInit()
        {
            // copy reservation table
            scheduledTable = _reservationTable.DeepCopy();
            scheduledPath = new();
            scheduleSequence = new();
        }
        /// <summary>
        /// Find path based on schedule table, will add path to schedule table if success.
        /// </summary>
        /// <param name="overwrite">input true to overwrite previous schedule path of the agent</param>
        /// <returns>false, if can't find path</returns>
        public bool schedulePath(out double endTime, ref List<ReservationTable.Interval> path, double startTime, Agent agent)
        {
            if (agent != null) 
            {
                // init schedule path of the agent
                if(!scheduledPath.ContainsKey(agent.ID))
                {
                    if(_calculatedReservations.ContainsKey(agent.ID))
                        scheduledPath[agent.ID] = new List<ReservationTable.Interval>(_calculatedReservations[agent.ID]);
                    else
                        scheduledPath[agent.ID] = new();
                    // remove reservation of the starting point, because WHCAn* will reserve the ending waypoint of the existed path of the bot
                    var interval = scheduledTable.Get(agent.NextNode, startTime, startTime + LengthOfAWindow);
                    if(interval != null) scheduledTable.Remove(interval); // since only scheduling, no need to add back
                }

                //Create RRA* search if necessary.
                //Necessary if the agent has none or the agents destination has changed
                ReverseResumableAStar rraStar;
                if (!rraStars.TryGetValue(agent.ID, out rraStar) || rraStar.StartNode != agent.DestinationNode ||
                    UseDeadlockHandler && _deadlockHandler.IsInDeadlock(agent, startTime)) // TODO this last expression is used to set back the state of the RRA* in case of a deadlock - this is only a hotfix
                {
                    rraStars[agent.ID] = new ReverseResumableAStar(Graph, agent, agent.Physics, agent.DestinationNode);
                }

                // ignore the bot's path for now
                scheduledTable.CarefulRemoves(scheduledPath[agent.ID]);
                // consider extra path
                scheduledTable.Add(path);

                //search my path to the goal (within time window)
                var aStar = new SpaceTimeAStar(Graph, LengthOfAWaitStep, startTime + LengthOfAWindow, scheduledTable, agent, rraStars[agent.ID]);
                var found = aStar.Search();

                // remove extra path
                scheduledTable.CarefulRemoves(path);
                // add ignore bot's path back
                scheduledTable.Add(scheduledPath[agent.ID]);

                if(found) 
                {
                    List<ReservationTable.Interval> reservations;
                    aStar.GetPathAndReservations(ref agent.Path, out reservations);
                    endTime = reservations.Last().End;
                    path.AddRange(reservations);
                    return true;
                }
            }
            // failed to find path
            endTime = double.MaxValue;
            return false;
        }
        /// <summary>
        /// Initialization of scheduling paths based on current reservation table. 
        /// Scheduled paths will not affect real reservation table.
        /// </summary>
        public void OverwriteScheduledPath(int ID,  List<ReservationTable.Interval> path)
        {
            // remove previous scheduled path
            scheduledTable.CarefulRemoves(scheduledPath[ID]);
            // add new scheduled path
            scheduledPath[ID] = path;
            scheduledTable.Add(path);
            // store the priority of the bot
            if(scheduleSequence.Contains(ID))
                scheduleSequence.Remove(ID);
            scheduleSequence.Insert(0, ID);
        }
        /// <summary>
        /// Find the starting time of the last reservation of a point if the point
        /// is the ending point of a reserved path.
        /// </summary>
        /// <returns>false, if the input point does not have reservation to infinity.</returns>
        virtual public bool findEndReservation(out double startTime, int node)
        {
            var interval  = _reservationTable.GetLast(node);
            if (interval == null || !double.IsInfinity(interval.End))
            {
                startTime = -1;
                return false;
            }
            else
            { 
                startTime = interval.Start;
                return true;
            }
        }

        /// <summary>
        /// Update the priority of the agent used in FindPaths.
        /// </summary>
        public void UpdateAgentPriority(int ID, int priority)
        {
            AgentPriorities[ID] = priority;
        }
    }
}
