using RAWSimO.Core.Bots;
using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Helper;
using RAWSimO.Core.Interfaces;
using RAWSimO.Core.Metrics;
using RAWSimO.Core.Waypoints;
using RAWSimO.MultiAgentPathFinding;
using RAWSimO.MultiAgentPathFinding.DataStructures;
using RAWSimO.MultiAgentPathFinding.Elements;
using RAWSimO.MultiAgentPathFinding.Methods;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.Core.Control.Defaults.PathPlanning
{

    /// <summary>
    /// Controller of the bot.
    /// </summary>
    public class WHCAvStarPathManager : PathManager
    {
        /// <summary>
        /// Priorities and the corresponding task of the bots. 
        /// </summary>
        private Dictionary<int, int> botsPriority;
        /// <summary>
        /// The corresponding task of the bots' priority. 
        /// </summary>
        private Dictionary<int, BotTask> botsTask;
        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="instance">instance</param>
        public WHCAvStarPathManager(Instance instance)
            : base(instance)
        {
            //Need a Request on Fail
            BotNormal.RequestReoptimizationAfterFailingOfNextWaypointReservation = true;

            //translate to lightweight graph
            var graph = GenerateGraph();
            var config = instance.ControllerConfig.PathPlanningConfig as WHCAvStarPathPlanningConfiguration;

            PathFinder = new WHCAvStarMethod(graph, instance.SettingConfig.Seed, new PathPlanningCommunicator(
                instance.LogSevere,
                instance.LogDefault,
                instance.LogInfo,
                instance.LogVerbose,
                () => { instance.StatOverallPathPlanningTimeouts++; }));
            var method = PathFinder as WHCAvStarMethod;
            method.LengthOfAWaitStep = config.LengthOfAWaitStep;
            method.RuntimeLimitPerAgent = config.RuntimeLimitPerAgent;
            method.RunTimeLimitOverall = config.RunTimeLimitOverall;
            method.LengthOfAWindow = config.LengthOfAWindow;
            method.AbortAtFirstConflict = config.AbortAtFirstConflict;
            method.UseDeadlockHandler = config.UseDeadlockHandler;

            if (config.AutoSetParameter)
            {
                //best parameter determined my master thesis
                method.LengthOfAWindow = 20;
                method.RuntimeLimitPerAgent = config.Clocking / instance.Bots.Count;
                method.RunTimeLimitOverall = config.Clocking;
            }
            botsPriority = new();
            botsTask = new();
        }
        /// <summary>
        /// Find single path to the goal within the window and estimated ending time of a bot, using current reservation table.
        /// </summary>
        /// <param name="endTime">ending time of the bot reach the goal or reach the end of the window.</param>
        /// <param name="bot"></param>
        /// <param name="currentTime"></param>
        /// <param name="startWaypoint"></param>
        /// <param name="endWaypoint"></param>
        /// <param name="carryingPod"></param>
        /// <returns>false, if no path can be found.</returns>
        override public bool findPath(out double endTime, double currentTime,
            Bot bot, Waypoint startWaypoint, Waypoint endWaypoint, bool carryingPod)
        {
            Agent agent;
            getBotAgent(out agent, bot, currentTime, startWaypoint, endWaypoint, carryingPod);
            var method = PathFinder as WHCAvStarMethod;
            var success =  method.findPath(out endTime, currentTime, agent);
            if (success){
                // estimated travel time of path outside of WHCA* window
                var waypoint = bot.Instance.Controller.PathManager.GetWaypointByNodeId(agent.Path.LastAction.Node);
                if(carryingPod)
                    endTime += Distances.CalculateShortestTimePathPodSafe(waypoint, endWaypoint, Instance);
                else
                    endTime += Distances.CalculateShortestTimePath(waypoint, endWaypoint, Instance);
                // TODO: add penalty for possible collision
            }
            return success;
        }
        /// <summary>
        /// Find path based on schedule table, will add path to schedule table if success.
        /// </summary>
        /// <returns>false, if can't find path</returns>
        override public bool schedulePath(out double endTime, ref List<ReservationTable.Interval> path, double currentTime, Bot bot, Waypoint startWaypoint, Waypoint endWaypoint, bool carryingPod)
        {
            Agent agent;
            getBotAgent(out agent, bot, currentTime, startWaypoint, endWaypoint, carryingPod);
            var method = PathFinder as WHCAvStarMethod;
            var success = method.schedulePath(out endTime, ref path, currentTime, agent);
            if (success){
                // estimated travel time of path outside of WHCA* window
                var waypoint = bot.Instance.Controller.PathManager.GetWaypointByNodeId(agent.Path.LastAction.Node);
                if(carryingPod)
                    endTime += Distances.EstimateManhattanTime(waypoint, endWaypoint, Instance);
                else
                    endTime += Distances.EstimateManhattanTime(waypoint, endWaypoint, Instance);
                // TODO: add penalty for possible collision
            }
            return success;
        }
        /// <summary>
        /// Initialization of scheduling paths based on current reservation table. 
        /// Scheduled paths will not affect real reservation table.
        /// </summary>
        override public void scheduleInit()
        {
            var method = PathFinder as WHCAvStarMethod;
            method.scheduleInit();
        }

        /// <summary>
        /// Overwrite a bot's scheduled path
        /// </summary>
        override public void OverwriteScheduledPath(Bot bot,  List<ReservationTable.Interval> path)
        {
            var method = PathFinder as WHCAvStarMethod;
            method.OverwriteScheduledPath(bot.ID, path);
        }

        /// <summary>
        /// Find the arrival time of a bot in the reservation table.
        /// </summary>
        /// <returns>false, if arrival time can't be found</returns>
        override public bool FindArrivalTime(out double startTime, Bot bot)
        {
            var method = PathFinder as WHCAvStarMethod;
            var node = GetNodeIdByWaypoint(bot.TargetWaypoint);
            return method.findEndReservation(out startTime, node);
        }

        /// <summary>
        /// Add reservation to the schedule table
        /// </summary>
        override public void AddSchedule(List<ReservationTable.Interval> path)
        {
            var method = PathFinder as WHCAvStarMethod;
            method.scheduledTable.Add(path);
        }

        /// <summary>
        /// Remove reservation from the schedule table
        /// </summary>
        override public void RemoveSchedule(List<ReservationTable.Interval> path)
        {
            var method = PathFinder as WHCAvStarMethod;
            method.scheduledTable.CarefulRemoves(path);
        }
        /// <summary>
        /// Get the schedule path of the bot. 
        /// </summary>
        override public List<ReservationTable.Interval> GetSchedulePath(Bot bot)
        {
            var method = PathFinder as WHCAvStarMethod;
            return method.scheduledPath[bot.ID];
        }
        /// <summary>
        /// Modify the priority of bot in path planning with the priority in schedule. 
        /// </summary>
        override public void OutputScheduledPriority(Dictionary<Bot, BotTask> _botsTask)
        {
            var method = PathFinder as WHCAvStarMethod;
            // off set previous bots priority as the number of new bots task
            foreach(var id in botsPriority.Keys)
                botsPriority[id] += _botsTask.Count;
            // priority start from 1, to separate from other bots which has priority 0
            foreach(var (bot, task) in _botsTask.Select(d => (d.Key, d.Value)))
            {
                int priority = method.scheduleSequence.FindIndex(i => i == bot.ID) + 1; 
                botsPriority[bot.ID] = priority;
                botsTask[bot.ID] = task;
            }
        }

        /// <summary>
        /// Update bots' priority from previous output of scheduling. 
        /// Bot only get priority (> 0), when bot's current task match the task in scheduled. 
        /// </summary>
        override public void UpdateBotPriorities(List<Bot> bots)
        {
            var method = PathFinder as WHCAvStarMethod;
            // offset priority to start from 1
            if(botsPriority.Count > 0)
            {
                var minPriority = botsPriority.Values.Min() - 1;
                if(minPriority > 0)
                    foreach(var id in botsPriority.Keys)
                        botsPriority[id] -= minPriority;
            }

            // set priority of bots according to schedule and their task
            foreach(var bot in bots)
            {
                if(!botsPriority.ContainsKey(bot.ID)) continue;
                // only use priority when having same task
                if(botsTask[bot.ID] == bot.CurrentTask) 
                    method.UpdateAgentPriority(bot.ID, botsPriority[bot.ID]);
                else
                    method.UpdateAgentPriority(bot.ID, 0);
            }
        }
    }
}
