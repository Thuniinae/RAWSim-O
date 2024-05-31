using RAWSimO.Core.Bots;
using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Helper;
using RAWSimO.Core.Interfaces;
using RAWSimO.Core.Metrics;
using RAWSimO.Core.Waypoints;
using RAWSimO.MultiAgentPathFinding;
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
    }
}
