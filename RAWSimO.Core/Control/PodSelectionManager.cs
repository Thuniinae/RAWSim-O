﻿using System;
using System.Collections.Generic;
using System.Linq;
using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Interfaces;
using RAWSimO.Core.Items;
using RAWSimO.Toolbox;

namespace RAWSimO.Core.Control
{
    #region PodSelectionManager

    /// <summary>
    /// Implements the core order manager functionality.
    /// </summary>
    public abstract class PodSelectionManager : IUpdateable, IOptimize, IStatTracker
    {
        #region Constructor

        /// <summary>
        /// Creates a new order manager.
        /// </summary>
        /// <param name="instance">The instance this order manager belongs to.</param>
        protected PodSelectionManager(Instance instance)
        {
            Instance = instance;
        }

        #endregion Constructor

        #region Fields
        /// <summary>
        /// The instance this manager is assigned to.
        /// </summary>
        protected Instance Instance { get; set; }

        #endregion Fields

        #region IUpdateable Members

        /// <summary>
        /// The next event when this element has to be updated.
        /// </summary>
        /// <param name="currentTime">The current time of the simulation.</param>
        /// <returns>The next time this element has to be updated.</returns>
        public virtual double GetNextEventTime(double currentTime) { return double.PositiveInfinity; }
        /// <summary>
        /// Updates the element to the specified time.
        /// </summary>
        /// <param name="lastTime">The time before the update.</param>
        /// <param name="currentTime">The time to update to.</param>
        public abstract void Update(double lastTime, double currentTime);
        #endregion

        #region IOptimize Members

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public abstract void SignalCurrentTime(double currentTime);

        #endregion

        #region IStatTracker Members

        /// <summary>
        /// The callback that indicates that the simulation is finished and statistics have to submitted to the instance.
        /// </summary>
        public virtual void StatFinish() { /* Default case: do not flush any statistics */ }

        /// <summary>
        /// The callback indicates a reset of the statistics.
        /// </summary>
        public virtual void StatReset() { /* Default case: nothing to reset */ }

        #endregion
    }

    #endregion PodSelectionManager
}
