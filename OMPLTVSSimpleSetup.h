/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_EXTENSION_OMPLTVS_SIMPLE_SETUP_
#define OMPL_EXTENSION_OMPLTVS_SIMPLE_SETUP_

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include "OMPLTVSStateValidityChecker.h"
#include "OMPLTVSStatePropagator.h"
#include "OMPLTVSControlSpace.h"

namespace ompl
{

    namespace control
    {
        class OMPLTVSGoalRegion : public base::GoalRegion
        {
        public:
            OMPLTVSGoalRegion(const base::SpaceInformationPtr &si, double x, double y, double z, double threshold);
            virtual double distanceGoal(const base::State *st) const;
        protected:
            double x_, y_, z_;
        };

        /** \brief Create the set of classes typically needed to solve a
            control problem when forward propagation is computed with OMPLTVS. */
        class OMPLTVSSimpleSetup : public SimpleSetup
        {
        public:

            /** \brief Constructor needs the control space needed for planning. */
            explicit
            OMPLTVSSimpleSetup(const ControlSpacePtr &space);

            /** \brief The control space is assumed to be OMPLTVSControlSpace. Constructor only needs the state space. */
            explicit
            OMPLTVSSimpleSetup(const base::StateSpacePtr &space);

            /** \brief The control space is assumed to be
                OMPLTVSControlSpace. The state space is assumed to
                be OMPLTVSStateSpace. Constructor only needs the OMPLTVS
                environment. */
            explicit
            OMPLTVSSimpleSetup(const OMPLTVSEnvironmentPtr &env);

            virtual ~OMPLTVSSimpleSetup()
            {
            }

            /** \brief Get the OMPLTVS environment associated to the state and control spaces */
            const OMPLTVSEnvironmentPtr& getEnvironment() const
            {
                return getStateSpace()->as<OMPLTVSStateSpace>()->getEnvironment();
            }

            /** \brief Get the current OMPLTVS state (read parameters from OMPLTVS bodies) */
            base::ScopedState<OMPLTVSStateSpace> getCurrentState() const;

            /** \brief Set the current OMPLTVS state (set parameters for OMPLTVS bodies) */
            void setCurrentState(const base::ScopedState<> &state);

            /** \brief Set the current OMPLTVS state (set parameters for OMPLTVS bodies) */
            void setCurrentState(const base::State *state);

            /** \brief Set the bounds for the planning volume */
            void setVolumeBounds(const base::RealVectorBounds &bounds)
            {
                getStateSpace()->as<OMPLTVSStateSpace>()->setVolumeBounds(bounds);
            }

            /** \brief Set the bounds for the linear velocity */
            void setLinearVelocityBounds(const base::RealVectorBounds &bounds)
            {
                getStateSpace()->as<OMPLTVSStateSpace>()->setLinearVelocityBounds(bounds);
            }

            /** \brief Set the bounds for the angular velocity */
            void setAngularVelocityBounds(const base::RealVectorBounds &bounds)
            {
                getStateSpace()->as<OMPLTVSStateSpace>()->setAngularVelocityBounds(bounds);
            }

            /** \brief Set the OMPLTVS world to the states that are
                contained in a given path, sequentially. Using \e
                timeFactor, the speed at which this sequence is
                iterated through is altered. */
            void playPath(const base::PathPtr &path, double timeFactor = 1.0) const;

            /** \brief Call playPath() on the solution path, if one is available */
            void playSolutionPath(double timeFactor = 1.0) const;

            /** \brief Simulate the OMPLTVS environment forward for \e steps simulation steps, using the control \e control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(int control, unsigned int steps) const;

            /** \brief Simulate the OMPLTVS environment forward for \e steps simulation steps, using the control \e control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(const Control *control, unsigned int steps) const;

            /** \brief Simulate the OMPLTVS environment forward for \e
                steps simulation steps, using the null control
                (ompl::control::ControlSpace::nullControl()).
                Construct a path representing this action. */
            base::PathPtr simulate(unsigned int steps) const;

            inline void setGoalRegion(double x, double y, double z, double threshold) {
                setGoal(base::GoalPtr(new OMPLTVSGoalRegion(getSpaceInformation(), x, y, z, threshold)));
            }
            
            virtual void setup();

        private:

            void useEnvParams();

        };
    }

}
#endif
