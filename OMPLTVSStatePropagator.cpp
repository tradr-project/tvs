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

#include "OMPLTVSStatePropagator.h"
#include "OMPLTVSStateSpace.h"
#include "OMPLTVSControlSpace.h"
#include <ompl/util/Exception.h>
#include <ompl/util/Console.h>

ompl::control::OMPLTVSStatePropagator::OMPLTVSStatePropagator(const SpaceInformationPtr &si) : StatePropagator(si)
{
    if (OMPLTVSStateSpace *oss = dynamic_cast<OMPLTVSStateSpace*>(si->getStateSpace().get()))
        env_ = oss->getEnvironment();
    else
        throw Exception("OMPLTVS State Space needed for OMPLTVSStatePropagator");
}

void ompl::control::OMPLTVSStatePropagator::propagate(const base::State *state, const Control *control, const double duration, base::State *result) const
{
    env_->mutex_.lock();

    // place the OMPLTVS world at the start state
    si_->getStateSpace()->as<OMPLTVSStateSpace>()->writeState(state);

    // apply the controls
    env_->applyControl(control->as<RealVectorControlSpace::ControlType>()->values);

    bool collision = false;
    for(double t = 0; t < duration; t += env_->env_->stepSize) {
        collision = env_->env_->step() || collision;
        if(collision) break;
    }

    // read the final state from the OMPLTVS world
    si_->getStateSpace()->as<OMPLTVSStateSpace>()->readState(result);

#if 1
    const dReal *a = state->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    const dReal *b = result->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    std::cout
    << control->as<RealVectorControlSpace::ControlType>()->values[0] << " "
    << control->as<RealVectorControlSpace::ControlType>()->values[1] << " "
    << "   "
    << duration
    << "   "
    << a[0] << " " << a[1] << " " << a[2]
    << "   "
    << b[0] << " " << b[1] << " " << b[2]
    << std::endl;
#endif
    
    env_->addToSearchTree(state, result); // vor visualization purposes
    
    env_->mutex_.unlock();

    // update the collision flag for the start state, if needed
    if (!(state->as<OMPLTVSStateSpace::StateType>()->collision & (1 << OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT)))
    {
        if (collision)
            state->as<OMPLTVSStateSpace::StateType>()->collision &= (1 << OMPLTVSStateSpace::STATE_COLLISION_VALUE_BIT);
        state->as<OMPLTVSStateSpace::StateType>()->collision &= (1 << OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT);
    }
}

bool ompl::control::OMPLTVSStatePropagator::canPropagateBackward() const
{
    return false;
}
