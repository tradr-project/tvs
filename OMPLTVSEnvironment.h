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

#ifndef OMPL_EXTENSION_OMPLTVS_ENVIRONMENT_
#define OMPL_EXTENSION_OMPLTVS_ENVIRONMENT_

#include "Environment.h"

#include <ompl/config.h>
#include <ompl/util/ClassForward.h>

#include <ode/ode.h>
#include <vector>
#include <boost/thread/mutex.hpp>

namespace ompl
{
    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::OMPLTVSEnvironment */
        OMPL_CLASS_FORWARD(OMPLTVSEnvironment);
        /// @endcond

        /** \class ompl::control::OMPLTVSEnvironmentPtr
            \brief A boost shared pointer wrapper for ompl::control::OMPLTVSEnvironment */

        /** \brief This class contains the OMPLTVS constructs OMPL needs to know about when planning. */
        class OMPLTVSEnvironment
        {
        public:
            Environment *env_;
            
            /** \brief The simulation step size */
            double                stepSize_;

            /** \brief The maximum number of times a control is applies in sequence */
            unsigned int          maxControlSteps_;

            /** \brief The minimum number of times a control is applies in sequence */
            unsigned int          minControlSteps_;

            /** \brief Lock to use when performing simulations in the world. (OMPLTVS simulations are NOT thread safe) */
            mutable boost::mutex  mutex_;

            OMPLTVSEnvironment(Environment *env);

            virtual ~OMPLTVSEnvironment();

            /** \brief Number of parameters (double values) needed to specify a control input */
            virtual unsigned int getControlDimension() const;

            /** \brief Get the control bounds -- the bounding box in which to sample controls */
            virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;

            /** \brief Application of a control. This function sets
                the forces/torques/velocities for bodies in the
                simulation based on control inputs.*/
            virtual void applyControl(const double *control) const;

            /** \brief Decide whether a collision is a valid one or
                not. In some cases, collisions between some bodies can
                be allowed. By default, this function always returns
                false, making all collisions invalid */
            virtual bool isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& contact) const;

            /** \brief Get the maximum number of contacts to set up
                between two colliding geoms. By default, this just
                returns the member variable maxContacts */
            virtual unsigned int getMaxContacts(dGeomID geom1, dGeomID geom2) const;
            
            /** \brief Parameters to set when contacts are created between \e geom1 and \e geom2. */
            virtual void setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const;
        };
    }
}

#endif
