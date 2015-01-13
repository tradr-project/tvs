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

#ifndef OMPL_EXTENSION_OMPLTVS_STATE_SPACE_
#define OMPL_EXTENSION_OMPLTVS_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include "OMPLTVSEnvironment.h"

namespace ompl
{
    namespace control
    {
        /** \brief Defines how we project a state to a lower dimensional (euclidean) space */
        class OMPLTVSStateProjectionEvaluator : public base::ProjectionEvaluator
        {
        public:
            OMPLTVSStateProjectionEvaluator(const base::StateSpace *space);
            virtual unsigned int getDimension(void) const;
            virtual void defaultCellSizes(void);
            virtual void project(const base::State *state, base::EuclideanProjection &projection) const;
        };

        
        /** \brief State space representing OMPLTVS states */
        class OMPLTVSStateSpace : public base::CompoundStateSpace
        {
        public:

            enum {
                /** \brief Index of bit in StateType::collision indicating whether it is known if a state is in collision or not. Initially this is 0. The value of this bit is updated by OMPLTVSStateSpace::evaluateCollision() and OMPLTVSControlSpace::propagate(). */
                STATE_COLLISION_KNOWN_BIT = 0,
                /** \brief Index of bit in StateType::collision indicating whether a state is in collision or not. Initially the value of this field is unspecified. The value gains meaning (1 or 0) when OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT becomes 1. The value of this bit is updated by OMPLTVSStateSpace::evaluateCollision() and OMPLTVSControlSpace::propagate(). A value of 1 implies that there is no collision for which OMPLTVSEnvironment::isValidCollision() returns false. */
                STATE_COLLISION_VALUE_BIT = 1,
                /** \brief Index of bit in StateType::collision indicating whether it is known if a state is in valid or not. Initially this is 0. The value of this bit is updated by OMPLTVSStateValidityChecker::isValid(). This bit is only used if the OMPLTVSStateValidityChecker is used. */
                STATE_VALIDITY_KNOWN_BIT = 2,
                /** \brief Index of bit in StateType::collision indicating whether a state is valid or not. Initially the value of this field is unspecified. The value gains meaning (1 or 0) when OMPLTVSStateSpace::STATE_VALIDITY_KNOWN_BIT becomes 1. The value of this bit is updated by OMPLTVSEnvironment::isValid(). A value of 1 implies that a state is valid. This bit is only used if the OMPLTVSStateValidityChecker is used. */
                STATE_VALIDITY_VALUE_BIT = 3,
            };
            
            enum {
                COMPONENT_POSITION = 0,
                COMPONENT_LINEAR_VELOCITY,
                COMPONENT_ANGULAR_VELOCITY,
                COMPONENT_ROTATION,
                
                NUM_COMPONENTS
            };

            /** \brief OMPLTVS State. This is a compound state that allows accessing the properties of the bodies the state space is constructed for. */
            class StateType : public base::CompoundStateSpace::StateType
            {
            public:
                StateType() : base::CompoundStateSpace::StateType(), collision(0)
                {
                }

                /** \brief Get the position (x, y, z) of the body at index \e body */
                const double* getBodyPosition(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_POSITION)->values;
                }

                /** \brief Get the position (x, y, z) of the body at index \e body */
                double* getBodyPosition(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_POSITION)->values;
                }

                /** \brief Get the quaternion of the body at index \e body */
                const base::SO3StateSpace::StateType& getBodyRotation(unsigned int body) const
                {
                    return *as<base::SO3StateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_ROTATION);
                }

                /** \brief Get the quaternion of the body at index \e body */
                base::SO3StateSpace::StateType& getBodyRotation(unsigned int body)
                {
                    return *as<base::SO3StateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_ROTATION);
                }

                /** \brief Get the linear velocity (x, y, z) of the body at index \e body */
                const double* getBodyLinearVelocity(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_LINEAR_VELOCITY)->values;
                }

                /** \brief Get the linear velocity (x, y, z) of the body at index \e body */
                double* getBodyLinearVelocity(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_LINEAR_VELOCITY)->values;
                }

                /** \brief Get the angular velocity (x, y, z) of the body at index \e body */
                const double* getBodyAngularVelocity(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_ANGULAR_VELOCITY)->values;
                }

                /** \brief Get the angular velocity (x, y, z) of the body at index \e body */
                double* getBodyAngularVelocity(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * NUM_COMPONENTS + COMPONENT_ANGULAR_VELOCITY)->values;
                }

                /** \brief Flag containing information about state validity.

                    - BIT 0: (OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT)
                    - BIT 1: (OMPLTVSStateSpace::STATE_COLLISION_VALUE_BIT)
                    - BIT 2: (OMPLTVSStateSpace::STATE_VALIDITY_KNOWN_BIT)
                    - BIT 3: (OMPLTVSStateSpace::STATE_VALIDITY_VALUE_BIT) */
                mutable int collision;

            };

            /** \brief Construct a state space representing OMPLTVS states.

                This will be a compound state space with 4 components for
                each body in \e env.stateBodies_. The 4 subspaces
                constructed for each body are: position
                (R<sup>3</sup>), linear velocity (R<sup>3</sup>),
                angular velocity (R<sup>3</sup>) and orientation
                (SO(3)). Default bounds are set by calling setDefaultBounds().

                \param env the environment to construct the state space for
                \param positionWeight the weight to pass to CompoundStateSpace::addSubspace() for position subspaces
                \param linVelWeight the weight to pass to CompoundStateSpace::addSubspace() for linear velocity subspaces
                \param angVelWeight the weight to pass to CompoundStateSpace::addSubspace() for angular velocity subspaces
                \param orientationWeight the weight to pass to CompoundStateSpace::addSubspace() for orientation subspaces
            */
            OMPLTVSStateSpace(const OMPLTVSEnvironmentPtr &env,
                             double positionWeight = 1.0, double linVelWeight = 0.5,
                             double angVelWeight = 0.5, double orientationWeight = 1.0);

            virtual ~OMPLTVSStateSpace()
            {
            }

            /** \brief Get the OMPLTVS environment this state space corresponds to */
            const OMPLTVSEnvironmentPtr& getEnvironment() const
            {
                return env_;
            }

            /** \brief By default, the volume bounds enclosing the
              geometry of the environment are computed to include all
              objects in the spaces collision checking is performed
              (env.collisionSpaces_). The linear and angular velocity
              bounds are set as -1 to 1 for each dimension. */
            void setDefaultBounds();

            /** \brief Read the parameters of the OMPLTVS bodies and store
                them in \e state. */
            virtual void readState(base::State *state) const;

            /** \brief Set the parameters of the OMPLTVS bodies to be the
                ones read from \e state.  The code will technically work if
                this function is called from multiple threads
                simultaneously, but the results are unpredictable. */
            virtual void writeState(const base::State *state) const;

            /** \brief This is a convenience function provided for
                optimization purposes. It checks whether a state
                satisfies its bounds. Typically, in the process of
                simulation the rotations remain valid (very slightly
                out of bounds), so there is no point in updating or
                checking them. This function checks all other bounds
                (position, linear and agular velocities) */
            bool satisfiesBoundsExceptRotation(const StateType *state) const;

            void setVolumeBounds(const base::RealVectorBounds &bounds);
            void setLinearVelocityBounds(const base::RealVectorBounds &bounds);
            void setAngularVelocityBounds(const base::RealVectorBounds &bounds);

            virtual base::State* allocState() const;
            virtual void freeState(base::State *state) const;
            virtual void copyState(base::State *destination, const base::State *source) const;
            virtual void interpolate(const base::State *from, const base::State *to, const double t, base::State *state) const;

            virtual base::StateSamplerPtr allocDefaultStateSampler() const;
            virtual base::StateSamplerPtr allocStateSampler() const;

            /** \brief Fill the OMPLTVSStateSpace::STATE_COLLISION_VALUE_BIT of StateType::collision member of a state, if unspecified.
                Return the value value of that bit. */
            virtual bool evaluateCollision(const base::State *source) const;
            
            virtual double distance(const base::State *s1, const base::State *s2) const;
            virtual void registerProjections(void);

            virtual double getMeasure() const;
            
        protected:

            /** \brief Representation of the OMPLTVS parameters OMPL needs to plan */
            OMPLTVSEnvironmentPtr env_;
        };
    }
}


#endif
