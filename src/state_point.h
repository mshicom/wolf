/*
 * state_point.h
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#ifndef STATE_POINT_H_
#define STATE_POINT_H_

// wolf
#include "state_base.h"
#include "wolf.h"

/** \brief Class for position states.
 *
 * A position state is a state containing a 2D or 3D position.
 *
 * Do not derive from this class to include additional substates such as velocities or other possible
 * things related to motion. See state_p for this functionallity.
 *
 * It inherits StateBase, so it can be constructed as local or remote.
 * P is mapped over the StateBase map.
 * 
 */
class StatePoint : public StateBase
{
    public:

        // Local Constructors
        /**
         * Local constructor from size. Map member will map local vector.
         * \param _dim simension of the state point (2D or 3D)
         */
        StatePoint(const unsigned int _dim);

        /**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state point
         */
        StatePoint(const Eigen::VectorXs& _p);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _dim dimension of the state point (2D or 3D)
         */
        StatePoint(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _dim);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _p the state point
         */
        StatePoint(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _p);

        /**
         * Destructor
         */
        virtual ~StatePoint();

};

#endif /* STATE_POINT_H_ */
