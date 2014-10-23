/*
 * state_p.h
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#ifndef STATE_P_H_
#define STATE_P_H_

// wolf
#include "state_base.h"
#include "state_point.h"
#include "wolf.h"

/** \brief Class for states which have at least a position
 *
 * A position state is a state containing at least a state_point of the dimension fixed by the DIM non-type template: 2D or 3D.
 *
 * Derive from this class to include additional substates such as orientation, velocities or other possible
 * things related to motion.
 *
 * It inherits StateBase, so it can be constructed as local or remote.
 * 
 */
template <unsigned int DIM>
class StateP<DIM> : public StateBase
{
	protected:

		StatePoint p_;

    public:

        // Local Constructors
        /**
         * Local constructor from size. Map member will map local vector.
         * \param _size size of the state vector
         */
        StateP(const unsigned int _size);

        /**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StateP(const Eigen::VectorXs& _x);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateP(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateP(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StateP();

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////

using namespace Eigen;

StateP::StateP(const unsigned int _dim) :
        StateBase(_dim) //
{
}

StateP::StateP(const VectorXs& _p) :
		StateBase(_p) //
{
}

StateP::StateP(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _dim) :
        StateBase(_st_remote,_idx, _dim) //
{
}

StateP::StateP(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _p) :
		StateBase(_st_remote,_idx, _p) //
{
}

StateP::~StateP()
{
}


#endif /* STATE_P_H_ */
