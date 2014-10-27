/*
 * state_orientation.h
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#ifndef STATE_ORIENTATION_H_
#define STATE_ORIENTATION_H_

// wolf
#include <iostream>
#include "state_base.h"
#include "wolf.h"

/** \brief Class for state containing an orientation
 *
 * An orientation state is a non-type template state containing an orientation using the orientation parametrization fixed by O_PARAM
 * which can be THETA (2D yaw), EULER angles or QUATERNION.
 *
 * It inherits StateBase, so it can be constructed as local or remote.
 * 
 */

template <orientationParametrization O_PARAM>
class StateOrientation : public StateBase
{
	protected:
		Eigen::Map<Eigen::Quaternions> q_; ///< mapped quaternion

	public:

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StateOrientation();

		/**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StateOrientation(const Eigen::VectorXs& _x);

        /**
		 * Local copy constructor. Map member will map local vector.
		 * \param _state_p the state
		 */
		StateOrientation(const StateOrientation<O_PARAM>& _state_p);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateOrientation(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateOrientation(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StateOrientation();

        /**
		 * Change the mapped positions in the remote vector of the stateEstimatedMap
		 */
		virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /**
		 * Print the state point
		 */
		virtual void print() const;

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////

using namespace Eigen;

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation() :
        StateBase(O_PARAM),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() + 3 : NULL)
{
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(const VectorXs& _x) :
		StateBase(_x),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() + 3 : NULL)
{
	assert(O_PARAM == _x.size());
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(const StateOrientation<O_PARAM>& _state_p) :
		StateBase(_state_p.x()),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() + 3 : NULL)
{
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx) :
        StateBase(_st_remote,_idx, O_PARAM),
		q_(O_PARAM == QUATERNION ? _st_remote.data() + 3 : NULL)
{
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x),
		q_(O_PARAM == QUATERNION ? _st_remote.data() + 3 : NULL)
{
	assert(O_PARAM == _x.size());
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::~StateOrientation()
{
}

template <orientationParametrization O_PARAM>
inline void StateOrientation<O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
}

template <orientationParametrization O_PARAM>
void StateOrientation<O_PARAM>::print() const
{
	std::cout << "orientation ";
	switch(O_PARAM)
	{
		case THETA :
			std::cout << "(yaw): ";
			break;
		case EULER:
			std::cout << "(euler angles): ";
			break;
		case QUATERNION :
			std::cout << "(quaternion): ";
	}
	StateBase::print();
}

#endif /* STATE_ORIENTATION_H_ */
