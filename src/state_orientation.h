/*
 * state_p.h
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#ifndef STATE_ORIENTATION_H_
#define STATE_ORIENTATION_H_

// wolf
#include "state_base.h"
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
enum orientationParameter {THETA, EULER, QUATERION};

template <orientationParameter OPARAM>
class StateOrientation : public StateBase
{
    public:

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StateOrientation();

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StateOrientation(const unsigned int _size);

		/**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StateOrientation(const Eigen::VectorXs& _x);

        /**
		 * Local copy constructor. Map member will map local vector.
		 * \param _state_p the state
		 */
		StateOrientation(const StateOrientation& _state_p);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateOrientation(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

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

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation() :
        StateBase(DIM), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation(const unsigned int _size) :
        StateBase(_size), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation(const VectorXs& _x) :
		StateBase(_x), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation(const StateOrientation& _state_p) :
		StateBase(_state_p.x()), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        StateBase(_st_remote,_idx, _size), //
		p_(_st_remote, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x), //
		p_(_st_remote, 0, DIM)
{
}

template<unsigned int DIM>
StateOrientation<DIM>::~StateOrientation()
{
}

template<unsigned int DIM>
inline void StateOrientation<DIM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
	p_.remap(_st_remote, _idx);
}

template<unsigned int DIM>
inline void StateOrientation<DIM>::print() const
{
	std::cout << "p: ";
	p_.print();
}

#endif /* STATE_P_H_ */
