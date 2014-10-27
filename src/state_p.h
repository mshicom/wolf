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
class StateP : public StateBase
{
	protected:

		StatePoint p_;

    public:

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StateP();

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

        /**
		 * Local constructor from vector and size. Map member will map local vector.
		 * \param _x the state vector
		 * \param _size size of the state vector
		 */
		StateP(const Eigen::VectorXs& _x, const unsigned int _size);

		/**
		 * Local copy constructor. Map member will map local vector.
		 * \param _state_p the state
		 */
		StateP(const StateP& _state_p);

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
StateP<DIM>::StateP() :
        StateBase(DIM), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::StateP(const unsigned int _size) :
        StateBase(_size), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::StateP(const VectorXs& _x) :
		StateBase(_x), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::StateP(const Eigen::VectorXs& _x, const unsigned int _size) :
		StateBase(_size), //
		p_(state_estimated_local_, 0, _x)
{
	assert(_x.size() == DIM);
}

template<unsigned int DIM>
StateP<DIM>::StateP(const StateP& _state_p) :
		StateBase(_state_p.x()), //
		p_(state_estimated_local_, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::StateP(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        StateBase(_st_remote,_idx, _size), //
		p_(_st_remote, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::StateP(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x), //
		p_(_st_remote, 0, DIM)
{
}

template<unsigned int DIM>
StateP<DIM>::~StateP()
{
}

template<unsigned int DIM>
inline void StateP<DIM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
	p_.remap(_st_remote, _idx);
}

template<unsigned int DIM>
inline void StateP<DIM>::print() const
{
	p_.print();
}

#endif /* STATE_P_H_ */
