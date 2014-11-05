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
		 * Local constructor from StatePoint. Map member will map local vector.
		 * \param _p the StatePoint
		 */
		StateP(const StatePoint& _p);

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

        /**
		 * Concatenate a frame (in local reference) to the current frame.
         * \param _s_local the frame in local reference
         * \return the given frame in the global reference (reference of *this)
		 */
		virtual StateP concatenate(const StateP& _s_local) const;

        /**
		 * Concatenate a frame (in local reference) to the current frame.
         * \param _s_local the frame in local reference
         * \param _s_out output parameter, the given frame in the reference of *this
		 */
		virtual void concatenate(const StateP& _s_local, StateP& _s_out) const;

        /**
		 * Concatenate a frame (in local reference) to the current frame and store it in *this
         * \param _s_local the frame in local reference
		 */
		virtual void concatenateInPlace(const StateP& _s_local);

        /**
		 * Inverse the current frame
         * \return the inverse frame
		 */
		virtual StateP inverse() const;

        /**
		 * Inverse the current frame
         * \param _s_out output parameter, the inverse frame
		 */
		virtual void inverse(StateP& _s_out) const;

        /**
		 * Inverse the current frame and store it in *this
		 */
		virtual void makeInverse();

        /**
		 * The given frame expressed in the current frame local reference.
         * \param _s_reference the frame in global reference (same reference of *this)
         * \return the given frame in local reference
		 */
		virtual StateP relativeTo(const StateP& _s_reference) const;

        /**
		 * The given frame expressed in the current frame local reference.
         * \param _s_reference the frame in global reference (same reference of *this)
         * \param _s_out output parameter, the given frame in local reference
		 */
		virtual void relativeTo(const StateP& _s_reference, StateP& _s_out) const;

        /**
		 * The given frame expressed in the current frame local reference and store it in *this
         * \param _s_reference the frame in global reference (same reference of *this)
		 */
		virtual void makeRelativeTo(const StateP& _s_reference);

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////


using namespace Eigen;

//#################################################################
// Local empty constructor
template<unsigned int DIM>
StateP<DIM>::StateP() :
        StateBase(DIM), //
		p_(state_estimated_local_, 0, DIM)
{
}

//#################################################################
// Local constructor from size
template<unsigned int DIM>
StateP<DIM>::StateP(const unsigned int _size) :
        StateBase(_size), //
		p_(state_estimated_local_, 0, DIM)
{
}

//#################################################################
// Local constructor from vector
template<unsigned int DIM>
StateP<DIM>::StateP(const VectorXs& _x) :
		StateBase(_x), //
		p_(state_estimated_local_, 0, DIM)
{
}

//#################################################################
// Local constructor from vector and size
//TODO: Check if it has sense
template<unsigned int DIM>
StateP<DIM>::StateP(const VectorXs& _x, const unsigned int _size) :
		StateBase(_size), //
		p_(state_estimated_local_, 0, _x)
{
	assert(_x.size() == DIM);
}

//#################################################################
// Local constructor from StatePoint
template<unsigned int DIM>
StateP<DIM>::StateP(const StatePoint& _p) :
		StateBase(_p.stateEstimatedMap()), //
		p_(state_estimated_local_, 0, DIM)
{
	assert(_p.size() == DIM);
}

//#################################################################
// Local copy constructor
template<unsigned int DIM>
StateP<DIM>::StateP(const StateP& _state_p) :
		StateBase(_state_p.stateEstimatedMap()), //
		p_(state_estimated_local_, 0, DIM)
{
}

//#################################################################
// Remote constructor from index and size
template<unsigned int DIM>
StateP<DIM>::StateP(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        StateBase(_st_remote,_idx, _size), //
		p_(_st_remote, 0, DIM)
{
}

//#################################################################
// Remote constructor from index and vector
template<unsigned int DIM>
StateP<DIM>::StateP(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x), //
		p_(_st_remote, 0, DIM)
{
}

//#################################################################
// Destructor
template<unsigned int DIM>
StateP<DIM>::~StateP()
{
}

//#################################################################
// remap
template<unsigned int DIM>
inline void StateP<DIM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
	p_.remap(_st_remote, _idx);
}

//#################################################################
// print
template<unsigned int DIM>
inline void StateP<DIM>::print() const
{
	p_.print();
}

//#################################################################
// concatenate
template<unsigned int DIM>
StateP<DIM> StateP<DIM>::concatenate(const StateP<DIM>& _s_local) const
{
	return StateP<DIM>(this->p_+_s_local.p_);
}

template<unsigned int DIM>
void StateP<DIM>::concatenate(const StateP<DIM>& _s_local, StateP<DIM>& _s_out) const
{
	_s_out.p_ = this->p_+_s_local.p_;
}

template<unsigned int DIM>
void StateP<DIM>::concatenateInPlace(const StateP<DIM>& _s_local)
{
	this->p_ = this->p_+_s_local.p_;
}

//#################################################################
// inverse
template<unsigned int DIM>
StateP<DIM> StateP<DIM>::inverse() const
{
	return StateP<DIM>(-this->p_);
}

template<unsigned int DIM>
void StateP<DIM>::inverse(StateP<DIM>& _s_out) const
{
	_s_out.p_ = -this->p_;
}

template<unsigned int DIM>
void StateP<DIM>::makeInverse()
{
	this->p_.makeOpposite();
}

//#################################################################
// relative to
template<unsigned int DIM>
StateP<DIM> StateP<DIM>::relativeTo(const StateP<DIM>& _s_reference) const
{
	return StateP<DIM>(this->p_-_s_reference.p_);
}

template<unsigned int DIM>
void StateP<DIM>::relativeTo(const StateP<DIM>& _s_reference, StateP<DIM>& _s_out) const
{
	_s_out.p_ = this->p_-_s_reference.p_;
}

template<unsigned int DIM>
void StateP<DIM>::makeRelativeTo(const StateP<DIM>& _s_reference)
{
	this->p_-=_s_reference.p_;
}

#endif /* STATE_P_H_ */
