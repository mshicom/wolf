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
#include "math.h"
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
		StateOrientation(const StateOrientation<O_PARAM>& _state_o);

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
		 * Get reference to quaternion
         */
        const Eigen::Map<Eigen::Quaternions>& q() const;

        /**
		 * Change the mapped positions in the remote vector of the stateEstimatedMap
		 */
		virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /**
		 * Print the state point
		 */
		virtual void print() const;

		/**
		 * Composition of rotations
		 */
		StateOrientation<O_PARAM> operator+(const StateOrientation<O_PARAM>& _o) const;

		/**
		 * Composition of rotations
		 */
		void operator+=(const StateOrientation<O_PARAM>& _o);

		/**
		 * Inverse rotation
		 */
		StateOrientation<O_PARAM> operator-() const;

		/**
		 * Substraction of rotations
		 */
		StateOrientation<O_PARAM> operator-(const StateOrientation<O_PARAM>& _o) const;

		/**
		 * Substraction of rotations
		 */
		void operator-=(const StateOrientation<O_PARAM>& _o);

		/**
		 * Normalization of rotation: impose (-pi,pi] in THETA and EULER angles and normalize in QUATERNION
		 */
		void normalize();
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
StateOrientation<O_PARAM>::StateOrientation(const StateOrientation<O_PARAM>& _state_o) :
		StateBase(state_estimated_local_),
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
inline const Eigen::Map<Eigen::Quaternions>& StateOrientation<O_PARAM>::q() const
{
	return q_;
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
			break;
	}
	StateBase::print();
}

template<>
inline StateOrientation<THETA> StateOrientation<THETA>::operator+(const StateOrientation<THETA>& _o) const
{
	StateOrientation<THETA> res(this->state_estimated_map_+_o.state_estimated_map_);
	return res;
}

template<>
inline StateOrientation<EULER> StateOrientation<EULER>::operator+(const StateOrientation<EULER>& _o) const
{
	//TODO
	StateOrientation<EULER> res(this->state_estimated_map_+_o.state_estimated_map_);
	return res;
}

template<>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator+(const StateOrientation<QUATERNION>& _o) const
{
	StateOrientation<QUATERNION> res((this->q_*_o.q_).coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::operator+(const StateOrientation<O_PARAM>& _o) const
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			StateOrientation<O_PARAM> res1(this->state_estimated_map_+_o.state_estimated_map_);
//			return res1;
//			break;
//		case EULER:
//			//TODO
//			StateOrientation<O_PARAM> res2(this->state_estimated_map_+_o.state_estimated_map_);
//			return res2;
//			break;
//		case QUATERNION :
//			StateOrientation<O_PARAM> res3(this->q_*_o.q_);
//			return res3;
//			break;
//	}
//}

template <>
inline void StateOrientation<THETA>::operator+=(const StateOrientation<THETA>& _o)
{
	this->state_estimated_map_+=_o.state_estimated_map_;
}

template <>
inline void StateOrientation<EULER>::operator+=(const StateOrientation<EULER>& _o)
{
	//TODO
}

template <>
inline void StateOrientation<QUATERNION>::operator+=(const StateOrientation<QUATERNION>& _o)
{
	this->q_*=_o.q_;
}

//template <orientationParametrization O_PARAM>
//void StateOrientation<O_PARAM>::operator+=(const StateOrientation<O_PARAM>& _o)
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			this->state_estimated_map_+=_o.state_estimated_map_;
//			break;
//		case EULER:
//			//TODO
//			break;
//		case QUATERNION :
//			this->q_*=_o.q_;
//			break;
//	}
//}

template <>
inline StateOrientation<THETA> StateOrientation<THETA>::operator-() const
{
	StateOrientation<THETA> res(-this->state_estimated_map_);
	return res;
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::operator-() const
{
	//TODO
	StateOrientation<EULER> res(*this);
	return res;
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator-() const
{
	StateOrientation<QUATERNION> res(this->q_.conjugate().coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::operator-() const
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			//res.state_estimated_map_ = -res.state_estimated_map_;
//			StateOrientation<THETA> res1(-this->state_estimated_map_);
//			return res1;
//			break;
//		case EULER:
//			//TODO
//			StateOrientation<EULER> res2(*this);
//			return res2;
//			break;
//		case QUATERNION :
//			StateOrientation<QUATERNION> res3(*this->q_.conjugate());
//			return res3;
//			break;
//	}
//}

template <>
inline StateOrientation<THETA> StateOrientation<THETA>::operator-(const StateOrientation<THETA>& _o) const
{
	StateOrientation<THETA> res(this->state_estimated_map_-_o.state_estimated_map_);
	return res;
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::operator-(const StateOrientation<EULER>& _o) const
{
	//TODO
	StateOrientation<EULER> res(this->state_estimated_map_-_o.state_estimated_map_);
	return res;
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator-(const StateOrientation<QUATERNION>& _o) const
{
	StateOrientation<QUATERNION> res((this->q_*_o.q_.conjugate()).coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::operator-(const StateOrientation<O_PARAM>& _o) const
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			StateOrientation<O_PARAM> res1(this->state_estimated_map_-_o.state_estimated_map_);
//			return res1;
//			break;
//		case EULER:
//			//TODO
//			StateOrientation<O_PARAM> res2(this->state_estimated_map_-_o.state_estimated_map_);
//			return res2;
//			break;
//		case QUATERNION :
//			StateOrientation<O_PARAM> res3(this->q_*_o.q_.conjugate());
//			return res3;
//			break;
//	}
//}

template <>
inline void StateOrientation<THETA>::operator-=(const StateOrientation<THETA>& _o)
{
	this->state_estimated_map_-=_o.state_estimated_map_;
}

template <>
inline void StateOrientation<EULER>::operator-=(const StateOrientation<EULER>& _o)
{
	//TODO
	this->state_estimated_map_-=_o.state_estimated_map_;
}

template <>
inline void StateOrientation<QUATERNION>::operator-=(const StateOrientation<QUATERNION>& _o)
{
	this->q_*=_o.q_.conjugate();
}

//template <orientationParametrization O_PARAM>
//void StateOrientation<O_PARAM>::operator-=(const StateOrientation<O_PARAM>& _o)
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			this->state_estimated_map_-=_o.state_estimated_map_;
//			break;
//		case EULER:
//			//TODO
//			break;
//		case QUATERNION :
//			this->q_*=_o.q_.conjugate();
//			break;
//	}
//}

template <>
inline void StateOrientation<THETA>::normalize()
{
	this->state_estimated_map_(0)-= 2 * M_PI * floor( (this->state_estimated_map_(0) + M_PI) / 2 * M_PI );
}

template <>
inline void StateOrientation<EULER>::normalize()
{
	//TODO
}

template <>
inline void StateOrientation<QUATERNION>::normalize()
{
	this->q_.normalize();
}

//template <orientationParametrization O_PARAM>
//void StateOrientation<O_PARAM>::normalize()
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			this->state_estimated_map_-= 2 * M_PI * floor( (this->state_estimated_map_ + M_PI) / 2 * M_PI );
//			break;
//		case EULER:
//
//			break;
//		case QUATERNION :
//			this->q_.normalize();
//			break;
//	}
//}

#endif /* STATE_ORIENTATION_H_ */
