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
 * THETA angle is represented by a complex number. It can be initiallized giving 1 value (angle in rads) or 2 values (real and imaginary parts).
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
		 * Get the minimal representation
         */
        const Eigen::VectorXs get_angles() const;

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
		StateOrientation<O_PARAM> operator*(const StateOrientation<O_PARAM>& _o) const;

		/**
		 * Composition of rotations overloading the result in *this
		 */
		void operator*=(const StateOrientation<O_PARAM>& _o);

		/**
		 * Inverse rotation
		 */
		StateOrientation<O_PARAM> inverse() const;

		/**
		 * Composition with the inverse rotation of the given one
		 */
		StateOrientation<O_PARAM> operator/(const StateOrientation<O_PARAM>& _o) const;

		/**
		 * Composition with the inverse rotation of the given one overloading the result in *this
		 */
		void operator/=(const StateOrientation<O_PARAM>& _o);

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

template <>
StateOrientation<THETA>::StateOrientation(const VectorXs& _x) :
		StateBase(_x.size() == 2 ? _x : VectorXs(Vector2s(cos(_x(0)), sin(_x(0))))),
		q_(NULL)
{
	assert(StateBase::size() == 2);
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(const StateOrientation<O_PARAM>& _state_o) :
		StateBase(_state_o.x()),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() + 3 : NULL)
{
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx) :
        StateBase(_st_remote,_idx, O_PARAM),
		q_(O_PARAM == QUATERNION ? _st_remote.data() + 3 : NULL)
{
	assert(_st_remote.size() >= _idx + O_PARAM);
}

template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x),
		q_(O_PARAM == QUATERNION ? _st_remote.data() + 3 : NULL)
{
	assert(O_PARAM == _x.size());
}

template <>
StateOrientation<THETA>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x.size() == 2 ? _x : VectorXs(Vector2s(cos(_x(0)), sin(_x(0))))),
		q_(NULL)
{
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

template <>
inline const Eigen::VectorXs StateOrientation<THETA>::get_angles() const
{
	Eigen::VectorXs res(1);
	res(0) = atan2(this->state_estimated_map_(1), this->state_estimated_map_(0));
	return res;
}

template <>
inline const Eigen::VectorXs StateOrientation<EULER>::get_angles() const
{
	return this->StateBase::stateEstimatedMap();
}

template <>
inline const Eigen::VectorXs StateOrientation<QUATERNION>::get_angles() const
{
	return q_.matrix().eulerAngles(2,1,0);
}

template <orientationParametrization O_PARAM>
inline void StateOrientation<O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
}

template <orientationParametrization O_PARAM>
void StateOrientation<O_PARAM>::print() const
{
	StateBase::print();
	switch(O_PARAM)
	{
		case THETA :
			std::cout << "i (complex)" << std::endl;
			std::cout << "    " << get_angles() << "(yaw)" << std::endl;
			break;
		case EULER:
			std::cout << "(euler angles)" << std::endl;
			break;
		case QUATERNION :
			std::cout << "(quaternion)" << std::endl;
			break;
	}
}

template<>
inline StateOrientation<THETA> StateOrientation<THETA>::operator*(const StateOrientation<THETA>& _o) const
{
	StateOrientation<THETA> res(Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
									   - this->state_estimated_map_(1) * _o.state_estimated_map_(1),
									     this->state_estimated_map_(0) * _o.state_estimated_map_(1)
									   + this->state_estimated_map_(1) * _o.state_estimated_map_(0)));
	return res;
}

template<>
inline StateOrientation<EULER> StateOrientation<EULER>::operator*(const StateOrientation<EULER>& _o) const
{
	//TODO: Treure tot això a quaternion_tools o algo així
	Matrix3s R1, R2, R;
	R1 = AngleAxiss(this->state_estimated_map_(0), Vector3s::UnitZ())
	   * AngleAxiss(this->state_estimated_map_(1), Vector3s::UnitY())
	   * AngleAxiss(this->state_estimated_map_(2), Vector3s::UnitX());
	R2 = AngleAxiss(_o.state_estimated_map_(0), Vector3s::UnitZ())
	   * AngleAxiss(_o.state_estimated_map_(1), Vector3s::UnitY())
	   * AngleAxiss(_o.state_estimated_map_(2), Vector3s::UnitX());
	R = R1 * R2;

	StateOrientation<EULER> res(Vector3s(atan2(R(2,1),R(2,2)), atan2(-R(2,0), sqrt(pow(R(2,1),2)+pow(R(2,2),2))), atan2(R(1,0),R(0,0))));
	return res;
}

template<>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator*(const StateOrientation<QUATERNION>& _o) const
{
	StateOrientation<QUATERNION> res((this->q_*_o.q_).coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::operator*(const StateOrientation<O_PARAM>& _o) const
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			StateOrientation<THETA> res1(Matrix(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
//					                   - this->state_estimated_map_(1) * _o.state_estimated_map_(1),
//					                     this->state_estimated_map_(0) * _o.state_estimated_map_(1)
//				                       + this->state_estimated_map_(1) * _o.state_estimated_map_(0)));
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
inline void StateOrientation<THETA>::operator*=(const StateOrientation<THETA>& _o)
{
	this->state_estimated_map_= Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
									   - this->state_estimated_map_(1) * _o.state_estimated_map_(1),
									     this->state_estimated_map_(0) * _o.state_estimated_map_(1)
								       + this->state_estimated_map_(1) * _o.state_estimated_map_(0));
}

template <>
inline void StateOrientation<EULER>::operator*=(const StateOrientation<EULER>& _o)
{
	//TODO
	Matrix3s m1, m2, m;
	m1 = AngleAxiss(this->state_estimated_map_(0), Vector3s::UnitZ())
	   * AngleAxiss(this->state_estimated_map_(1), Vector3s::UnitY())
	   * AngleAxiss(this->state_estimated_map_(2), Vector3s::UnitX());
	m2 = AngleAxiss(_o.state_estimated_map_(0), Vector3s::UnitZ())
	   * AngleAxiss(_o.state_estimated_map_(1), Vector3s::UnitY())
	   * AngleAxiss(_o.state_estimated_map_(2), Vector3s::UnitX());
	m = m1 * m2;

	this->state_estimated_map_(0) = atan2(m(2,1),m(2,2));
	this->state_estimated_map_(1) = atan2(-m(2,0),sqrt(pow(m(2,1),2)+pow(m(2,2),2)));
	this->state_estimated_map_(2) = atan2(m(1,0),m(0,0));
	//\theta_{x} = atan2\left(r_{32}, r_{33}\right)
	//\theta_{y} = atan2\left(-r_{31}, \sqrt{r_{32}^2 + r_{33}^2}\right)
	//\theta_{z} = atan2\left(r_{21}, r_{11}\right)
}

template <>
inline void StateOrientation<QUATERNION>::operator*=(const StateOrientation<QUATERNION>& _o)
{
	this->q_*=_o.q_;
}

//template <orientationParametrization O_PARAM>
//void StateOrientation<O_PARAM>::operator*=(const StateOrientation<O_PARAM>& _o)
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			this->state_estimated_map_= Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
//											   - this->state_estimated_map_(1) * _o.state_estimated_map_(1),
//												 this->state_estimated_map_(0) * _o.state_estimated_map_(1)
//											   + this->state_estimated_map_(1) * _o.state_estimated_map_(0));
//			break;
//		case EULER:
//			//TODO
//			this->state_estimated_map_ += _o.state_estimated_map_;
//			break;
//		case QUATERNION :
//			this->q_*=_o.q_;
//			break;
//	}
//}

template <>
inline StateOrientation<THETA> StateOrientation<THETA>::inverse() const
{
	StateOrientation<THETA> res(Vector2s(this->state_estimated_map_(0), -this->state_estimated_map_(1)));
	return res;
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::inverse() const
{
	//TODO
	StateOrientation<EULER> res(-this->state_estimated_map_);
	return res;
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::inverse() const
{
	StateOrientation<QUATERNION> res(this->q_.conjugate().coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::inverse() const
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
inline StateOrientation<THETA> StateOrientation<THETA>::operator/(const StateOrientation<THETA>& _o) const
{

	StateOrientation<THETA> res(Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
							  	  	   + this->state_estimated_map_(1) * _o.state_estimated_map_(1),
									   - this->state_estimated_map_(0) * _o.state_estimated_map_(1)
									   + this->state_estimated_map_(1) * _o.state_estimated_map_(0)));
	return res;
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::operator/(const StateOrientation<EULER>& _o) const
{
	//TODO
	StateOrientation<EULER> res(this->state_estimated_map_-_o.state_estimated_map_);
	return res;
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator/(const StateOrientation<QUATERNION>& _o) const
{
	StateOrientation<QUATERNION> res((this->q_*_o.q_.conjugate()).coeffs());
	return res;
}

//template <orientationParametrization O_PARAM>
//StateOrientation<O_PARAM> StateOrientation<O_PARAM>::operator/(const StateOrientation<O_PARAM>& _o) const
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
inline void StateOrientation<THETA>::operator/=(const StateOrientation<THETA>& _o)
{
	this->state_estimated_map_ = Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
									    + this->state_estimated_map_(1) * _o.state_estimated_map_(1),
									    - this->state_estimated_map_(0) * _o.state_estimated_map_(1)
									    + this->state_estimated_map_(1) * _o.state_estimated_map_(0));
}

template <>
inline void StateOrientation<EULER>::operator/=(const StateOrientation<EULER>& _o)
{
	//TODO
	this->state_estimated_map_-=_o.state_estimated_map_;
}

template <>
inline void StateOrientation<QUATERNION>::operator/=(const StateOrientation<QUATERNION>& _o)
{
	this->q_*=_o.q_.conjugate();
}

//template <orientationParametrization O_PARAM>
//void StateOrientation<O_PARAM>::operator/=(const StateOrientation<O_PARAM>& _o)
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
	this->state_estimated_map_ /= sqrt(this->state_estimated_map_.transpose() *  this->state_estimated_map_);
}

template <>
inline void StateOrientation<EULER>::normalize()
{
	//TODO
	this->state_estimated_map_(0)-= 2 * M_PI * floor( (this->state_estimated_map_(0) + M_PI) / 2 * M_PI );
	this->state_estimated_map_(1)-= 2 * M_PI * floor( (this->state_estimated_map_(1) + M_PI) / 2 * M_PI );
	this->state_estimated_map_(2)-= 2 * M_PI * floor( (this->state_estimated_map_(2) + M_PI) / 2 * M_PI );
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
