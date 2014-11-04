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
 * THETA angle is represented using a complex number. It can be initiallized giving 1 value (angle in rads) or 2 values (real and imaginary parts).
 * Both THETA complex representation and QUATERNION must be normalized in initialization.
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
        Eigen::VectorXs getAngles() const;

        /**
		 * Get the minimal representation
         */
        Eigen::MatrixXs getRotationMatrix() const;

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
		 * Invert the rotation
		 */
		void makeInverse();

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

//#################################################################
// Local empty constructor
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation() :
        StateBase(O_PARAM),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() : NULL)
{
}

//#################################################################
// Local constructor from vector
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(const VectorXs& _x) :
		StateBase(_x),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() : NULL)
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

//#################################################################
// Local copy constructor
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(const StateOrientation<O_PARAM>& _state_o) :
		StateBase(_state_o.x()),
		q_(O_PARAM == QUATERNION ? state_estimated_local_.data() : NULL)
{
}

//#################################################################
// Remote constructor from idx
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx) :
        StateBase(_st_remote,_idx, O_PARAM),
		q_(O_PARAM == QUATERNION ? _st_remote.data() + _idx : NULL)
{
	assert(_st_remote.size() >= _idx + O_PARAM);
}

//#################################################################
// Remote constructor from vector and idx
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x),
		q_(O_PARAM == QUATERNION ? _st_remote.data()  + _idx: NULL)
{
	assert(O_PARAM == _x.size());
}

template <>
StateOrientation<THETA>::StateOrientation(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x.size() == 2 ? _x : VectorXs(Vector2s(cos(_x(0)), sin(_x(0))))),
		q_(NULL)
{
}

//#################################################################
// Destructor
template <orientationParametrization O_PARAM>
StateOrientation<O_PARAM>::~StateOrientation()
{
}

//#################################################################
// q()
template <orientationParametrization O_PARAM>
inline const Eigen::Map<Eigen::Quaternions>& StateOrientation<O_PARAM>::q() const
{
	return q_;
}

//#################################################################
// getAngles
template <>
inline Eigen::VectorXs StateOrientation<THETA>::getAngles() const
{
	Eigen::VectorXs res(1);
	res(0) = atan2(this->state_estimated_map_(1), this->state_estimated_map_(0));
	return res;
}

template <>
inline Eigen::VectorXs StateOrientation<EULER>::getAngles() const
{
	return this->StateBase::stateEstimatedMap();
}

template <>
inline Eigen::VectorXs StateOrientation<QUATERNION>::getAngles() const
{
	return q_.matrix().eulerAngles(2,1,0);
}

//#################################################################
// getRotationMatrix
template <>
inline Eigen::MatrixXs StateOrientation<THETA>::getRotationMatrix() const
{
	MatrixXs R(2,2);
	R << state_estimated_map_(0), -state_estimated_map_(1), state_estimated_map_(1), state_estimated_map_(0);
	return R;
}

template <>
inline Eigen::MatrixXs StateOrientation<EULER>::getRotationMatrix() const
{
	Matrix3s R(3,3);
	R = AngleAxiss(this->state_estimated_map_(0), Vector3s::UnitZ())
	  * AngleAxiss(this->state_estimated_map_(1), Vector3s::UnitY())
	  * AngleAxiss(this->state_estimated_map_(2), Vector3s::UnitX());
	return R;
}

template <>
inline Eigen::MatrixXs StateOrientation<QUATERNION>::getRotationMatrix() const
{
	return q_.matrix();
}

//#################################################################
// remap
template <orientationParametrization O_PARAM>
inline void StateOrientation<O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
}

//#################################################################
// print
template <orientationParametrization O_PARAM>
void StateOrientation<O_PARAM>::print() const
{
	StateBase::print();
	switch(O_PARAM)
	{
		case THETA :
			std::cout << "i (complex)" << std::endl;
			std::cout << "    " << getAngles() << " (yaw)" << std::endl;
			break;
		case EULER:
			std::cout << " (euler angles)" << std::endl;
			break;
		case QUATERNION :
			std::cout << " (quaternion)" << std::endl;
			break;
	}
}

//#################################################################
// operator*
template<>
inline StateOrientation<THETA> StateOrientation<THETA>::operator*(const StateOrientation<THETA>& _o) const
{
	return StateOrientation<THETA>(Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
										  - this->state_estimated_map_(1) * _o.state_estimated_map_(1),
											this->state_estimated_map_(0) * _o.state_estimated_map_(1)
										  + this->state_estimated_map_(1) * _o.state_estimated_map_(0)));
}

template<>
inline StateOrientation<EULER> StateOrientation<EULER>::operator*(const StateOrientation<EULER>& _o) const
{
	return StateOrientation<EULER>(Matrix3s(this->getRotationMatrix() * _o.getRotationMatrix()).eulerAngles(2,1,0));
}

template<>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator*(const StateOrientation<QUATERNION>& _o) const
{
	return StateOrientation<QUATERNION>((this->q_*_o.q_).coeffs());
}

//#################################################################
// operator *=
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
	this->state_estimated_map_ = (Matrix3s(this->getRotationMatrix() * _o.getRotationMatrix()).eulerAngles(2,1,0));
}

template <>
inline void StateOrientation<QUATERNION>::operator*=(const StateOrientation<QUATERNION>& _o)
{
	this->q_*= _o.q_;
}

//#################################################################
// inverse
template <>
inline StateOrientation<THETA> StateOrientation<THETA>::inverse() const
{
	return StateOrientation<THETA>(Vector2s(this->state_estimated_map_(0), -this->state_estimated_map_(1)));
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::inverse() const
{
	//TODO
	return StateOrientation<EULER>(Matrix3s(this->getRotationMatrix().transpose()).eulerAngles(2,1,0));
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::inverse() const
{
	return StateOrientation<QUATERNION>(this->q_.conjugate().coeffs());
}

//#################################################################
// makeInverse
template <>
inline void StateOrientation<THETA>::makeInverse()
{
	this->state_estimated_map_(1) = -this->state_estimated_map_(1);
}

template <>
inline void StateOrientation<EULER>::makeInverse()
{
	this->state_estimated_map_ = Matrix3s(this->getRotationMatrix().transpose()).eulerAngles(2,1,0);
}

template <>
inline void StateOrientation<QUATERNION>::makeInverse()
{
	this->q_ = this->q_.conjugate();
}

//#################################################################
// operator/
template <>
inline StateOrientation<THETA> StateOrientation<THETA>::operator/(const StateOrientation<THETA>& _o) const
{
	return StateOrientation<THETA>(Vector2s(this->state_estimated_map_(0) * _o.state_estimated_map_(0)
								  	  	  + this->state_estimated_map_(1) * _o.state_estimated_map_(1),
										  - this->state_estimated_map_(0) * _o.state_estimated_map_(1)
										  + this->state_estimated_map_(1) * _o.state_estimated_map_(0)));
}

template <>
inline StateOrientation<EULER> StateOrientation<EULER>::operator/(const StateOrientation<EULER>& _o) const
{
	return StateOrientation<EULER>(Matrix3s(this->getRotationMatrix() * _o.getRotationMatrix().transpose()).eulerAngles(2,1,0));
}

template <>
inline StateOrientation<QUATERNION> StateOrientation<QUATERNION>::operator/(const StateOrientation<QUATERNION>& _o) const
{
	return StateOrientation<QUATERNION>((this->q_*_o.q_.conjugate()).coeffs());;
}

//#################################################################
// operator/=
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
	this->state_estimated_map_ = Matrix3s(this->getRotationMatrix() * _o.getRotationMatrix().transpose()).eulerAngles(2,1,0);
}

template <>
inline void StateOrientation<QUATERNION>::operator/=(const StateOrientation<QUATERNION>& _o)
{
	this->q_= this->q_ * _o.q_.conjugate();
}

//#################################################################
// normalize
template <>
inline void StateOrientation<THETA>::normalize()
{
	this->state_estimated_map_ /= sqrt(this->state_estimated_map_.transpose() *  this->state_estimated_map_);
}

template <>
inline void StateOrientation<EULER>::normalize()
{
	this->state_estimated_map_ = Matrix3s(this->getRotationMatrix()).eulerAngles(2,1,0);
}

template <>
inline void StateOrientation<QUATERNION>::normalize()
{
	this->q_.normalize();
}

#endif /* STATE_ORIENTATION_H_ */
