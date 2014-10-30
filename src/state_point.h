/*
 * state_point.h
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#ifndef STATE_POINT_H_
#define STATE_POINT_H_

// wolf
#include <iostream>
#include "state_base.h"
#include "state_orientation.h"
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

		/**
		 * Rotation of the point
		 */
        template <orientationParametrization O_PARAM>
		void operator*=(const StateOrientation<O_PARAM>& _o);

        /**
		 * Print the state point
		 */
		virtual void print() const;

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////

template <>
inline void StatePoint::operator*=(const StateOrientation<THETA>& _o)
{
	MatrixXs R(2,2);
	R << _o.stateEstimatedMap()(0), -_o.stateEstimatedMap()(1), _o.stateEstimatedMap()(1), _o.stateEstimatedMap()(0);
	this->state_estimated_map_ = R * state_estimated_map_;
}

template <>
inline void StatePoint::operator*=(const StateOrientation<EULER>& _o)
{
	//TODO
}

template <>
inline void StatePoint::operator*=(const StateOrientation<QUATERNION>& _o)
{
	this->state_estimated_map_ = _o.q() * this->state_estimated_map_;
}

//template <orientationParametrization O_PARAM>
//void StatePoint::operator*=(const StateOrientation<O_PARAM>& _o)
//{
//	switch(O_PARAM)
//	{
//		case THETA :
//			MatrixXs R(2,2);
//			R << cos(_o.state_estimated_map), -sin(_o.state_estimated_map), sin(_o.state_estimated_map), cos(_o.state_estimated_map);
//			this->state_estimated_map_ *= R;
//			break;
//		case EULER:
//			//TODO
//			break;
//		case QUATERNION :
//			this->state_estimated_map_ = _o.q_ * this->state_estimated_map_;
//			break;
//	}
//}

template <orientationParametrization O_PARAM>
inline StatePoint operator*(const StateOrientation<O_PARAM>& _o, const StatePoint& _p)
{
	StatePoint res(_p);
	res *= _o;
	return res;
}

template <>
inline StatePoint operator*(const StateOrientation<THETA>& _o, const StatePoint& _p)
{
	MatrixXs R(2,2);
	R << _o.stateEstimatedMap()(0), -_o.stateEstimatedMap()(1), _o.stateEstimatedMap()(1), _o.stateEstimatedMap()(0);
	StatePoint res(R * _p.stateEstimatedMap());
	return res;
}

template <>
inline StatePoint operator*(const StateOrientation<EULER>& _o, const StatePoint& _p)
{
	//TODO
	Vector3s angles = _o.get_angles();
	Matrix3s R;
	R = AngleAxiss(angles(0), Vector3s::UnitZ())
	  * AngleAxiss(angles(1), Vector3s::UnitY())
	  * AngleAxiss(angles(2), Vector3s::UnitX());
	StatePoint res(R * _p.stateEstimatedMap());
	return res;
}

template <>
inline StatePoint operator*(const StateOrientation<QUATERNION>& _o, const StatePoint& _p)
{
	StatePoint res(_o.q() * _p.stateEstimatedMap());
	return res;
}



#endif /* STATE_POINT_H_ */
