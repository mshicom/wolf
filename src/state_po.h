/*
 * state_po.h
 *
 *  Created on: Oct 27, 2014
 *      \author: jvallve
 */

#ifndef STATE_PO_H_
#define STATE_PO_H_

// wolf
#include "state_base.h"
#include "state_point.h"
#include "state_orientation.h"
#include "wolf.h"

/** \brief Class for states which have at least a position and an orientation
 *
 * A PO state is a non-type template state containing at least a state_point and a state_orientation of the dimension
 * fixed by DIM (2D or 3D) and the orientation parametrization fixed by O_PARAM which can be THETA (2D), EULER angles or QUATERNION.
 *
 * Derive from this class to include additional substates such as orientation, velocities or other possible
 * things related to motion.
 *
 * It inherits StateBase, so it can be constructed as local or remote.
 * 
 */

enum orientationParametrization {THETA=1, EULER=3, QUATERNION=4};

template <unsigned int DIM, orientationParametrization O_PARAM>
class StatePO : public StateBase
{
	protected:

		StatePoint p_;
		StateOrientation<O_PARAM> o_;

    public:

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StatePO();

		// Local Constructors
		/**
		 * Local constructor from size. Map member will map local vector.
		 * \param _size size of the state vector
		 */
		StatePO(const unsigned int _size);

		/**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StatePO(const Eigen::VectorXs& _x);

        /**
		 * Local copy constructor. Map member will map local vector.
		 * \param _state_p the state
		 */
		StatePO(const StatePO& _state_p);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StatePO(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StatePO(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StatePO();

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

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO() :
        StateBase(DIM+O_PARAM), //
		p_(state_estimated_local_, 0, DIM),
		o_(state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const unsigned int _size) :
        StateBase(_size), //
		p_(state_estimated_local_, 0, DIM),
		o_(state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const VectorXs& _x) :
		StateBase(_x), //
		p_(state_estimated_local_, 0, DIM),
		o_(state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const StatePO& _state_p) :
		StateBase(_state_p.x()), //
		p_(state_estimated_local_, 0, DIM),
		o_(state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        StateBase(_st_remote,_idx, _size), //
		p_(_st_remote, 0, DIM),
		o_(_st_remote, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
		StateBase(_st_remote,_idx, _x), //
		p_(_st_remote, 0, DIM),
		o_(_st_remote, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::~StatePO()
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateBase::remap(_st_remote, _idx);
	p_.remap(_st_remote, _idx);
	o_.remap(_st_remote, _idx + DIM);
}

template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::print() const
{
	p_.print();
	o_.print();
}

#endif /* STATE_PO_H_ */
