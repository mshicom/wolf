/*
 * state_po.h
 *
 *  Created on: Oct 27, 2014
 *      \author: jvallve
 */

#ifndef STATE_PO_H_
#define STATE_PO_H_

// wolf
#include "state_p.h"
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
 * It inherits StateP, so it can be constructed as local or remote.
 * 
 */

template <unsigned int DIM, orientationParametrization O_PARAM>
class StatePO : protected StateP<DIM>
{
	protected:

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
		 * Local constructor from StatePoint and StateOrientation. Map member will map local vector.
		 * \param _x the state vector
		 */
		StatePO(const StatePoint& _p, const StateOrientation<O_PARAM>& _o);

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
	StateP<DIM>(DIM+O_PARAM), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const unsigned int _size) :
	StateP<DIM>(_size), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const VectorXs& _x) :
	StateP<DIM>(_x), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const StatePoint& _p, const StateOrientation<O_PARAM>& _o) :
	StateP<DIM>(_p.x(), DIM+O_PARAM),
	o_(StateP<DIM>::state_estimated_local_,DIM,_o.x())
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const StatePO& _state_p) :
	StateP<DIM>(_state_p.x()),
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
	StateP<DIM>(_st_remote,_idx, _size),
	o_(_st_remote, DIM)
{
	assert(_size >= DIM + O_PARAM);
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
	StateP<DIM>(_st_remote,_idx, _x),
	o_(_st_remote, DIM)
{
	assert(_x.size() >= DIM + O_PARAM);
}

template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::~StatePO()
{
}

template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateP<DIM>::remap(_st_remote, _idx);
	o_.remap(_st_remote, _idx + DIM);
}

template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::print() const
{
	StateP<DIM>::print();
	o_.print();
}

#endif /* STATE_PO_H_ */
