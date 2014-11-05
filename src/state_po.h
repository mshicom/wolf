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
 * StatePO inherits StateP<DIM> so it includes a StatePoint of dimension DIM.
 * The StatePO is a non-type template state containing at least a position and a orientation. The dimension is
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

        /**
		 * Concatenate a frame (in local reference) to the current frame.
         * \param _s_local the frame in local reference
         * \return the given frame in the global reference (reference of *this)
		 */
		StatePO concatenate(const StatePO& _s_local) const;

        /**
		 * Concatenate a frame (in local reference) to the current frame.
         * \param _s_local the frame in local reference
         * \param _s_out output parameter, the given frame in the reference of *this
		 */
		void concatenate(const StatePO& _s_local, StatePO& _s_out) const;

        /**
		 * Concatenate a frame (in local reference) to the current frame and store it in *this
         * \param _s_local the frame in local reference
		 */
		void concatenateInPlace(const StatePO& _s_local);

        /**
		 * Inverse the current frame
         * \return the inverse frame
		 */
		StatePO inverse() const;

        /**
		 * Inverse the current frame
         * \param _s_out output parameter, the inverse frame
		 */
		void inverse(StatePO& _s_out) const;

        /**
		 * Inverse the current frame and store it in *this
		 */
		void makeInverse();

        /**
		 * The given frame expressed in the current frame local reference.
         * \param _s_reference the frame in global reference (same reference of *this)
         * \return the given frame in local reference
		 */
		StatePO relativeTo(const StatePO& _s_reference) const;

        /**
		 * The given frame expressed in the current frame local reference.
         * \param _s_reference the frame in global reference (same reference of *this)
         * \param _s_out output parameter, the given frame in local reference
		 */
		void relativeTo(const StatePO& _s_reference, StatePO& _s_out) const;

        /**
		 * The given frame expressed in the current frame local reference and store it in *this
         * \param _s_reference the frame in global reference (same reference of *this)
		 */
		void makeRelativeTo(const StatePO& _s_reference);

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////


using namespace Eigen;

//#################################################################
// Local empty constructor
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO() :
	StateP<DIM>(DIM+O_PARAM), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

//#################################################################
// Local constructor from size
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const unsigned int _size) :
	StateP<DIM>(_size), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

//#################################################################
// Local constructor from vector
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const VectorXs& _x) :
	StateP<DIM>(_x), //
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

//#################################################################
// Local constructor from StatePoint and StateOrientation
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const StatePoint& _p, const StateOrientation<O_PARAM>& _o) :
	StateP<DIM>(_p.x(), DIM+O_PARAM),
	o_(StateP<DIM>::state_estimated_local_,DIM,_o.x())
{
}

//#################################################################
// Local copy constructor
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(const StatePO<DIM,O_PARAM>& _state_p) :
	StateP<DIM>(_state_p.x()),
	o_(StateP<DIM>::state_estimated_local_, DIM)
{
}

//#################################################################
// Remote constructor from index and size
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
	StateP<DIM>(_st_remote,_idx, _size),
	o_(_st_remote, DIM)
{
	assert(_size >= DIM + O_PARAM);
}

//#################################################################
// Remote constructor from index and vector
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::StatePO(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
	StateP<DIM>(_st_remote,_idx, _x),
	o_(_st_remote, DIM)
{
	assert(_x.size() >= DIM + O_PARAM);
}

//#################################################################
// Destructor
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM>::~StatePO()
{
}

//#################################################################
// remap
template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::remap(VectorXs& _st_remote, const unsigned int _idx)
{
	StateP<DIM>::remap(_st_remote, _idx);
	o_.remap(_st_remote, _idx + DIM);
}

//#################################################################
// print
template<unsigned int DIM, orientationParametrization O_PARAM>
inline void StatePO<DIM,O_PARAM>::print() const
{
	StateP<DIM>::print();
	o_.print();
}

//#################################################################
// concatenate
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM> StatePO<DIM,O_PARAM>::concatenate(const StatePO<DIM,O_PARAM>& _s_local) const
{
	return StatePO<DIM,O_PARAM>(this->p_ + this->o_*_s_local.p_,// p
								this->o_ * _s_local.o_); 		  // o
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::concatenate(const StatePO<DIM,O_PARAM>& _s_local, StatePO<DIM,O_PARAM>& _s_out) const
{
	_s_out.p_ = this->p_ + this->o_*_s_local.p_;
	_s_out.o_ = this->o_ * _s_local.o_;
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::concatenateInPlace(const StatePO<DIM,O_PARAM>& _s_local)
{
	this->p_ += this->o_*_s_local.p_;
	this->o_ *= _s_local.o_;
}

//#################################################################
// inverse
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM> StatePO<DIM,O_PARAM>::inverse() const
{
	return StatePO<DIM,O_PARAM>(this->p_.inverse(),this->o_.inverse());
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::inverse(StatePO<DIM,O_PARAM>& _s_out) const
{
	_s_out.p_ = this->p_.inverse();
	_s_out.o_ = this->o_.inverse();
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::makeInverse()
{
	this->p_.makeInverse();
	this->o_.makeInverse();
}

//#################################################################
// relative to
template<unsigned int DIM, orientationParametrization O_PARAM>
StatePO<DIM,O_PARAM> StatePO<DIM,O_PARAM>::relativeTo(const StatePO<DIM,O_PARAM>& _s_reference) const
{
	return StatePO<DIM,O_PARAM>(_s_reference.o_.inverse() * (this->p_ - _s_reference.p_),
								_s_reference.o_.inverse() * this->o_);
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::relativeTo(const StatePO<DIM,O_PARAM>& _s_reference, StatePO<DIM,O_PARAM>& _s_out) const
{
	_s_out.p_ = _s_reference.o_.inverse() * (this->p_ - _s_reference.p_);
	_s_out.o_ = _s_reference.o_.inverse() * this->o_;
}

template<unsigned int DIM, orientationParametrization O_PARAM>
void StatePO<DIM,O_PARAM>::makeRelativeTo(const StatePO<DIM,O_PARAM>& _s_reference)
{
	this->p_ = _s_reference.o_.inverse() * (this->p_ - _s_reference.p_);
	this->o_ = _s_reference.o_.inverse() * this->o_;
}

#endif /* STATE_PO_H_ */
