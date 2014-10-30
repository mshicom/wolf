/*
 * state_base.h
 *
 *  Created on: Oct 14, 2014
 *      \author: jvallve
 */

#ifndef STATE_BASE_H_
#define STATE_BASE_H_

// wolf
#include "wolf.h"
#include <iostream>

/** \brief Base class for all kind of states.
 * 
 * This class is the base for all state classes. Conceptually, it has a vector which represents the state, but this vector is stored either locally or remotely, depending on how the object has been constructed:
 *      - Local Case: an Eigen VectorXs member is stored by the object itself.
 *      - Remote Case: an Eigen Map member maps a remote storage Vector, so the object does not allocate memory for state Vector
 * In the local case the map member is pointing to the local vector, thus in both cases the accessors implemented in the inherited classes will be the same, since these accessors are implemented on the map member. 
 * 
 */
class StateBase
{
    protected:

        Eigen::VectorXs state_estimated_local_;   ///< Local storage
        Eigen::Map<Eigen::VectorXs> state_estimated_map_; ///< mapped vector, to remote storage

    protected:

        // Local Constructors
        /**
         * Local constructor from size. Map member will map local vector.
         * \param _size size of the state vector
         */
        StateBase(const unsigned int _size);

        /**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StateBase(const Eigen::VectorXs& _x);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StateBase();

    public:

        /**
         * Get reference to stateEstimatedLocal
         */
        Eigen::VectorXs& stateEstimatedLocal();

        /**
         * Get a const reference to stateEstimatedLocal
         */
        const Eigen::VectorXs& stateEstimatedLocal() const;

        /**
         * Get reference to stateEstimatedMap
         */
        Eigen::Map<Eigen::VectorXs>& stateEstimatedMap();

        /**
         * Get a const reference to stateEstimatedMap
         */
        const Eigen::Map<Eigen::VectorXs>& stateEstimatedMap() const;

        /**
         * Change the mapped positions in the remote vector of the stateEstimatedMap
         */
        virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /**
         * Get the size of the state
         */
        unsigned int size() const;

        // Setters and getters

        /**
         * Get reference to state
         */
        Eigen::Map<Eigen::VectorXs>& x();

        /**
         * Get reference to state
         */
        const Eigen::Map<Eigen::VectorXs>& x() const;

        /**
         * Set state vector
         * \param _x the state vector
         */
        void x(const Eigen::VectorXs& _x);

        /**
		 * Print the state vector
		 */
        virtual void print() const;
};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////



inline const Eigen::VectorXs& StateBase::stateEstimatedLocal() const
{
    return state_estimated_local_;
}

inline Eigen::VectorXs& StateBase::stateEstimatedLocal()
{
    return state_estimated_local_;
}

inline const Eigen::Map<Eigen::VectorXs>& StateBase::stateEstimatedMap() const
{
    return state_estimated_map_;
}

inline Eigen::Map<Eigen::VectorXs>& StateBase::stateEstimatedMap()
{
    return state_estimated_map_;
}

inline void StateBase::remap(Eigen::VectorXs& _st_remote, const unsigned int _idx)
{
    new (&state_estimated_map_) Eigen::Map<Eigen::VectorXs>(&_st_remote(_idx), this->size());
}

inline unsigned int StateBase::size() const
{
    return state_estimated_map_.size();
}

inline void StateBase::x(const Eigen::VectorXs& _x)
{
    assert(state_estimated_map_.size() == _x.size());
    state_estimated_map_ = _x;
}

inline const Eigen::Map<Eigen::VectorXs>& StateBase::x() const
{
    return state_estimated_map_;
}

inline Eigen::Map<Eigen::VectorXs>& StateBase::x()
{
    return state_estimated_map_;
}

inline void StateBase::print() const
{
    std::cout << "    " << state_estimated_map_.transpose();
}


#endif /* STATE_BASE_H_ */
