/*
 * StateBase.cpp
 *
 *  Created on: Oct 14, 2014
 *      \author: jvallve
 */

#include "state_base.h"

using namespace Eigen;

StateBase::StateBase(const unsigned int _size) :
        state_estimated_local_(_size), //
        state_estimated_map_(state_estimated_local_.data(), _size) //
{
//    state_estimated_local_ = VectorXs::Zero(_size); // makes sure it initializes to zero
}

StateBase::StateBase(const VectorXs& _x) :
        state_estimated_local_(_x), //
        state_estimated_map_(state_estimated_local_.data(), _x.size()) //
{

}

StateBase::StateBase(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        state_estimated_local_(), //
        state_estimated_map_( //
                ((_idx + _size <= _st_remote.size()) ? _st_remote.data() + _idx : NULL), //
                ((_idx + _size <= _st_remote.size()) ? _size : 0)) //
{
    assert(_idx + _size <= _st_remote.size());
}

StateBase::StateBase(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
        state_estimated_local_(), //
        state_estimated_map_( //
                (_idx + _x.size() <= _st_remote.size()) ? _st_remote.data() + _idx : NULL, //
                (_idx + _x.size() <= _st_remote.size()) ? _x.size() : 0) //
{
    assert(_idx + _x.size() <= _st_remote.size());
    state_estimated_map_ = _x;
}

StateBase::~StateBase()
{
}
