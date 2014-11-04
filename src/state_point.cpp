/*
 * state_point.cpp
 *
 *  Created on: Oct 23, 2014
 *      \author: jvallve
 */

#include "state_point.h"

using namespace Eigen;

StatePoint::StatePoint(const unsigned int _dim) :
        StateBase(_dim) //
{
}

StatePoint::StatePoint(const VectorXs& _p) :
		StateBase(_p) //
{
}

StatePoint::StatePoint(const StatePoint& _x) :
		StateBase(_x.x()) //
{
}

StatePoint::StatePoint(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _dim) :
        StateBase(_st_remote,_idx, _dim) //
{
}

StatePoint::StatePoint(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _p) :
		StateBase(_st_remote,_idx, _p) //
{
}

StatePoint::~StatePoint()
{
}

void StatePoint::print() const
{
	StateBase::print();
	std::cout << std::endl;
}
