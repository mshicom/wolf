#include "wolf_problem.h"

WolfProblem::WolfProblem(unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
		state_(_size),
		state_idx_last_(0),
        location_(TOP),
		map_ptr_(new MapBase),
		trajectory_ptr_(new TrajectoryBase),
        reallocated_(false)
{
//	map_ptr_ = new MapBase;
//	trajectory_ptr_ = new TrajectoryBase;
    std::cout << "WolfProblem::WolfProblem(): " << __LINE__ << std::endl;
	map_ptr_->linkToUpperNode( this );
    std::cout << "WolfProblem::WolfProblem(): " << __LINE__ << std::endl;
	trajectory_ptr_->linkToUpperNode( this );
    std::cout << "WolfProblem::WolfProblem(): " << __LINE__ << std::endl;
}

WolfProblem::WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr, unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
		state_(_size),
		state_idx_last_(0),
        location_(TOP),
		map_ptr_(_map_ptr==nullptr ? new MapBase : _map_ptr),
		trajectory_ptr_(_trajectory_ptr==nullptr ? new TrajectoryBase : _trajectory_ptr),
        reallocated_(false)
{
	map_ptr_->linkToUpperNode( this );
	trajectory_ptr_->linkToUpperNode( this );
}

WolfProblem::~WolfProblem()
{
	delete trajectory_ptr_;
	delete map_ptr_;
}

bool WolfProblem::addState(StateBase* _new_state_ptr, const Eigen::VectorXs& _new_state_values)
{
	// Check if resize should be done
	if (state_idx_last_+_new_state_ptr->getStateSize() > state_.size())
	{
		std::cout << "\nState size: " << state_.size() << " last idx: " << state_idx_last_ << " last idx + new state size: " << state_idx_last_+_new_state_ptr->getStateSize() << std::endl;
		std::cout << "Resizing state and remapping al state units..." << std::endl;
		WolfScalar* old_first_pointer = state_.data();
		state_.resize(state_.size()*2);
		for (auto state_units_it = state_list_.begin(); state_units_it != state_list_.end(); state_units_it++)
			(*state_units_it)->setPtr(state_.data() + ( (*state_units_it)->getPtr() - old_first_pointer) );
		std::cout << "New state size: " << state_.size() << "last idx: " << state_idx_last_ << std::endl;
		reallocated_ = true;
	}
	//std::cout << "\nnew state unit: " << _new_state_values.transpose() << std::endl;
	//std::cout << "\nPrev state: " << state_.segment(0,state_idx_last_).transpose() << std::endl;

	// copy the values of the new state
	assert(_new_state_values.size() == _new_state_ptr->getStateSize() && "Different state unit and vector sizes");
	state_.segment(state_idx_last_,_new_state_ptr->getStateSize()) = _new_state_values;

	// add the state unit to the list
	state_list_.push_back(_new_state_ptr);

	// update the last state index
	state_idx_last_ += _new_state_ptr->getStateSize();

	//std::cout << "\nPost state: " << state_.segment(0,state_idx_last_).transpose() << std::endl;
	return reallocated_;
}

void WolfProblem::removeState(StateBase* _state_ptr)
{
	// TODO: Reordering? Mandatory for filtering...
	state_list_.remove(_state_ptr);
	removed_state_ptr_list_.push_back(_state_ptr->getPtr());
	delete _state_ptr;
}

WolfScalar* WolfProblem::getStatePtr()
{
	return state_.data();
}

WolfScalar* WolfProblem::getNewStatePtr()
{
	return state_.data()+state_idx_last_;
}

const unsigned int WolfProblem::getStateSize() const
{
	return state_idx_last_;
}

void WolfProblem::addMap(MapBase* _map_ptr)
{
	map_ptr_ = _map_ptr;
	map_ptr_->linkToUpperNode( this );
}

void WolfProblem::addTrajectory(TrajectoryBase* _trajectory_ptr)
{
	trajectory_ptr_ = _trajectory_ptr;
	trajectory_ptr_->linkToUpperNode( this );
}

MapBase* WolfProblem::getMapPtr()
{
	return map_ptr_;
}

TrajectoryBase* WolfProblem::getTrajectoryPtr()
{
	return trajectory_ptr_;
}

StateBaseList* WolfProblem::getStateListPtr()
{
	return &state_list_;
}

std::list<WolfScalar*>* WolfProblem::getRemovedStateListPtr()
{
	return &removed_state_ptr_list_;
}

void WolfProblem::print(unsigned int _ntabs, std::ostream& _ost) const
{
    printSelf(_ntabs, _ost); //one line
    printLower(_ntabs, _ost);
}

void WolfProblem::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << nodeLabel() << " " << nodeId() << " : ";
    _ost << "TOP" << std::endl;
}

const Eigen::VectorXs WolfProblem::getState() const
{
	return state_;
}

bool WolfProblem::isReallocated() const
{
	return reallocated_;
}


void WolfProblem::reallocationDone()
{
	reallocated_ = false;
}

WolfProblem* WolfProblem::getTop()
{
	return this;
}

void WolfProblem::printLower(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << "\tLower Nodes  ==> [ ";
    _ost << map_ptr_->nodeId() << " ";
    _ost << trajectory_ptr_->nodeId() << " ]" << std::endl;
    _ntabs++;
	map_ptr_->print(_ntabs, _ost);
	trajectory_ptr_->print(_ntabs, _ost);
}
