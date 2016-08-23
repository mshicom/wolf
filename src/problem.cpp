#include "problem.h"
#include "constraint_base.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "hardware_base.h"
#include "trajectory_base.h"
#include "trajectory_window.h"
#include "map_base.h"
#include "processor_motion.h"
#include "sensor_base.h"
#include "sensor_gps.h"
#include "capture_fix.h"
#include "factory.h"
#include "sensor_factory.h"
#include "processor_factory.h"

namespace wolf
{

// unnamed namespace used for helper functions local to this file.
namespace
{
std::string uppercase(std::string s) {for (auto & c: s) c = std::toupper(c); return s;}
}


Problem::Problem(const FrameStructure _frame_structure, const unsigned int _max_window_keyframes) :
        NodeBase("PROBLEM", ""), //
        location_(TOP),
        trajectory_ptr_(_max_window_keyframes == 0 ? new TrajectoryBase(_frame_structure) : new TrajectoryWindow(_frame_structure, _max_window_keyframes)),
        map_ptr_(new MapBase),
        hardware_ptr_(new HardwareBase),
        processor_motion_ptr_(nullptr)
{
    trajectory_ptr_->linkToUpperNode(this);
    map_ptr_->linkToUpperNode(this);
    hardware_ptr_->linkToUpperNode(this);
}

Problem::~Problem()
{
    hardware_ptr_->destruct();
    trajectory_ptr_->destruct();
    map_ptr_->destruct();
}

void Problem::destruct()
{
    delete this;
}

void Problem::addSensor(SensorBase* _sen_ptr)
{
    getHardwarePtr()->addSensor(_sen_ptr);
}

SensorBase* Problem::installSensor(const std::string& _sen_type, //
                                   const std::string& _unique_sensor_name, //
                                   const Eigen::VectorXs& _extrinsics, //
                                   IntrinsicsBase* _intrinsics)
{
    SensorBase* sen_ptr = SensorFactory::get().create(uppercase(_sen_type), _unique_sensor_name, _extrinsics, _intrinsics);
    addSensor(sen_ptr);
    return sen_ptr;
}

SensorBase* Problem::installSensor(const std::string& _sen_type, //
                                   const std::string& _unique_sensor_name, //
                                   const Eigen::VectorXs& _extrinsics, //
                                   const std::string& _intrinsics_filename)
{
    IntrinsicsBase* intr_ptr = IntrinsicsFactory::get().create(_sen_type, _intrinsics_filename);
    return installSensor(_sen_type, _unique_sensor_name, _extrinsics, intr_ptr);
}

ProcessorBase* Problem::installProcessor(const std::string& _prc_type, //
                                         const std::string& _unique_processor_name, //
                                         SensorBase* _corresponding_sensor_ptr, //
                                         ProcessorParamsBase* _prc_params)
{
    ProcessorBase* prc_ptr = ProcessorFactory::get().create(uppercase(_prc_type), _unique_processor_name, _prc_params);
    _corresponding_sensor_ptr->addProcessor(prc_ptr);

    // setting the main processor motion
    if (prc_ptr->isMotion() && processor_motion_ptr_ == nullptr)
        processor_motion_ptr_ = (ProcessorMotion*)prc_ptr;

    return prc_ptr;
}

void Problem::installProcessor(const std::string& _prc_type, //
                               const std::string& _unique_processor_name, //
                               const std::string& _corresponding_sensor_name, //
                               const std::string& _params_filename)
{
    SensorBase* sen_ptr = getSensorPtr(_corresponding_sensor_name);
    if (sen_ptr == nullptr)
        throw std::runtime_error("Sensor not found. Cannot bind Processor.");
    if (_params_filename == "")
        installProcessor(_prc_type, _unique_processor_name, sen_ptr, nullptr);
    else
    {
        ProcessorParamsBase* prc_params = ProcessorParamsFactory::get().create(_prc_type, _params_filename);
        installProcessor(_prc_type, _unique_processor_name, sen_ptr, prc_params);
    }
}

void Problem::setProcessorMotion(ProcessorMotion* _processor_motion_ptr)
{
    processor_motion_ptr_ = _processor_motion_ptr;
}

FrameBase* Problem::createFrame(FrameKeyType _frame_type, const TimeStamp& _time_stamp)
{
    return createFrame(_frame_type, getStateAtTimeStamp(_time_stamp), _time_stamp);
}

FrameBase* Problem::createFrame(FrameKeyType _frame_type, const Eigen::VectorXs& _frame_state,
                                const TimeStamp& _time_stamp)
{
    //std::cout << "Problem::createFrame" << std::endl;
    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
        {
            assert(_frame_state.size() == 3 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(2)),
                                  new StateBlock(_frame_state.tail(1))));
        }
        case FRM_PO_3D:
        {
            assert(_frame_state.size() == 7 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(3)),
                                  new StateQuaternion(_frame_state.tail(4))));
        }
        case FRM_POV_3D:
        {
            assert(_frame_state.size() == 10 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(3)),
                                  new StateQuaternion(_frame_state.segment<4>(3)),
                                  new StateBlock(_frame_state.tail(3))));
        }
        default:
            throw std::runtime_error(
                    "Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
    return trajectory_ptr_->getLastFramePtr();
}

Eigen::VectorXs Problem::getCurrentState()
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state);
    return state;
}

Eigen::VectorXs Problem::getCurrentState(TimeStamp& ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state, ts);
    return state;
}

void Problem::getCurrentState(Eigen::VectorXs& state)
{
    TimeStamp ts;
    getCurrentState(state, ts);
}


void Problem::getCurrentState(Eigen::VectorXs& state, TimeStamp& ts)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->getCurrentState(state, ts);
    else if (trajectory_ptr_->getLastKeyFramePtr() != nullptr)
    {
        trajectory_ptr_->getLastKeyFramePtr()->getTimeStamp(ts);
        trajectory_ptr_->getLastKeyFramePtr()->getState(state);
    }
    else
        state = zeroState();
}

void Problem::getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getStateAtTimeStamp: bad state size");

    if (processor_motion_ptr_ != nullptr && processor_motion_ptr_->getOriginCapturePtr() != nullptr)
        processor_motion_ptr_->getState(_ts, state);
    else
    {
        FrameBase* closest_frame = trajectory_ptr_->closestKeyFrameToTimeStamp(_ts);
        if (closest_frame != nullptr)
            closest_frame->getState(state);
        else
            state = zeroState();
    }
}

Eigen::VectorXs Problem::getStateAtTimeStamp(const TimeStamp& _ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getStateAtTimeStamp(_ts, state);
    return state;
}

unsigned int Problem::getFrameStructureSize()
{
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
            return 3;
        case FRM_PO_3D:
            return 7;
        case FRM_POV_3D:
            return 10;
        default:
            throw std::runtime_error(
                    "Problem::getFrameStructureSize(): Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
}

Eigen::VectorXs Problem::zeroState()
{
    Eigen::VectorXs state = Eigen::VectorXs::Zero(getFrameStructureSize());
    if (trajectory_ptr_->getFrameStructure() == FRM_PO_3D || trajectory_ptr_->getFrameStructure() == FRM_POV_3D)
        state(6) = 1;
    return state;
}

bool Problem::permitKeyFrame(ProcessorBase* _processor_ptr)
{
    return true;
}

void Problem::keyFrameCallback(FrameBase* _keyframe_ptr, ProcessorBase* _processor_ptr, const Scalar& _time_tolerance)
{
    //std::cout << "Problem::keyFrameCallback: processor " << (_processor_ptr == nullptr ? "fix" : _processor_ptr->getName()) << std::endl;
    for (auto sensor : (*hardware_ptr_->getSensorListPtr()))
    	for (auto processor : (*sensor->getProcessorListPtr()))
    		if (processor != _processor_ptr)
                processor->keyFrameCallback(_keyframe_ptr, _time_tolerance);
}

LandmarkBase* Problem::addLandmark(LandmarkBase* _lmk_ptr)
{
    getMapPtr()->addLandmark(_lmk_ptr);
    return _lmk_ptr;
}

void Problem::addLandmarkList(LandmarkBaseList _lmk_list)
{
    //std::cout << "Problem::addLandmarkList" << std::endl;
    getMapPtr()->addLandmarkList(_lmk_list);
}

StateBlock* Problem::addStateBlockPtr(StateBlock* _state_ptr)
{
    //std::cout << "addStateBlockPtr" << std::endl;
    // add the state unit to the list
    state_block_ptr_list_.push_back(_state_ptr);
    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({ADD,_state_ptr}));

    return _state_ptr;
}

void Problem::updateStateBlockPtr(StateBlock* _state_ptr)
{
    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({UPDATE,_state_ptr}));
}

void Problem::removeStateBlockPtr(StateBlock* _state_ptr)
{
    // add the state unit to the list
    state_block_ptr_list_.remove(_state_ptr);

    // Check if the state addition is still as a notification
    auto state_found_it = state_block_notification_list_.end();
    for (auto state_notif_it = state_block_notification_list_.begin(); state_notif_it != state_block_notification_list_.end(); state_notif_it++)
    {
        if (state_notif_it->notification_ == ADD && state_notif_it->state_block_ptr_ == _state_ptr)
        {
            state_found_it = state_notif_it;
            break;
        }
    }
    // Remove addition notification
    if (state_found_it != state_block_notification_list_.end())
    	state_block_notification_list_.erase(state_found_it);
    // Add remove notification
    else
    	state_block_notification_list_.push_back(StateBlockNotification({REMOVE, nullptr, _state_ptr->getPtr()}));

}

ConstraintBase* Problem::addConstraintPtr(ConstraintBase* _constraint_ptr)
{
    //std::cout << "addConstraintPtr" << std::endl;
    // queue for solver manager
    constraint_notification_list_.push_back(ConstraintNotification({ADD, _constraint_ptr, _constraint_ptr->id()}));

    return _constraint_ptr;
}

void Problem::removeConstraintPtr(ConstraintBase* _constraint_ptr)
{
    // Check if the constraint addition is still as a notification
    auto ctr_found_it = constraint_notification_list_.end();
    for (auto ctr_notif_it = constraint_notification_list_.begin(); ctr_notif_it != constraint_notification_list_.end(); ctr_notif_it++)
    {
        if (ctr_notif_it->notification_ == ADD && ctr_notif_it->constraint_ptr_ == _constraint_ptr)
        {
            ctr_found_it = ctr_notif_it;
            break;
        }
    }
    // Remove addition notification
    if (ctr_found_it != constraint_notification_list_.end())
        constraint_notification_list_.erase(ctr_found_it);
    // Add remove notification
    else
        constraint_notification_list_.push_back(ConstraintNotification({REMOVE, nullptr, _constraint_ptr->id()}));
}

void Problem::clearCovariance()
{
    covariances_.clear();
}

void Problem::addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] = _cov;
}

bool Problem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row,
                                 const int _col)
{
    //std::cout << "entire cov to be filled:" << std::endl << _cov << std::endl;
    //std::cout << "_row " << _row << std::endl;
    //std::cout << "_col " << _col << std::endl;
    //std::cout << "_state1 size: " << _state1->getSize() << std::endl;
    //std::cout << "_state2 size: " << _state2->getSize() << std::endl;
    //std::cout << "part of cov to be filled:" << std::endl <<  _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) << std::endl;
    //if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] << std::endl;
    //else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose() << std::endl;

    assert(_row + _state1->getSize() <= _cov.rows() && _col + _state2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
       _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose();
    else
        return false;

    return true;
}

bool Problem::getFrameCovariance(FrameBase* _frame_ptr, Eigen::MatrixXs& _covariance)
{
    return getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getPPtr(), _covariance, 0, 0) &&
    getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _covariance, 0,_frame_ptr->getPPtr()->getSize()) &&
    getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getPPtr(), _covariance, _frame_ptr->getPPtr()->getSize(), 0) &&
    getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getOPtr(), _covariance, _frame_ptr->getPPtr()->getSize() ,_frame_ptr->getPPtr()->getSize());
}

Eigen::MatrixXs Problem::getFrameCovariance(FrameBase* _frame_ptr)
{
    Eigen::MatrixXs covariance = Eigen::MatrixXs::Zero(_frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize(), _frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize());
    getFrameCovariance(_frame_ptr, covariance);
    return covariance;
}

bool Problem::getLandmarkCovariance(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& _covariance)
{
    return getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), _covariance, 0, 0) &&
    getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), _covariance, 0,_landmark_ptr->getPPtr()->getSize()) &&
    getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getPPtr(), _covariance, _landmark_ptr->getPPtr()->getSize(), 0) &&
    getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), _covariance, _landmark_ptr->getPPtr()->getSize() ,_landmark_ptr->getPPtr()->getSize());
}

Eigen::MatrixXs Problem::getLandmarkCovariance(LandmarkBase* _landmark_ptr)
{
    Eigen::MatrixXs covariance = Eigen::MatrixXs::Zero(_landmark_ptr->getPPtr()->getSize()+_landmark_ptr->getOPtr()->getSize(), _landmark_ptr->getPPtr()->getSize()+_landmark_ptr->getOPtr()->getSize());
    getLandmarkCovariance(_landmark_ptr, covariance);
    return covariance;
}

MapBase* Problem::addMap(MapBase* _map_ptr)
{
    // TODO: not necessary but update map maybe..
    map_ptr_ = _map_ptr;
    map_ptr_->linkToUpperNode(this);

    return map_ptr_;
}

TrajectoryBase* Problem::addTrajectory(TrajectoryBase* _trajectory_ptr)
{
    trajectory_ptr_ = _trajectory_ptr;
    trajectory_ptr_->linkToUpperNode(this);

    return trajectory_ptr_;
}

MapBase* Problem::getMapPtr()
{
    return map_ptr_;
}

TrajectoryBase* Problem::getTrajectoryPtr()
{
    return trajectory_ptr_;
}

HardwareBase* Problem::getHardwarePtr()
{
    return hardware_ptr_;
}

FrameBase* Problem::getLastFramePtr()
{
    return trajectory_ptr_->getLastFramePtr();
}

FrameBase* Problem::getLastKeyFramePtr()
{
    return trajectory_ptr_->getLastKeyFramePtr();;
}

StateBlockList* Problem::getStateListPtr()
{
    return &state_block_ptr_list_;
}

SensorBase* Problem::getSensorPtr(const std::string& _sensor_name)
{
    return hardware_ptr_->findSensor(_sensor_name);
}

void Problem::setPoseEstimation(const Eigen::VectorXs& _new_pose, const Eigen::MatrixXs& _new_cov, const TimeStamp& _ts)
{
    //std::cout << "setPoseEstimation:" << std::endl;

    // store the prior of the keyframe
    Eigen::VectorXs old_pose(getStateAtTimeStamp(_ts));

    // store transformation
    // FIXME: generalize for all frame structures
    Eigen::Vector1s theta_rotation;
    theta_rotation(0) = _new_pose(2) - old_pose(2);
    Eigen::Vector2s translation = _new_pose.head<2>() - Eigen::Rotation2Ds(theta_rotation(0))*old_pose.head<2>();
    Eigen::Rotation2Ds rotation = Eigen::Rotation2Ds(theta_rotation(0));

    // Find sensor fix ptr (and create it if it's not found)
    SensorBase* sensor_fix = hardware_ptr_->findSensor("initial pose");
    if (sensor_fix == nullptr)
        sensor_fix = installSensor("FIX", "initial pose", Eigen::VectorXs::Zero(0), nullptr );

    // Transform all frames
    //bool prev_fix_removed = false;
    for (auto frame_ptr : *(trajectory_ptr_->getFrameListPtr()))
    {
        // transform frame
        // FIXME: generalize for all frame structures
        frame_ptr->getPPtr()->setVector(rotation * frame_ptr->getPPtr()->getVector() + translation);
        frame_ptr->getOPtr()->setVector(frame_ptr->getOPtr()->getVector() + theta_rotation);
    }

    // Remove all non-odometry constraints
    ConstraintBaseList all_constraints;
    trajectory_ptr_->getConstraintList(all_constraints);
    for (auto ctr_ptr : all_constraints)
    	if (ctr_ptr->getCategory() != CTR_FRAME)
    		ctr_ptr->destruct();

    // TODO: matching and constraints for all old captures's features

    // create fix capture
    FrameBase* keyframe_ptr = createFrame(KEY_FRAME, _new_pose, _ts);
    CaptureFix* fix_capture = new CaptureFix(_ts, sensor_fix, _new_pose, _new_cov);
    keyframe_ptr->addCapture(fix_capture);
    fix_capture->process();

    // notify processors about the new keyframe
    keyFrameCallback(keyframe_ptr, nullptr , 0.1);
}

void Problem::loadMap(const std::string& _filename_dot_yaml)
{
    getMapPtr()->load(_filename_dot_yaml);
}

void Problem::saveMap(const std::string& _filename_dot_yaml, const std::string& _map_name)
{
    getMapPtr()->save(_filename_dot_yaml, _map_name);
}

} // namespace wolf
