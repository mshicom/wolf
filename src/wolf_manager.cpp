#include "wolf_manager.h"


namespace wolf {

WolfManager::WolfManager(const FrameStructure _frame_structure,
                         SensorBase* _sensor_prior_ptr,
                         const Eigen::VectorXs& _prior,
                         const Eigen::MatrixXs& _prior_cov,
                         const unsigned int& _trajectory_size,
                         const Scalar& _new_frame_elapsed_time) :
        problem_(new Problem(_frame_structure)),
        sensor_prior_(_sensor_prior_ptr),
        current_frame_(nullptr),
        last_key_frame_(nullptr),
        last_capture_relative_(nullptr),
        trajectory_size_(_trajectory_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
{
    if (_frame_structure == FRM_PO_2D)
        assert( _prior.size() == 3 &&
                _prior_cov.cols() == 3 &&
                _prior_cov.rows() == 3 &&
                "Wrong init_frame state vector or covariance matrix size");
    else
        assert( _prior.size() == 7 &&
                _prior_cov.cols() == 7 &&
                _prior_cov.rows() == 7 &&
                "Wrong init_frame state vector or covariance matrix size");

    //std::cout << "initializing wolfmanager" << std::endl;

    // Initial frame
    createFrame(_prior, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
    //std::cout << " first_window_frame_" << std::endl;

    // Initial covariance
    SensorBase* prior_sensor = new SensorBase(SEN_ABSOLUTE_POSE, "ABSOLUTE POSE", nullptr, nullptr, nullptr, 0);
    problem_->getHardwarePtr()->addSensor(prior_sensor);
    CaptureFix* initial_covariance = new CaptureFix(TimeStamp(0), prior_sensor, _prior, _prior_cov);
    //std::cout << " initial_covariance" << std::endl;
    current_frame_->addCapture(initial_covariance);
    //std::cout << " addCapture" << std::endl;

    // Current robot frame
    createFrame(_prior, TimeStamp(0));

    //std::cout << " wolfmanager initialized" << std::endl;
}

WolfManager::~WolfManager()
{
    //std::cout << "deleting wolf manager..." << std::endl;
    problem_->destruct();
}

void WolfManager::createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame..." << std::endl;

    // current frame -> KEYFRAME
    last_key_frame_ = current_frame_;

    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    problem_->createFrame(KEY_FRAME, _frame_state, _time_stamp);
    //std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
    //std::cout << "current_frame_" << std::endl;

    // Zero odometry (to be integrated)
    if (last_key_frame_ != nullptr)
    {
        CaptureMotion* empty_odom = new CaptureOdom2D(_time_stamp, _time_stamp, sensor_prior_, Eigen::Vector3s::Zero());
        current_frame_->addCapture(empty_odom);
        empty_odom->process();
        last_capture_relative_ = empty_odom;
    }
    //std::cout << "last_key_frame_" << std::endl;

    // ---------------------- KEY FRAME ---------------------
    if (last_key_frame_ != nullptr)
    {
        last_key_frame_->setKey();
        //std::cout << "Processing last frame non-odometry captures " << current_frame_->getCaptureListPtr()->size() << std::endl;
        for (auto capture_it = last_key_frame_->getCaptureListPtr()->begin(); capture_it != last_key_frame_->getCaptureListPtr()->end(); capture_it++)
            if ((*capture_it)->getSensorPtr() != sensor_prior_)
            {
                //std::cout << "processing capture " << (*capture_it)->nodeId() << std::endl;
                (*capture_it)->process();
            }


    }
    //std::cout << "Last key frame non-odometry captures processed" << std::endl;

    // ---------------------- MANAGE WINDOW OF POSES ---------------------
    manageWindow();
    //std::cout << "new frame created" << std::endl;
}


void WolfManager::createFrame(const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame from prior..." << std::endl;
    createFrame(last_capture_relative_->computeFramePose(_time_stamp), _time_stamp);
}

void WolfManager::addSensor(SensorBase* _sensor_ptr)
{
    //std::cout << "adding sensor... to hardware " << problem_->getHardwarePtr()->nodeId() << std::endl;
    problem_->addSensor(_sensor_ptr);
    //std::cout << "added!" << std::endl;
}

void WolfManager::addCapture(CaptureBase* _capture)
{
    new_captures_.push(_capture);
    //std::cout << "added new capture: " << _capture->nodeId() << " stamp: ";
    //std::cout << std::endl;
}


void WolfManager::manageWindow()
{
    //std::cout << "managing window..." << std::endl;
    // WINDOW of FRAMES (remove or fix old frames)
    if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > trajectory_size_+1)
    {
        //std::cout << "first_window_frame_ " << (*first_window_frame_)->nodeId() << std::endl;
        //problem_->getTrajectoryPtr()->removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
        (*first_window_frame_)->fix();
        first_window_frame_++;
    }
    //std::cout << "window managed" << std::endl;
}


bool WolfManager::checkNewFrame(CaptureBase* new_capture)
{
    //std::cout << "checking if new frame..." << std::endl;
    // TODO: not only time, depending on the sensor...
    //std::cout << new_capture->getTimeStamp().get() - last_frame_->getTimeStamp().get() << std::endl;
    return new_capture->getTimeStamp().get() - last_key_frame_->getTimeStamp().get() > new_frame_elapsed_time_;
}


void WolfManager::update()
{
    //std::cout << "updating..." << std::endl;
    while (!new_captures_.empty())
    {
        // EXTRACT NEW CAPTURE
        CaptureBase* new_capture = new_captures_.front();
        new_captures_.pop();

        // OVERWRITE CURRENT STAMP
        current_frame_->setTimeStamp(new_capture->getTimeStamp());

        // INITIALIZE FIRST FRAME STAMP
        if (last_key_frame_->getTimeStamp().get() == 0)
            last_key_frame_->setTimeStamp(new_capture->getTimeStamp());

        // NEW KEY FRAME ?
        if (checkNewFrame(new_capture))
            createFrame(new_capture->getTimeStamp());

        // ODOMETRY SENSOR
        if (new_capture->getSensorPtr() == sensor_prior_)
        {
            //std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD/INTEGRATE NEW ODOMETRY TO THE LAST FRAME
            last_capture_relative_->integrateCapture((CaptureMotion*) (new_capture));
            current_frame_->setState(last_capture_relative_->computeFramePose(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
            delete new_capture;
        }
        else
        {
            //std::cout << "adding not odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD CAPTURE TO THE CURRENT FRAME (or substitute the same sensor previous capture)
            //std::cout << "searching repeated capture..." << new_capture->nodeId() << std::endl;
            CaptureBase* repeated_capture_ptr = current_frame_->findCaptureOfSensor(new_capture->getSensorPtr());

            if (repeated_capture_ptr != nullptr) // repeated capture
            {
                //std::cout << "repeated capture, keeping new capture" << new_capture->nodeId() << std::endl;
                current_frame_->removeCapture(repeated_capture_ptr);
                current_frame_->addCapture(new_capture);
            }
            else
            {
                //std::cout << "not repeated, adding capture..." << new_capture->nodeId() << std::endl;
                current_frame_->addCapture(new_capture);
            }
        }
    }
    //std::cout << "updated" << std::endl;
}


Eigen::VectorXs WolfManager::getVehiclePose(const TimeStamp& _now)
{
    //std::cout << "getting vehicle pose... " << std::endl;

    if (last_capture_relative_ == nullptr)
        return current_frame_->getState();
    else
    {
        //std::cout << "last capture relative... " << std::endl;
        return last_capture_relative_->computeFramePose(_now);
    }
}


Problem* WolfManager::getProblemPtr()
{
    return problem_;
}


void WolfManager::setWindowSize(const unsigned int& _size)
{
    trajectory_size_ = _size;
}


void WolfManager::setNewFrameElapsedTime(const Scalar& _dt)
{
    new_frame_elapsed_time_ = _dt;
}


} // namespace wolf
