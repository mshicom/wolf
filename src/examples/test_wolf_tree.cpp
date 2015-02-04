// Testing a full wolf tree avoiding template classes for NodeLinked derived classes

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//ceres
#include "ceres/ceres.h"

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "capture_base.h"


class SensorBase
{
    protected:
        Eigen::VectorXs sensor_pose_;//sensor pose in the vehicle frame
    
    public:
        SensorBase(const Eigen::VectorXs & _sp) : 
            sensor_pose_(_sp)
        {
            //
        };
        
        ~SensorBase()
        {
            //
        };
        
        const Eigen::VectorXs * getSensorPose() const
        {   
            return & sensor_pose_;
        };
        
};

int main(int argc, char** argv) 
{    
    std::queue<FrameBase> trajectory; //this will be the main object of the wolf manager 
    Eigen::VectorXs sp(6);
    sp << 0.1,0.1,0.1,0,0,0;
    SensorBase sensor1(sp); //just one sensor. This will be owned by the manager
    sp << 0.2,0.2,0.2,0,0,0;
    SensorBase sensor2(sp); //just another sensor. This will be owned by the manager
    std::shared_ptr<CaptureBase> capture; //specialized class will be placed on each ROS callback
    TimeStamp ros_ts; //this plays the role of ros::Time
    Eigen::VectorXs sensor_reading(4); //this plays the role of the ROS message content (sensor reading). Reading of dim=4 (example)
    
    //Welcome message
    std::cout << std::endl << " ========= WOLF TREE test ===========" << std::endl << std::endl;

    //main loop
    for (unsigned int ii = 0; ii<10; ii++)
    {
        //1. a new sensor data arrives (this part will be placed on ROS callbacks)
        ros_ts.setToNow();
        sensor_reading << 1,2,3,4;
        capture.reset( new CaptureBase(ros_ts.get(), &sensor1) ); 
        capture->setData(sensor_reading.size(), sensor_reading.data());
        capture->processCapture(); //This should create features
        //TODO: add capture to the recent_captures_list
        
        //2. Process recent_captures_list, deciding for each new capture wheter a Frame has to be created or they have to be linked to the last one
        
        //3. Stablish correspondences
        
        //4. Call ceres solver
        
        //5. publish results
        
    }
    
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
