#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_


//std includes
//

//Wolf includes
#include "wolf.h"

class SensorBase
{
    protected:
        Eigen::VectorXs sensor_pose_;//sensor pose in the vehicle frame
        // TODO: VectorXs params_;
        bool generate_prior_; //flag indicating if this sensor generates the prior or not
    
    public:
        SensorBase(const Eigen::VectorXs & _sp);

        ~SensorBase();
        
        const Eigen::VectorXs * getSensorPose() const;
        
};
#endif

