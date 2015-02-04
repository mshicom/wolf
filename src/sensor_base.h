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
        SensorType type_;
        // TODO: VectorXs params_;
        bool generate_prior_; //flag indicating if this sensor generates the prior or not
    
    public:
        SensorBase(const SensorType & _tp, const Eigen::VectorXs & _sp);

        ~SensorBase();
        
        const Eigen::VectorXs * getSensorPose() const;
        
        const SensorType getSensorType() const;

};
#endif

