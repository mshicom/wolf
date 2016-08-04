#ifndef SENSOR_FIX_H_
#define SENSOR_FIX_H_

//wolf
#include "sensor_base.h"

namespace wolf {

class SensorFix : public SensorBase
{
    public:
        //pointer to sensor position, orientation, bias, init vehicle position and orientation
        SensorFix();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/
        virtual ~SensorFix();

    public:
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p, const IntrinsicsBase* _intrinsics);

};

} // namespace wolf

#endif //SENSOR_FIX_H_
