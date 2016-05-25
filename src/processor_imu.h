#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "capture_imu.h"
#include "wolf.h"

// STL
#include <deque>

#include "sensor_imu.h"

namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();

        // not redefining main operations (they should be the same for all derived classes)

    protected:

//        virtual void preProcess(){}
//        virtual void postProcess(){}

        // Helper functions


        /**
         * @brief extractData Extract data from the capture_imu object and store them
         * @param _data : acceleration (data[0:2]), angular rates (data[3:5]), (bias random walks) -> 6-vector (12-vector with biases)
         *                acceleration bias (data[6:8]), gyroscope bias (data[9:11])
         * @param _data_cov
         * @param _dt
         * @param _delta :  deltas of position, orientation vector, velocity, biases -> 15-vector
         * @param _delta_cov
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov)
        {
            //Euler integration (zero-order integration) for inputs, non-integration for perturbation (perturbation is already integrated)
//            adt = (am - ab)*dt - an;
//            wdt = (wm - wb)*dt - wn;
            Eigen::Vector6s random_walk() = Eigen::Vector6s::Random();
            Eigen::Vector6s data_euler_int;
            data_euler_int = (_data.head<6>() - _data.tail<6>())*_dt - random_walk();
//            ADT_am = dt;
//            WDT_wm = dt;
//            ADT_ab = -dt;
//            WDT_wb = -dt;
//            ADT_an = -1;
//            WDT_wn = -1;

            //exponential map
            //TODO : DEFINE EXPONENTIAL MAP METHOD WITH COMPUTATION OF JACOBIAN
            Eigen::Matrix<4,3,wolf::Scalar> DQ_wdt;
            //exp(date_euler_int.tail<3>(), _delta.segment(3,4), DQ_wdt);

            //projection onto the manifold
            //TODO : PROJECTION METHOD WITH COMPUTATION OF JACOBIAN TO BE DEFINED
            //[dv, DV_adt, DV_dq] = qRot(adt,dq); --> see act and *
            Eigen::Matrix3s DV_adt;
            Eigen::Matrix3s DV_dq;
            _delta.segment(7,3) = qRot(_delta.segment(3,4),data_euler_int.head<3>(), DV_adt, DV_dq);
            _delta.head<3>() = 1.5*_delta.segment<7,3>()*dt;
            _delta.tail<6>() = _data.tail<6>(); //BIAS ??
            Scalar DP_dv = 1.5*dt;
            Eigen::MatrixXs DP_adt(1.5*dt*DV_adt);

            /// Compute jacobians
            /// Jacobian wrt bias perturbation in discrete time
            //        ar     wr
            //   px  _ _ _ | _ _ _
            //   py  _ _ _ | _ _ _
            //   pz  _ _ _ | _ _ _
            //   qw  _ _ _ | _ _ _
            //   qx  _ _ _ | _ _ _
            //   qy  _ _ _ | _ _ _
            //   qz  _ _ _ | _ _ _
            //   vx  _ _ _ | _ _ _
            //   vy  _ _ _ | _ _ _
            //   vz  _ _ _ | _ _ _

            Eigen::Matrix<10,6,wolf::Scalar> D_nd = Eigen::Matrix<10,6,wolf::Scalar>::Zero();
//            Eigen::MatrixXs DQ_wn(DQ_wdt * (-1)); //WDT_wn == -1
//            Eigen::MatrixXs WDT_wn(DV_adt*(-1));
//            Eigen::MatrixXs DV_wn(DV_dq*DQ_wn);
//            Eigen::MatrixXs DP_an(DP_adt*(-1));
//            Eigen::MatrixXs DP_wn(DP_dv*DV_wn);
            D_nd.block<3,3>(4,3) = DQ_wdt * (-1);
            D_nd.block<7,0>(3,3) = DV_adt*(-1);
            D_nd.block<7,3>(3,3) = DV_dq*DQ_wn;
            D_nd.block<0,0>(3,3) = DP_adt*(-1);
            D_nd.block<0,3>(3,3) = DP_dv*DV_wn;

            /// Jacobian wrt bias
            Eigen::Matrix<10,6,wolf::Scalar> D_b = Eigen::Matrix<10,6,wolf::Scalar>::Zero();
            D_b = D_nd * dt;

        }

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _dt, Eigen::VectorXs& _x_plus_delta)
        {
            // TODO: all the work to be done here

        }

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         * (e.g [0:2]-> position, [3:6]->orientation quaternion, [7:9]->velocity, [10:12]-> Acceleration bias, [13:15]->Gyroscope Bias)
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
        {
            // TODO: all the work to be done here
            //Quaternion integration
            //[dqi_out, DQI_OUT_dqi, DQI_OUT_dq] = qProd(dqi,dq);


        }

        /** \brief composes a delta-state on top of another delta-state and computes jacobians
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         * (e.g [0:2]-> position, [3:6]->orientation quaternion, [7:9]->velocity, [10:12]-> Acceleration bias, [13:15]->Gyroscope Bias)
         * \param _jacobian1 : jacobian wrt di (_delta1)
         * \param _jacobian2 : jacobian wrt d (_delta2)
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */

        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _dt,
                                    Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2)
        {
            // di: integrated delta (_delta2)
            // d : instantaneous delta (_delta1)
            //Quaternion integration
            //[dqi_out, DQI_OUT_dqi, DQI_OUT_dq] = qProd(dqi,dq);
            Eigen::Matrix4s DQI_OUT_dqi, DQI_OUT_dq;// 4x4 matrix
            //TODO : DEFINE qProd
            _delta1_plus_delta2.segment(3,4) = qProd(_delta2.segment(3,4), _delta1.segment(3,4), DQI_OUT_dqi, DQI_OUT_dq); //left hand operation

            //Velocity Integration
            //[dv_tmp, DVT_dv, DVT_dqi_out] =  qRot(dv, dqi_out);
            Eigen::Matrix3s DVT_dv; //3x3 matrix
            Eigen::Matrix<3,4,wolf::Scalar> DVT_dqi_out; //3x4 matrix
            Eigen::Vector3s dv_tmp;
            dv_tmp = qRot(_delta1.segment(7,3), _delta1_plus_delta2.segment(3,4), DVT_dv, DVT_dqi_out);
            _delta1_plus_delta2.segment(7,3) = _dv_tmp + _delta2.segment(7,3);
//            DVI_OUT_dvi = 1;
//            DVI_OUT_dvt = 1;
//            DVI_OUT_dqi = DVI_OUT_dvt * DVT_dqi_out * DQI_OUT_dqi;
//            DVI_OUT_dv  = DVI_OUT_dvt * DVT_dv;
            Eigen::Matrix<3,4,wolf::Scalar> DVI_OUT_dqi(DVT_dqi_out*DQI_OUT_dqi); // 3x4 matrix
            Eigen::Matrix3s DVI_OUT_dv(DVT_dv); // 3x3 mstrix

            //Position integration
            _delta1_plus_delta2.head<3>() = _delta2.head<3>() +  1.5*dv_tmp*dt;
            //DPI_OUT_dpi = 1;
            wolf::Scalar DPI_OUT_dvt = 1.5 * dt;
            //DPI_OUT_dqi = DPI_OUT_dvt * DVT_dqi_out * DQI_OUT_dqi;
            Eigen::Matrix<3,4,wolf::Scalar> DPI_OUT_dqi(DPI_OUT_dvt*DVT_dqi_out*DQI_OUT_dqi); //3x4 matrix
            //DPI_OUT_dv  = DPI_OUT_dvt * DVT_dv;
            Eigen::Matrix3s DPI_OUT_dv(DPI_OUT_dvt*DVT_dv); //3x3 matrix

            //BIAS
            _delta1_plus_delta2.tail<6>() = _delta2.tail<6>(); //constant bias model ?

            // Jacobian wrt di --> _jacobian1
            Eigen::Matrix<10,10,wolf::Scalar> DI_OUT_di = Eigen::Matrix<10,10,wolf::Scalar>::Zero();
            DI_OUT_di.block<3,3>(4,4) = DQI_OUT_dqi;
            DI_OUT_di.block<7,7>(3,3) = DVI_OUT_dvi;
            DI_OUT_di.block<0,3>(3,4) = DPI_OUT_dqi;
            DI_OUT_di.block<0,0>(3,3) = DPI_OUT_dpi;

            // Jacobian wrt d --> _jacobian2
            /*DI_OUT_d(qr,qr) = DQI_OUT_dq;
            DI_OUT_d(vr,qr) = DVI_OUT_dvt * DVT_dqi_out * DQI_OUT_dq;
            DI_OUT_d(vr,vr) = DVI_OUT_dv;
            DI_OUT_d(pr,qr) = DPI_OUT_dvt * DVT_dqi_out * DQI_OUT_dq;
            DI_OUT_d(pr,vr) = DPI_OUT_dv;*/

            Eigen::Matrix<10,10,wolf::Scalar> DI_OUT_d = Eigen::Matrix<10,10,wolf::Scalar>::Zero();
            DI_OUT_d.block<3,3>(4,4) = DQI_OUT_dq;
            DI_OUT_d.block<7,3>(3,4) = DVT_dqi_out * DQI_OUT_dq;
            DI_OUT_d.block<7,7>(3,3) = DVI_OUT_dv;
            DI_OUT_d.block<0,3>(3,4) = DPI_OUT_dvt * DVT_dqi_out * DQI_OUT_dq;
            DI_OUT_d.block<0,7>(3,3) = DPI_OUT_dv;

        }

        /** \brief Computes the delta-state the goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1) { };

        virtual Eigen::VectorXs deltaZero() const
        {
            Eigen::VectorXs tmp(10);
            tmp <<  0,0,0,  0,0,0,1,  0,0,0;  // p, q, v
            return tmp;
        }

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
        {
            Motion tmp(_motion_ref);
            tmp.ts_ = _ts;
            tmp.delta_ = deltaZero();
            tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
            return tmp;
        }

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
        {
            // TODO: all the work to be done here
            return nullptr;
        }


    protected:
//        SensorIMU* sensor_imu_ptr_; //will contain IMU parameters
//        CaptureIMU* capture_imu_ptr_; //specific pointer to capture imu data object

    private:
        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

} // namespace wolf

#endif // PROCESSOR_IMU_H
