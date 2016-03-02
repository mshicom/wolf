#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

// Fwd refs
class HardwareBase;
class ProcessorBase;
class StateBlock;

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "state_block.h"

//std includes

class SensorBase : public NodeLinked<HardwareBase, ProcessorBase>
{
    protected:
        StateBlock* p_ptr_;		// sensor position state block pointer
        StateBlock* o_ptr_; 	// sensor orientation state block pointer
        StateBlock* intrinsic_ptr_; // intrinsic parameters
        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes.

        Eigen::VectorXs noise_std_; // std of sensor noise
        Eigen::MatrixXs noise_cov_; // cov matrix of noise

    public:        
        
        /** \brief Constructor with noise size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr TODO update documentation
         * \param _noise_size TODO update documentation
         * \param _extr_dyn TODO update documentation
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const unsigned int _noise_size, const bool _extr_dyn = false);

        /** \brief Constructor with noise std vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr TODO update documentation
         * \param _noise_std TODO update documentation
         * \param _extr_dyn TODO update documentation
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const Eigen::VectorXs & _noise_std, const bool _extr_dyn = false);


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorBase();

        void addProcessor(ProcessorBase* _proc_ptr);

        ProcessorBaseList* getProcessorListPtr();

        StateBlock* getPPtr() const;

        StateBlock* getOPtr() const;

        StateBlock* getIntrinsicPtr() const;

        void fix();

        void unfix();

        /**
         * Check if sensor is dynamic
         */
        bool isExtrinsicDynamic();

        void setNoise(const Eigen::VectorXs & _noise_std);

        Eigen::VectorXs getNoiseStd();

        Eigen::MatrixXs getNoiseCov();

};
#endif
