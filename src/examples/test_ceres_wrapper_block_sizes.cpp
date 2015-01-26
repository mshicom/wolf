//std includes
#include <iostream>
#include <memory>
#include <random>
#include <typeinfo>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
//#include "ceres/loss_function.h"
#include "glog/logging.h"

//Wolf includes
#include "wolf.h"

/**
 * This class emulates a Wolf top tree node class, such as vehicle. 
 * This class will be removed when effective linking to Wolf, and using actual Vehicle class. 
 * It holds:
 *      - a map to a state vector
 *      - a map to an error vector
 *      - a method to compute the error from the state
 * 
 **/

using namespace Eigen;
enum correspondenceType {
	CORR_N_BLOCKS,
	CORR_GPS_3D,
	CORR_GPS_2D,
	CORR_3D_RANGE,
	CORR_2D_ODOMETRY,
	CORR_2D_RANGE};
enum parametrizationType {
	NONE          = 0,
	COMPLEX_ANGLE = 1,
	QUATERNION    = 2};

class StateUnitBase
{
	protected:
		WolfScalar* state_ptr_;

	public:

		StateUnitBase(VectorXs& _st_remote, const unsigned int _idx) :
			state_ptr_(_st_remote.data() + _idx)
		{
			std::cout << "StateUnitBase created!" << std::endl;
		}

		StateUnitBase(WolfScalar* _st_ptr) :
			state_ptr_(_st_ptr)
		{
			std::cout << "StateUnitBase created!" << std::endl;
		}

		StateUnitBase(StateUnitBase& _st) :
			state_ptr_(_st.state_ptr_)
		{
			std::cout << "StateUnitBase copied!" << std::endl;
		}


		virtual ~StateUnitBase()
		{
		}

		virtual WolfScalar* getPtr()
		{
			return state_ptr_;
		}

		virtual parametrizationType getParametrizationType() const = 0;
};

class StateUnitPoint2D: public StateUnitBase
{
	public:
	StateUnitPoint2D(VectorXs& _st_remote, const unsigned int _idx) :
			StateUnitBase(_st_remote, _idx)
		{
		}

		StateUnitPoint2D(WolfScalar* _st_ptr) :
			StateUnitBase(_st_ptr)
		{
		}

		virtual ~StateUnitPoint2D()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return NONE;
		}
};

class StateUnitPoint3D: public StateUnitBase
{
	public:
		StateUnitPoint3D(VectorXs& _st_remote, const unsigned int _idx) :
			StateUnitBase(_st_remote, _idx)
		{
		}

		StateUnitPoint3D(WolfScalar* _st_ptr) :
			StateUnitBase(_st_ptr)
		{
		}

		virtual ~StateUnitPoint3D()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return NONE;
		}
};

class CorrespondenceBase
{
	protected:
    	VectorXs measurement_;

    public:

        CorrespondenceBase(const unsigned int & _measurement_size) :
        	measurement_(_measurement_size)
        {
        }

        virtual ~CorrespondenceBase()
        {
        }

        virtual void inventMeasurement(const VectorXs& _measurement, std::default_random_engine& _generator, std::normal_distribution<WolfScalar>& _distribution)
		{
			measurement_ = _measurement;
			for(unsigned int ii=0; ii<measurement_.size(); ii++)
				measurement_(ii) += _distribution(_generator); //just inventing a sort of noise measurements
			//std::cout << "measurement_" << measurement_ << std::endl;
		}

        virtual correspondenceType getType() const = 0;
        virtual const std::vector<WolfScalar *> getBlockPtrVector() = 0;
};

template <const unsigned int MEASUREMENT_SIZE = 1,
				unsigned int BLOCK_0_SIZE = 1,
				unsigned int BLOCK_1_SIZE = 0,
				unsigned int BLOCK_2_SIZE = 0,
				unsigned int BLOCK_3_SIZE = 0,
				unsigned int BLOCK_4_SIZE = 0,
				unsigned int BLOCK_5_SIZE = 0,
				unsigned int BLOCK_6_SIZE = 0,
				unsigned int BLOCK_7_SIZE = 0,
				unsigned int BLOCK_8_SIZE = 0,
				unsigned int BLOCK_9_SIZE = 0>
class CorrespondenceSparse: public CorrespondenceBase
{
    protected:
		std::vector<Map<VectorXs>> state_block_map_vector_;
		std::vector<WolfScalar*> state_block_ptr_vector_;
		std::vector<unsigned int> block_sizes_vector_;

    public:
		static const unsigned int measurementSize = MEASUREMENT_SIZE;
		static const unsigned int block0Size = BLOCK_0_SIZE;
		static const unsigned int block1Size = BLOCK_1_SIZE;
		static const unsigned int block2Size = BLOCK_2_SIZE;
		static const unsigned int block3Size = BLOCK_3_SIZE;
		static const unsigned int block4Size = BLOCK_4_SIZE;
		static const unsigned int block5Size = BLOCK_5_SIZE;
		static const unsigned int block6Size = BLOCK_6_SIZE;
		static const unsigned int block7Size = BLOCK_7_SIZE;
		static const unsigned int block8Size = BLOCK_8_SIZE;
		static const unsigned int block9Size = BLOCK_9_SIZE;

		CorrespondenceSparse(WolfScalar** _blockPtrArray) :
        	CorrespondenceBase(MEASUREMENT_SIZE),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
        {
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(_blockPtrArray[i],block_sizes_vector_.at(i)));
					state_block_ptr_vector_.push_back(_blockPtrArray[i]);
				}
			}
        }

		CorrespondenceSparse(WolfScalar* _state0Ptr,
							 WolfScalar* _state1Ptr = NULL,
							 WolfScalar* _state2Ptr = NULL,
							 WolfScalar* _state3Ptr = NULL,
							 WolfScalar* _state4Ptr = NULL,
							 WolfScalar* _state5Ptr = NULL,
							 WolfScalar* _state6Ptr = NULL,
							 WolfScalar* _state7Ptr = NULL,
							 WolfScalar* _state8Ptr = NULL,
							 WolfScalar* _state9Ptr = NULL ) :
			CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_ptr_vector_({_state0Ptr,
									 _state1Ptr,
									 _state2Ptr,
									 _state3Ptr,
									 _state4Ptr,
									 _state5Ptr,
									 _state6Ptr,
									 _state7Ptr,
									 _state8Ptr,
									 _state9Ptr}),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
		{
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					state_block_ptr_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(state_block_ptr_vector_.at(i),block_sizes_vector_.at(i)));
				}
			}
		}

		CorrespondenceSparse(StateUnitBase* _state0Ptr,
							 StateUnitBase* _state1Ptr = NULL,
							 StateUnitBase* _state2Ptr = NULL,
							 StateUnitBase* _state3Ptr = NULL,
							 StateUnitBase* _state4Ptr = NULL,
							 StateUnitBase* _state5Ptr = NULL,
							 StateUnitBase* _state6Ptr = NULL,
							 StateUnitBase* _state7Ptr = NULL,
							 StateUnitBase* _state8Ptr = NULL,
							 StateUnitBase* _state9Ptr = NULL ) :
			CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_ptr_vector_({_state0Ptr->getPtr(),
			 	 	 	 	 	 	 _state1Ptr==NULL ? NULL : _state1Ptr->getPtr(),
									 _state2Ptr==NULL ? NULL : _state2Ptr->getPtr(),
									 _state3Ptr==NULL ? NULL : _state3Ptr->getPtr(),
									 _state4Ptr==NULL ? NULL : _state4Ptr->getPtr(),
									 _state5Ptr==NULL ? NULL : _state5Ptr->getPtr(),
									 _state6Ptr==NULL ? NULL : _state6Ptr->getPtr(),
									 _state7Ptr==NULL ? NULL : _state7Ptr->getPtr(),
									 _state8Ptr==NULL ? NULL : _state8Ptr->getPtr(),
									 _state9Ptr==NULL ? NULL : _state9Ptr->getPtr()}),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
		{
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					state_block_ptr_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(state_block_ptr_vector_.at(i),block_sizes_vector_.at(i)));
				}
			}
		}

        virtual ~CorrespondenceSparse()
        {
        }

        virtual correspondenceType getType() const
        {
        	return CORR_N_BLOCKS;
        }

		virtual const std::vector<WolfScalar *> getBlockPtrVector()
		{
			return state_block_ptr_vector_;
		}
};

class CorrespondenceGPS3D : public CorrespondenceSparse<3,3>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		CorrespondenceGPS3D(WolfScalar* _statePtrs) :
			CorrespondenceSparse(_statePtrs)
		{
		}

		CorrespondenceGPS3D(StateUnitBase* _statePtr) :
			CorrespondenceSparse(_statePtr)
		{
		}

		~CorrespondenceGPS3D()
		{
		}

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const
		{
			//std::cout << "adress of x: " << _x << std::endl;

			// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, this->block1Size);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);

			// Compute error or residuals
			mapped_residuals = measurement_.cast<T>() - state_map_const;

			return true;
		}

		virtual correspondenceType getType() const
		{
			return CORR_GPS_3D;
		}
};

class CorrespondenceGPS2D : public CorrespondenceSparse<2,2>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		CorrespondenceGPS2D(WolfScalar* _statePtrs) :
			CorrespondenceSparse(_statePtrs)
		{
		}

		CorrespondenceGPS2D(StateUnitBase* _statePtr) :
			CorrespondenceSparse(_statePtr)
		{
		}

		~CorrespondenceGPS2D()
		{
		}

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const
		{
			//std::cout << "adress of x: " << _x << std::endl;

			// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, this->block1Size);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);

			// Compute error or residuals
			mapped_residuals = measurement_.cast<T>() - state_map_const;

			return true;
		}

		virtual correspondenceType getType() const
		{
			return CORR_GPS_2D;
		}
};

class WolfProblem
{
    protected:
        std::list<StateUnitBase*> state_units_;
        std::list<CorrespondenceBase*> correspondences_;

    public: 
        WolfProblem(WolfScalar* _st_ptr, unsigned int _size) :
			state_units_(0),
			correspondences_(0)
        {
        }

        WolfProblem(VectorXs& _st) :
			state_units_(0),
			correspondences_(0)
        {
        }
        
        WolfProblem() :
			state_units_(0),
			correspondences_(0)
		{
		}

        virtual ~WolfProblem()
        {
        }

        std::list<CorrespondenceBase*> getCorrespondenceList()
        {
        	return correspondences_;
        }

        unsigned int getCorrespondencesSize()
        {
        	return correspondences_.size();
        }

        void addCorrespondence(CorrespondenceBase* _corrPtr)
        {
        	correspondences_.push_back(_corrPtr);
        }

        void removeCorrespondence(CorrespondenceBase* _corrPtr)
        {
        	correspondences_.remove(_corrPtr);
        }

        void addStateUnit(StateUnitBase* _stUnitPtr)
        {
        	std::cout << "Adding state unit to the wolf list..." << std::endl;
        	state_units_.push_back(_stUnitPtr);
        	std::cout << "Added!" << std::endl;
        }

        void removeStateUnit(StateUnitBase* _stUnitPtr)
        {
        	state_units_.remove(_stUnitPtr);
        }
};

class CeresManager
{
	protected:
		WolfProblem* wolf_problem_;
		struct CorrespondenceWrapper
		{
			CorrespondenceBase* corr_ptr_;
			ceres::CostFunction* cost_function_ptr_;
			const std::vector<WolfScalar*> block_ptrs_;
		};
		std::list<CorrespondenceWrapper> correspondence_list_;
		struct LocalParametrizationWrapper
		{
			StateUnitBase* st_ptr_;
			ceres::LocalParameterization* local_parametrization_ptr_;
		};
		std::list<LocalParametrizationWrapper> local_parametrization_list_;

		ceres::Problem ceres_problem_;
		ceres::Solver::Summary ceres_summary_;
		ceres::Solver::Options ceres_options_;

	public:
		CeresWrapper(const ceres::Solver::Options& _ceres_options) :
			ceres_options_(_ceres_options)
		{
			wolf_problem_= new WolfProblem;
		}

		CeresWrapper(const ceres::Solver::Options& _ceres_options, WolfProblem* _wolf_problem) :
			wolf_problem_(_wolf_problem),
			ceres_options_(_ceres_options)
		{
		}

		~CeresManager()
		{
		}

		void solve(const bool display_summary)
		{
			// add Residual Blocks
			addResidualBlocks();

			// apply Local Parametrizations
			applyLocalParametrizations();

			// run Ceres Solver
			ceres::Solve(ceres_options_, &ceres_problem_, &ceres_summary_);

			//display results
			if (display_summary)
				summary();
		}

		void summary()
		{
			std::cout << ceres_summary_.FullReport() << "\n";
		}

		void addCorrespondence(CorrespondenceBase* _corr_ptr)
		{
			std::cout << "Adding a Correspondence to wolf_problem..." << std::endl;
			wolf_problem_->addCorrespondence(_corr_ptr);
			std::cout << "Adding a Correspondence to the List..." << std::endl;
			ceres::CostFunction* cost_function_ptr;
			switch (_corr_ptr->getType())
			{
				case CORR_GPS_3D:
				{
					cost_function_ptr = createCostFunction<CorrespondenceGPS3D>(_corr_ptr);
					break;
				}
				case CORR_GPS_2D:
				{
					cost_function_ptr = createCostFunction<CorrespondenceGPS2D>(_corr_ptr);
					break;
				}
				case CORR_N_BLOCKS:
				{
					std::cout << "CORR_N_BLOCK" << std::endl;
					break;
				}
				default:
					std::cout << "Unknown correspondence type!" << std::endl;
			}
			correspondence_list_.push_back(CorrespondenceWrapper{_corr_ptr, cost_function_ptr, _corr_ptr->getBlockPtrVector()});
		}

		void addStateUnit(StateUnitBase* _st_ptr)
		{
			std::cout << "Adding a State Unit to wolf_problem... " << std::endl;
			//wolf_problem_->addStateUnit(_st_ptr);
			switch (_st_ptr->getParametrizationType())
			{
				case NONE:
				{
					std::cout << "No Local Parametrization to be added" << std::endl;
					break;
				}
				default:
					std::cout << "Unknown  Local Parametrization type!" << std::endl;
			}

		}

		void addResidualBlocks()
		{
			std::cout << "Adding residual blocks..." << std::endl;
			//int i = 0;
			for (std::list<CorrespondenceWrapper>::iterator it=correspondence_list_.begin(); it!=correspondence_list_.end(); ++it)
			{
				//std::cout << i++ << " block" << std::endl;
				ceres_problem_.AddResidualBlock(it->cost_function_ptr_, NULL, it->block_ptrs_);
			}
		}

		template <typename CorrespondenceDerived>
		ceres::CostFunction* createCostFunction(CorrespondenceBase* _corrBasePtr)
		{
			CorrespondenceDerived* corrCastedPtr = static_cast<CorrespondenceDerived*>(_corrBasePtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceDerived,
												   corrCastedPtr->measurementSize,
												   corrCastedPtr->block0Size,
												   corrCastedPtr->block1Size,
												   corrCastedPtr->block2Size,
												   corrCastedPtr->block3Size,
												   corrCastedPtr->block4Size,
												   corrCastedPtr->block5Size,
												   corrCastedPtr->block6Size,
												   corrCastedPtr->block7Size,
												   corrCastedPtr->block8Size,
												   corrCastedPtr->block9Size>(corrCastedPtr);
		}

		void applyLocalParametrizations()
		{
			std::cout << "Applying local parametrizations..." << std::endl;
			int i = 0;
			for (std::list<LocalParametrizationWrapper>::iterator it=local_parametrization_list_.begin(); it!=local_parametrization_list_.end(); ++it)
			{
				if (it->st_ptr_->getParametrizationType() != NONE)
				{
					std::cout << i << "th parametrization: " << it->st_ptr_->getParametrizationType() << std::endl;
					ceres_problem_.SetParameterization(it->st_ptr_->getPtr(), it->local_parametrization_ptr_);
				}
				i++;
			}
			std::cout << "Local parametrizations applied!" << std::endl;
		}
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= 2D Robot with odometry, GPS and Range only between poses ===========" << std::endl << std::endl;
    
    const unsigned int N_STATES = 20;
    // TODO: incorporar weights a les funcions residu (via LossFunction o directament a operador())
    //const double w_A = 1;
    //const double w_B = 10;

    // INITIALIZATION
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution_GPS(0.0,0.01);
    std::normal_distribution<WolfScalar> distribution_odometry(0.0,0.01);
    std::normal_distribution<WolfScalar> distribution_range_only(0.0,0.1);


    // Ceres problem initialization
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_progress_to_stdout = false;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    ceres_options.line_search_direction_type = ceres::LBFGS;
    ceres_options.max_num_iterations = 100;
    CeresManager ceres_wrapper(ceres_options);

    VectorXs state_(3);
    VectorXs actualState_(3);
    actualState_ << 1,2,3; // initial pose
	std::vector<WolfScalar*> frame_ptr_vector_;
	for (uint st=0; st < N_STATES; st++)
	{
		Vector2s actualDisplacement;
		WolfScalar actualRotation;
		std::cout << " ========= STEP " << st << "===========" << std::endl << std::endl;
		if (st>0)
		{
			// NEW STATE
			// Actual state
			Vector3s newActualState;
			newActualState = actualState_.tail(3) + Vector3s::Ones();
			std::cout << "newActualState : " << newActualState.transpose() << std::endl << std::endl;
		}
		// state resize
		if (st>0)
			state_.resize(state_.size()+4);
		state_.tail(3).setRandom(); // initialize with random values
		std::cout << "resized state_ : " << std::endl << state_.transpose() << std::endl << std::endl;

		// FRAME
		frame_ptr_vector_.push_back(state_.data() + state_.size() - 3);
		std::cout << "frame added, size = " << frame_ptr_vector_.size() << std::endl;

		// STATE UNITS
		// position
		StateUnitBase* pPrt = new StateUnitPoint3D(frame_ptr_vector_.back());
		ceres_wrapper.addStateUnit(pPrt);

		// CORRESPONDENCE GPS (2D)
		CorrespondenceGPS2D* corrGPSPtr = new CorrespondenceGPS2D(frame_ptr_vector_.back());
		// Invent measurement
		corrGPSPtr->inventMeasurement(actualState_.tail(3).head(2),generator,distribution_GPS);
		// Add correspondence
		ceres_wrapper.addCorrespondence(corrGPSPtr);

		// CORRESPONDENCE GPS (2D)
		CorrespondenceGPS3D* corrGPS3Ptr = new CorrespondenceGPS3D(frame_ptr_vector_.back());
		// Invent measurement
		corrGPS3Ptr->inventMeasurement(actualState_.tail(3),generator,distribution_GPS);
		// Add correspondence
		ceres_wrapper.addCorrespondence(corrGPS3Ptr);

		// SOLVE CERES PROBLEM
		if (st > 0)
		{
			ceres_wrapper.solve(true);
			std::cout << "state_ : " << std::endl << state_.transpose() << std::endl;
			std::cout << "actualState_ : " << std::endl << actualState_.transpose() << std::endl;
			std::cout << "error : " << std::endl << (actualState_ - state_).transpose() << std::endl << std::endl;
		}
	}

    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

