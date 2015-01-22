//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
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
	CORR_2D_ODOMETRY_THETA,
	CORR_2D_RANGE};
enum parametrizationType {
	NONE,
	COMPLEX_ANGLE,
	QUATERNION,
	PO_2D};

class StateBase
{
	protected:
		WolfScalar* state_ptr_;

	public:

		StateBase(VectorXs& _st_remote, const unsigned int _idx) :
			state_ptr_(_st_remote.data() + _idx)
		{
		}

		StateBase(WolfScalar* _st_ptr) :
			state_ptr_(_st_ptr)
		{
		}

		StateBase(StateBase& _st) :
			state_ptr_(_st.state_ptr_)
		{
		}


		virtual ~StateBase()
		{
		}

		virtual WolfScalar* getPtr()
		{
			return state_ptr_;
		}

		virtual parametrizationType getParametrizationType() const = 0;

		virtual void print() const = 0;
};

class StatePoint2D: public StateBase
{
	public:
		static const unsigned int BLOCK_SIZE = 2;

		StatePoint2D(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx)
		{
		}

		StatePoint2D(WolfScalar* _st_ptr) :
			StateBase(_st_ptr)
		{
		}

		virtual ~StatePoint2D()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return NONE;
		}

		virtual void print() const
		{
			std::cout << *this->state_ptr_ << " " << *(this->state_ptr_+1) << std::endl;
		}
};

class StateThetaAngle: public StateBase
{
	public:
		static const unsigned int BLOCK_SIZE = 1;

		StateThetaAngle(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx)
		{
		}

		StateThetaAngle(WolfScalar* _st_ptr) :
			StateBase(_st_ptr)
		{
		}

		virtual ~StateThetaAngle()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return NONE;
		}

		virtual void print() const
		{
			std::cout << *this->state_ptr_ << std::endl;
		}
};

class StateComplexAngle: public StateBase
{
	public:
		static const unsigned int BLOCK_SIZE = 2;

		StateComplexAngle(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx)
		{
		}

		StateComplexAngle(WolfScalar* _st_ptr) :
			StateBase(_st_ptr)
		{
		}

		virtual ~StateComplexAngle()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return COMPLEX_ANGLE;
		}

		virtual void print() const
		{
			std::cout << *this->state_ptr_ << " " << *(this->state_ptr_+1) << std::endl;
		}
};

class ComplexAngleParameterization : public ceres::LocalParameterization
{
	public:
		virtual ~ComplexAngleParameterization()
		{
		}

		virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
		{
			x_plus_delta_raw[0] = x_raw[0] * cos(delta_raw[0]) - x_raw[1] * sin(delta_raw[0]);
			x_plus_delta_raw[1] = x_raw[1] * cos(delta_raw[0]) + x_raw[0] * sin(delta_raw[0]);

			//normalize
			//double norm = sqrt(x_plus_delta_raw[0] * x_plus_delta_raw[0] + x_plus_delta_raw[1] * x_plus_delta_raw[1]);
			//std::cout << "(before normalization) norm = " << norm << std::endl;
			//x_plus_delta_raw[0] /= norm;
			//x_plus_delta_raw[1] /= norm;

			return true;
		}

		virtual bool ComputeJacobian(const double* x, double* jacobian) const
		{
			jacobian[0] = -x[1];
			jacobian[1] =  x[0];
			return true;
		}

		virtual int GlobalSize() const
		{
			return 2;
		}

		virtual int LocalSize() const
		{
			return 1;
		}
};

class CorrespondenceBase
{
	protected:
		WolfScalar *measurement_ptr_;

    public:

        CorrespondenceBase(WolfScalar * _measurement_ptr) :
        	measurement_ptr_(_measurement_ptr)
        {
        }

        virtual ~CorrespondenceBase()
        {
        }

        virtual correspondenceType getType() const = 0;
        virtual const std::vector<WolfScalar *> getBlockPtrVector() = 0;
};

template <const unsigned int MEASUREMENT_SIZE,
				unsigned int BLOCK_0_SIZE,
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

		CorrespondenceSparse(WolfScalar* _measurementPtr, WolfScalar** _blockPtrArray) :
        	CorrespondenceBase(_measurementPtr),
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

		CorrespondenceSparse(WolfScalar* _measurementPtr,
							 WolfScalar* _state0Ptr,
							 WolfScalar* _state1Ptr = NULL,
							 WolfScalar* _state2Ptr = NULL,
							 WolfScalar* _state3Ptr = NULL,
							 WolfScalar* _state4Ptr = NULL,
							 WolfScalar* _state5Ptr = NULL,
							 WolfScalar* _state6Ptr = NULL,
							 WolfScalar* _state7Ptr = NULL,
							 WolfScalar* _state8Ptr = NULL,
							 WolfScalar* _state9Ptr = NULL ) :
			CorrespondenceBase(_measurementPtr),
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
					//std::cout << "state " << i << ":" << std::endl;
					//for (uint j = 0; j<block_sizes_vector_.at(i); j++ )
					//	std::cout << *(state_block_ptr_vector_.at(i)+j) << ", ";
					//std::cout << std::endl;
				}
			}

			//TODO: Check if while size OK, pointers NULL
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

class CorrespondenceGPS2D : public CorrespondenceSparse<2,2>
{
	public:
		static const unsigned int N_BLOCKS = 1;
		const double stdev_ = 1;

		CorrespondenceGPS2D(WolfScalar* _measurementPtr, WolfScalar* _statePtr) :
			CorrespondenceSparse<2,2>(_measurementPtr, _statePtr)
		{
		}

		CorrespondenceGPS2D(WolfScalar* _measurementPtr, StateBase* _statePtr) :
			CorrespondenceSparse<2,2>(_measurementPtr, _statePtr->getPtr())
		{
		}

		virtual ~CorrespondenceGPS2D()
		{
		}

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const
		{
			_residuals[0] = (T(*this->measurement_ptr_) - _x[0]) / T(stdev_);
			_residuals[1] = (T(*(this->measurement_ptr_+1)) - _x[1]) / T(stdev_);

			return true;
		}

		virtual correspondenceType getType() const
		{
			return CORR_GPS_2D;
		}
};

//class Correspondence2DRange : public CorrespondenceSparse<1,2,2>
//{
//	public:
//		static const unsigned int N_BLOCKS = 2;
//
//		Correspondence2DRange(WolfScalar** _blockPtrs) :
//			CorrespondenceSparse(_blockPtrs)
//		{
//		}
//
//		Correspondence2DRange(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
//			CorrespondenceSparse(_block1Ptr, _block2Ptr)
//		{
//		}
//
////		Correspondence2DRange(StateBase* _state1Ptr, StateBase* _state2Ptr) :
////			CorrespondenceSparse(_state1Ptr, _state2Ptr)
////		{
////		}
//
//		~Correspondence2DRange()
//		{
//		}
//
//        template <typename T>
//        bool operator()(const T* const _x1, const T* const _x2, T* _residuals) const
//        {
//        	// print inputs
//        	// std::cout << "_x1 = ";
//        	// for (int i=0; i < this->block1Size; i++)
//        	// 	std::cout << _x1[i] << " ";
//        	// std::cout << std::endl;
//        	// std::cout << "_x2 = ";
//        	// for (int i=0; i < this->block2Size; i++)
//        	// 	std::cout << _x2[i] << " ";
//        	// std::cout << std::endl;
//        	// std::cout << "measurement = ";
//        	// for (int i=0; i < this->measurementSize; i++)
//        	// 	std::cout << this->measurement_(i) << " ";
//        	// std::cout << std::endl;
//
//        	// Remap the vehicle state to the const evaluation point
//			Map<const Matrix<T,Dynamic,1>> x1_map_const(_x1, this->block1Size);
//			Map<const Matrix<T,Dynamic,1>> x2_map_const(_x2, this->block2Size);
//
//			// Map residuals vector to matrix (with sizes of the measurements matrix)
//			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);
//
//			// Compute error or residuals
//			Matrix<T,Dynamic,1> expected_measurement = ((x1_map_const - x2_map_const).transpose() * (x1_map_const - x2_map_const)).cwiseSqrt();
//			VectorXd meas = this->measurement_;
//			mapped_residuals = (meas).cast<T>() - expected_measurement;
//
//			// print outputs
//			// std::cout << "expected    = ";
//			// for (int i=0; i < this->measurementSize; i++)
//			// 	std::cout << expected_measurement(i) << " ";
//			// std::cout << std::endl;
//			// std::cout << "_residuals  = ";
//			// for (int i=0; i < this->measurementSize; i++)
//			// 	std::cout << _residuals[i] << " ";
//			// std::cout << std::endl << std::endl;
//
//			return true;
//        }
//
//        virtual correspondenceType getType() const
//        {
//        	return CORR_2D_RANGE;
//        }
//};

class Correspondence2DOdometry : public CorrespondenceSparse<2,2,2,2,2>
{
	public:
		static const unsigned int N_BLOCKS = 4;
		const double stdev_ = 0.01; //model parameters

		Correspondence2DOdometry(WolfScalar* _measurementPtr, WolfScalar** _blockPtrs) :
			CorrespondenceSparse<2,2,2,2,2>(_measurementPtr, _blockPtrs)
		{
		}

		Correspondence2DOdometry(WolfScalar* _measurementPtr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr, WolfScalar* _block4Ptr) :
			CorrespondenceSparse<2,2,2,2,2>(_measurementPtr, _block1Ptr, _block2Ptr, _block3Ptr, _block4Ptr)
		{
		}

		Correspondence2DOdometry(WolfScalar* _measurementPtr, StateBase* _state1Ptr, StateBase* _state2Ptr, StateBase* _state3Ptr, StateBase* _state4Ptr) :
			CorrespondenceSparse<2,2,2,2,2>(_measurementPtr, _state1Ptr->getPtr(), _state2Ptr->getPtr(),_state3Ptr->getPtr(), _state4Ptr->getPtr())
		{
		}

		virtual ~Correspondence2DOdometry()
		{
		}

        template <typename T>
        bool operator()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2, T* _residuals) const
        {
        	// print inputs
        	// std::cout << "_p1:" << std::endl;
        	//for (int i=0; i < this->block0Size; i++)
        	//	std::cout << _p1[i] << std::endl;
        	// std::cout << std::endl;
        	// std::cout << "_o1:" << std::endl;
        	//for (int i=0; i < this->block1Size; i++)
        	//	 std::cout << _o1[i] << std::endl;
        	//std::cout << std::endl;
        	// std::cout << "_p2:" << std::endl;
        	// for (int i=0; i < this->block2Size; i++)
        	// 	std::cout << _p2[i] << std::endl;
        	// std::cout << std::endl;
        	//std::cout << "_o2:" << std::endl;
        	//for (int i=0; i < this->block3Size; i++)
        	//	 std::cout << _o2[i] << std::endl;
        	//std::cout << std::endl;
        	// std::cout << "measurement:" << std::endl;
        	//for (int i=0; i < this->measurementSize; i++)
        	//	std::cout << *(this->measurement_ptr_+i) << std::endl;
        	//std::cout << std::endl;

			// Expected measurement
			T expected_range = (_p1[0]-_p2[0])*(_p1[0]-_p2[0]) + (_p1[1]-_p2[1])*(_p1[1]-_p2[1]); //square of the range
			T expected_rotation = atan2(-_o1[0]*_o2[1] + _o1[1]*_o2[0], _o1[0]*_o2[0] + _o1[1]*_o2[1]);

			// Residuals
			_residuals[0] = (expected_range - T((*this->measurement_ptr_)*(*this->measurement_ptr_))) / T(stdev_);
			_residuals[1] = (expected_rotation - T(*this->measurement_ptr_+1)) / T(stdev_);

			return true;
        }

        virtual correspondenceType getType() const
        {
        	return CORR_2D_ODOMETRY;
        }
};

class Correspondence2DOdometryTheta : public CorrespondenceSparse<2,2,1,2,1>
{
	public:
		static const unsigned int N_BLOCKS = 4;
		const double stdev_ = 0.01; //model parameters

		Correspondence2DOdometryTheta(WolfScalar* _measurementPtr, WolfScalar** _blockPtrs) :
			CorrespondenceSparse<2,2,1,2,1>(_measurementPtr, _blockPtrs)
		{
		}

		Correspondence2DOdometryTheta(WolfScalar* _measurementPtr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr, WolfScalar* _block4Ptr) :
			CorrespondenceSparse<2,2,1,2,1>(_measurementPtr, _block1Ptr, _block2Ptr, _block3Ptr, _block4Ptr)
		{
		}

		Correspondence2DOdometryTheta(WolfScalar* _measurementPtr, StateBase* _state1Ptr, StateBase* _state2Ptr, StateBase* _state3Ptr, StateBase* _state4Ptr) :
			CorrespondenceSparse<2,2,1,2,1>(_measurementPtr, _state1Ptr->getPtr(), _state2Ptr->getPtr(),_state3Ptr->getPtr(), _state4Ptr->getPtr())
		{
		}

		virtual ~Correspondence2DOdometryTheta()
		{
		}

        template <typename T>
        bool operator()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2, T* _residuals) const
        {
			// Expected measurement
			T expected_range = (_p1[0]-_p2[0])*(_p1[0]-_p2[0]) + (_p1[1]-_p2[1])*(_p1[1]-_p2[1]); //square of the range
			T expected_rotation = _o2[0]-_o1[0];

			// Residuals
			_residuals[0] = (expected_range - T((*this->measurement_ptr_)*(*this->measurement_ptr_))) / T(stdev_);
			_residuals[1] = (expected_rotation - T(*this->measurement_ptr_+1)) / T(stdev_);

			return true;
        }

        virtual correspondenceType getType() const
        {
        	return CORR_2D_ODOMETRY_THETA;
        }
};

class WolfProblem
{
    protected:
        std::vector<StateBase*> state_units_;
        std::list<CorrespondenceBase*> correspondences_;

    public: 
        WolfProblem(WolfScalar* _st_ptr, unsigned int _size) :
			//tate_units_(0),
			correspondences_(0)
        {
        }

        WolfProblem(VectorXs& _st) :
			//state_units_(0),
			correspondences_(0)
        {
        }
        
        WolfProblem() :
			//state_units_(0),
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

        void addStateUnit(StateBase* _stPtr)
        {
        	//std::cout << "Adding state unit to the wolf list..." << std::endl;
        	state_units_.push_back(_stPtr);
        	//std::cout << "Added!" << std::endl;
        }

//        void removeStateUnit(StateBase* _stPtr)
//        {
//        	state_units_.remove(_stPtr);
//        }
};

class CeresWrapper
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
			StateBase* st_ptr_;
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

		~CeresWrapper()
		{
		}

		ceres::Solver::Summary solve()
		{
			// add Residual Blocks
			addResidualBlocks();

			// apply Local Parametrizations
			applyLocalParametrizations();

			// run Ceres Solver
			ceres::Solve(ceres_options_, &ceres_problem_, &ceres_summary_);

			//display results
			return ceres_summary_;
		}

		template <class CorrespondenceType>
		void addCorrespondence(CorrespondenceType* _corr_ptr)
		{
			wolf_problem_->addCorrespondence(_corr_ptr);
			correspondence_list_.push_back(CorrespondenceWrapper{_corr_ptr, createCostFunction<CorrespondenceType>(_corr_ptr), _corr_ptr->getBlockPtrVector()});
		}

		template <class StateType>
		void addStateUnit(StateType* _st_ptr)
		{
			//std::cout << "Adding a State Unit to wolf_problem... " << std::endl;
			//_st_ptr->print();

			switch (_st_ptr->getParametrizationType())
			{
				case COMPLEX_ANGLE:
				{
					//std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
					local_parametrization_list_.push_back(LocalParametrizationWrapper{_st_ptr, new ComplexAngleParameterization});
					break;
				}
//				case QUATERNION:
//				{
//					std::cout << "Adding Quaternion Local Parametrization to the List... " << std::endl;
//					local_parametrization_list_.push_back(LocalParametrizationWrapper{_st_ptr, new EigenQuaternionParameterization});
//					break;
//				}
				case NONE:
				{
					//std::cout << "No Local Parametrization to be added" << std::endl;
					break;
				}
				default:
					std::cout << "Unknown  Local Parametrization type!" << std::endl;
			}
			//std::cout << "Local Parametrization List... " << std::endl;
			//for (std::list<LocalParametrizationWrapper>::iterator it=local_parametrization_list_.begin(); it!=local_parametrization_list_.end(); ++it)
			//	it->st_ptr_->print();

		}

		void addResidualBlocks()
		{
			for (std::list<CorrespondenceWrapper>::iterator it=correspondence_list_.begin(); it!=correspondence_list_.end(); ++it)
				ceres_problem_.AddResidualBlock(it->cost_function_ptr_, NULL, it->block_ptrs_);
		}

		template <typename CorrespondenceDerived>
		ceres::CostFunction* createCostFunction(CorrespondenceDerived* _corrPtr)
		{
			// TODO: get_diff_type --> swich case (autodiff, numerical, etc)
			return new ceres::AutoDiffCostFunction<CorrespondenceDerived,
													_corrPtr->measurementSize,
													_corrPtr->block0Size,
													_corrPtr->block1Size,
													_corrPtr->block2Size,
													_corrPtr->block3Size,
													_corrPtr->block4Size,
													_corrPtr->block5Size,
													_corrPtr->block6Size,
													_corrPtr->block7Size,
													_corrPtr->block8Size,
													_corrPtr->block9Size>(_corrPtr);
		}

		void applyLocalParametrizations()
		{
			for (std::list<LocalParametrizationWrapper>::iterator it=local_parametrization_list_.begin(); it!=local_parametrization_list_.end(); ++it)
				ceres_problem_.SetParameterization(it->st_ptr_->getPtr(), it->local_parametrization_ptr_);
		}
};

int main(int argc, char** argv) 
{
    std::cout << " ========= 2D Robot with odometry and GPS ===========" << std::endl << std::endl;
    
    //user input
	if (argc!=4)
	{
		std::cout << "Please call me with: [./test_ceres_wrapper_states NI PRINT ORIENTATION_MODE], where:" << std::endl;
		std::cout << "     - NI is the number of iterations" << std::endl;
		std::cout << "     - PRINT = 1 for print results" << std::endl;
		std::cout << "     - ORIENTATION_MODE: 0 for theta, 1 for complex angle" << std::endl;
		std::cout << "EXIT due to bad user input" << std::endl << std::endl;
		return -1;
	}

	unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution
	bool print = (bool) atoi(argv[2]);
	bool complex_angle = (bool) atoi(argv[3]);

	//init google log
	google::InitGoogleLogging(argv[0]);

	//variables
	int dim = (complex_angle ? 4 : 3);
	Eigen::VectorXs odom_inc_true(n_execution*2);//invented motion
	Eigen::VectorXs pose_true(dim); //current true pose
	Eigen::VectorXs ground_truth(n_execution*dim); //all true poses
	Eigen::VectorXs pose_predicted(dim); // current predicted pose
	Eigen::VectorXs state(n_execution*dim); //running window winth solver result
	Eigen::VectorXs odom_readings(n_execution*2); // all odometry readings
	Eigen::VectorXs gps_fix_readings(n_execution*3); //all GPS fix readings
	std::vector<StateBase> state_vector_; // vector of state units
	std::vector<StateBase*> state_ptr_vector_; // vector of pointers to state units

	//init true odom and true pose
	for (unsigned int ii = 0; ii<n_execution; ii++)
	{
		if ( ii < (unsigned int)floor(n_execution/2) )
			odom_inc_true.middleRows(ii*2,2) << fabs(cos(ii/10.)) , fabs(sin(ii/2000.)); //invented motion increments.
		else
			odom_inc_true.middleRows(ii*2,2) << fabs(cos(ii/10.)) , -fabs(sin((ii-floor(n_execution/2))/2000.)); //invented motion increments.
	}
	if (complex_angle)
	{
		pose_true << 0,0,1,0;
		pose_predicted << 0,0,1,0;
	}
	else
	{
		pose_true << 0,0,0;
		pose_predicted << 0,0,0;
	}
	ground_truth.head(dim) << pose_true; //init point pushed to ground truth
	state.head(dim) << pose_predicted; //init state at origin

	//init random generators
	std::default_random_engine generator(1);
	std::normal_distribution<WolfScalar> distribution_odom(0.001,0.01); //odometry noise
	std::normal_distribution<WolfScalar> distribution_gps(0.0,1); //GPS noise

    // TODO: incorporar weights a les funcions residu (via LossFunction o directament a operador())

    // Ceres problem initialization
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_progress_to_stdout = false;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    ceres_options.line_search_direction_type = ceres::LBFGS;
    ceres_options.max_num_iterations = 100;
    CeresWrapper ceres_wrapper(ceres_options);
    std::ofstream log_file;  //output file

	// Start trajectory
	ceres_wrapper.addStateUnit<StatePoint2D>(new StatePoint2D(state.data()));
	state_ptr_vector_.push_back(new StatePoint2D(state.data()));
	if (complex_angle)
	{
		ceres_wrapper.addStateUnit<StateComplexAngle>(new StateComplexAngle(state.data()+2));
		state_ptr_vector_.push_back(new StateComplexAngle(state.data() + 2));
	}
	else
	{
		ceres_wrapper.addStateUnit<StateThetaAngle>(new StateThetaAngle(state.data()+2));
		state_ptr_vector_.push_back(new StateThetaAngle(state.data() + 2));
	}

	for (uint step=1; step < n_execution; step++)
	{
		// NEW STATE
		//inventing a simple motion
		if (complex_angle)
		{
			double new_pose_true_2 = pose_true(2) * cos(odom_inc_true(step*2+1)) - pose_true(3) * sin(odom_inc_true(step*2+1));
			double new_pose_true_3 = pose_true(2) * sin(odom_inc_true(step*2+1)) + pose_true(3) * cos(odom_inc_true(step*2+1));
			pose_true(0) = pose_true(0) + odom_inc_true(step*2) * new_pose_true_2;
			pose_true(1) = pose_true(1) + odom_inc_true(step*2) * new_pose_true_3;
			pose_true(2) = new_pose_true_2;
			pose_true(3) = new_pose_true_3;
		}
		else
		{
			double new_pose_true_2 = pose_true(2) + (odom_inc_true(step*2+1));
			pose_true(0) = pose_true(0) + odom_inc_true(step*2) * cos(new_pose_true_2);
			pose_true(1) = pose_true(1) + odom_inc_true(step*2) * sin(new_pose_true_2);
			pose_true(2) = new_pose_true_2;
		}

		//inventing sensor readings for odometry and GPS
		odom_readings.segment(step*2,2) << odom_inc_true(step*2)+distribution_odom(generator), odom_inc_true(step*2+1)+distribution_odom(generator); //true range and theta with noise
		gps_fix_readings.segment(step*3,3) << pose_true(0) + distribution_gps(generator), pose_true(1) + distribution_gps(generator), 0. + distribution_gps(generator);

		//setting initial guess as an odometry prediction, using noisy odometry
		if (complex_angle)
		{
			double new_pose_predicted_2 = pose_predicted(2) * cos(odom_readings(step*2+1)) - pose_predicted(3) * sin(odom_readings(step*2+1));
			double new_pose_predicted_3 = pose_predicted(2) * sin(odom_readings(step*2+1)) + pose_predicted(3) * cos(odom_readings(step*2+1));
			pose_predicted(0) = pose_predicted(0) + odom_readings(step*2) * new_pose_predicted_2;
			pose_predicted(1) = pose_predicted(1) + odom_readings(step*2) * new_pose_predicted_3;
			pose_predicted(2) = new_pose_predicted_2;
			pose_predicted(3) = new_pose_predicted_3;
		}
		else
		{
			double new_pose_predicted_2 = pose_predicted(2) + (odom_readings(step*2+1));
			pose_predicted(0) = pose_predicted(0) + odom_readings(step*2) * cos(new_pose_predicted_2);
			pose_predicted(1) = pose_predicted(1) + odom_readings(step*2) * sin(new_pose_predicted_2);
			pose_predicted(2) = new_pose_predicted_2;
		}
		// //setting initial guess randomly
		//state.segment(4*step, 4).setRandom(); // initialize with random values
		//state.segment(4*step+2, 2) /= state.segment(4*step+2, 2).norm(); //normalize

		// store
		state.segment(step*dim,dim) << pose_predicted;
		ground_truth.segment(step*dim,dim) << pose_true;

		// STATE UNITS
		// p
		ceres_wrapper.addStateUnit<StatePoint2D>(new StatePoint2D(state.data() + step * dim));
		state_ptr_vector_.push_back(new StatePoint2D(state.data() + step * dim));
		// o
		if (complex_angle)
		{
			ceres_wrapper.addStateUnit<StateComplexAngle>(new StateComplexAngle(state.data() + step * dim + 2));
			state_ptr_vector_.push_back(new StateComplexAngle(state.data() + step * dim + 2));
		}
		else
		{
			ceres_wrapper.addStateUnit<StateThetaAngle>(new StateThetaAngle(state.data() + step * dim + 2));
			state_ptr_vector_.push_back(new StateThetaAngle(state.data() + step * dim + 2));
		}
		//std::cout << "New state ptr in the vector: " << std::endl;
		//for (int j = 0; j < state_ptr_vector_.size(); j++)
		//{
		//	std::cout << j << ":" <<  std::endl;
		//	state_ptr_vector_.at(j)->print();
		//}

		// CORRESPONDENCE ODOMETRY
		if (complex_angle)
		{
			Correspondence2DOdometry* corrOdomPtr = new Correspondence2DOdometry(odom_readings.data() + step*2,
																				 state_ptr_vector_.at(step*2-2),
																				 state_ptr_vector_.at(step*2-1),
																				 state_ptr_vector_.at(step*2),
																				 state_ptr_vector_.back());
			// Add correspondence
			ceres_wrapper.addCorrespondence<Correspondence2DOdometry>(corrOdomPtr);
		}
		else
		{
			Correspondence2DOdometryTheta* corrOdomPtr = new Correspondence2DOdometryTheta(odom_readings.data() + step*2,
																						   state_ptr_vector_.at(step*2-2),
																						   state_ptr_vector_.at(step*2-1),
																						   state_ptr_vector_.at(step*2),
																						   state_ptr_vector_.back());
			// Add correspondence
			ceres_wrapper.addCorrespondence<Correspondence2DOdometryTheta>(corrOdomPtr);
		}

		// CORRESPONDENCE GPS (2D)
		CorrespondenceGPS2D* corrGPSPtr = new CorrespondenceGPS2D(gps_fix_readings.data() + step*3, state_ptr_vector_.at(step*2));
		// Add correspondence
		ceres_wrapper.addCorrespondence<CorrespondenceGPS2D>(corrGPSPtr);

		// SOLVE CERES PROBLEM
		//ceres::Solver::Summary summary = ceres_wrapper.solve();
		if (print)
		{
			std::cout << " ========= STEP " << step << "===========" << std::endl << std::endl;
			std::cout << "odom : " << odom_inc_true.segment(step*2,2).transpose() << std::endl << std::endl;
			std::cout << "pose_predicted : " << pose_predicted.transpose() << std::endl << std::endl;
			std::cout << "pose_true : " << pose_true.transpose() << std::endl << std::endl;
			std::cout << "gps measurement : " << gps_fix_readings.segment(step*3,3).transpose() << std::endl << std::endl;
			std::cout << "state : " << std::endl << state.head(step*dim).transpose() << std::endl;
			std::cout << "ground_truth : " << std::endl << ground_truth.head(step*dim).transpose() << std::endl;
			std::cout << "error : " << std::endl << (ground_truth.head(step*dim) - state.head(step*dim)).transpose() << std::endl << std::endl;
			//std::cout << "total time (s):" << summary.total_time_in_seconds << std::endl;
		}
	}
    
	ceres::Solver::Summary summary = ceres_wrapper.solve();
	std::cout << summary.total_time_in_seconds << std::endl;

	//display/log results, by setting cout flags properly
	VectorXs state_theta(n_execution * 3);
	VectorXs ground_truth_theta(n_execution * 3);
	if (complex_angle)
	{
		// change from complex angle to theta
		for (uint ii = 0; ii<n_execution; ii++)
		{
			state_theta.segment(ii*3,3) << state(ii*4), state(ii*4+1), atan2(state(ii*4+2), state(ii*4+3));
			ground_truth_theta.segment(ii*3,3) << ground_truth(ii*4), ground_truth(ii*4+1), atan2(ground_truth(ii*4+2), ground_truth(ii*4+3));
		}
	}
	else
	{
		state_theta = state;
		ground_truth_theta = ground_truth;
	}
	std::string homepath = getenv("HOME");
	log_file.open(homepath + "/Desktop/log_file_2.txt", std::ofstream::out); //open log file
	if (log_file.is_open())
	{
		log_file << summary.total_time_in_seconds << std::endl;
		for (unsigned int ii = 0; ii<n_execution; ii++)
			log_file << state_theta.segment(ii*3,3).transpose()
					 << " " << ground_truth_theta.segment(ii*3,3).transpose()
					 << " " << (state_theta.segment(ii*3,3)-ground_truth_theta.segment(ii*3,3)).transpose()
					 << " " << gps_fix_readings.segment(ii*3,3).transpose() << std::endl;
		log_file.close(); //close log file
		std::cout << std::endl << " Result file ~/Desktop/log_data.txt" << std::endl;
	}
	else
		std::cout << std::endl << " Failed to write the file ~/Desktop/log_data.txt" << std::endl;
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

