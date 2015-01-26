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
			std::cout << "StateUnitBase created!" << std::endl;
		}

		StateBase(WolfScalar* _st_ptr) :
			state_ptr_(_st_ptr)
		{
			std::cout << "StateBase created!" << std::endl;
		}

		StateBase(StateBase& _st) :
			state_ptr_(_st.state_ptr_)
		{
			std::cout << "StateBase copied!" << std::endl;
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

class StateComplexAngle: public StateBase
{
	public:
		static const unsigned int BLOCK_SIZE = 2;

		StateComplexAngle(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx)
		{
			std::cout << "StateComplexAngle created!" << std::endl;
		}

		StateComplexAngle(WolfScalar* _st_ptr) :
			StateBase(_st_ptr)
		{
			std::cout << "StateComplexAngle created!" << std::endl;
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

class StatePO2D: public StateBase
{
	public:
		static const unsigned int BLOCK_SIZE = 4;

		StatePO2D(WolfScalar* _st_ptr) :
			StateBase(_st_ptr)
		{
			std::cout << "statePO2D created!" << std::endl;
		}

		StatePO2D(StatePO2D& _st_ptr) :
			StateBase(_st_ptr)
		{
			std::cout << "statePO2D created!" << std::endl;
		}

		virtual ~StatePO2D()
		{
		}

		virtual parametrizationType getParametrizationType() const
		{
			return PO_2D;
		}

		virtual void print() const
		{
			std::cout << *this->state_ptr_ << " " << *(this->state_ptr_+1) << " " << *(this->state_ptr_+2) << " " << *(this->state_ptr_+3) << std::endl;
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
			double norm = sqrt(x_plus_delta_raw[0] * x_plus_delta_raw[0] + x_plus_delta_raw[1] * x_plus_delta_raw[1]);
			std::cout << "(before normalization) norm = " << norm << std::endl;
			x_plus_delta_raw[0] /= norm;
			x_plus_delta_raw[1] /= norm;

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

class StatePOParameterization : public ceres::LocalParameterization
{
	public:
		virtual ~StatePOParameterization()
		{
		}

		virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
		{
			x_plus_delta_raw[0] = x_raw[0];
			x_plus_delta_raw[1] = x_raw[1];
			x_plus_delta_raw[2] = x_raw[2] * cos(delta_raw[2]) - x_raw[3] * sin(delta_raw[2]);
			x_plus_delta_raw[3] = x_raw[3] * cos(delta_raw[2]) + x_raw[2] * sin(delta_raw[2]);

			//normalize
			double norm = sqrt(x_plus_delta_raw[0] * x_plus_delta_raw[0] + x_plus_delta_raw[1] * x_plus_delta_raw[1]);
			std::cout << "(before normalization) norm = " << norm << std::endl;
			x_plus_delta_raw[0] /= norm;
			x_plus_delta_raw[1] /= norm;

			return true;
		}

		virtual bool ComputeJacobian(const double* x, double* jacobian) const
		{
			jacobian[0] = x[0]; jacobian[1] = 0;    jacobian[2] = 0;
			jacobian[3] = 0;    jacobian[4] = x[1]; jacobian[5] = 0;
			jacobian[6] = 0;    jacobian[7] = 0;    jacobian[8] =-x[3];
			jacobian[9] = 0;    jacobian[10]= 0;    jacobian[11]= x[2];
			return true;
		}

		virtual int GlobalSize() const
		{
			return 4;
		}

		virtual int LocalSize() const
		{
			return 3;
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
					std::cout << "state " << i << ":" << std::endl;
					for (uint j = 0; j<block_sizes_vector_.at(i); j++ )
						std::cout << *(state_block_ptr_vector_.at(i)+j) << ", ";
					std::cout << std::endl;
				}
			}

			//TODO: Check if while size OK, pointers NULL
		}

//		CorrespondenceSparse(StateBase* _state0Ptr,
//							 StateBase* _state1Ptr = NULL,
//							 StateBase* _state2Ptr = NULL,
//							 StateBase* _state3Ptr = NULL,
//							 StateBase* _state4Ptr = NULL,
//							 StateBase* _state5Ptr = NULL,
//							 StateBase* _state6Ptr = NULL,
//							 StateBase* _state7Ptr = NULL,
//							 StateBase* _state8Ptr = NULL,
//							 StateBase* _state9Ptr = NULL ) :
//			CorrespondenceBase(MEASUREMENT_SIZE),
//			state_block_ptr_vector_({_state0Ptr->getPtr(),
//			 	 	 	 	 	 	 _state1Ptr==NULL ? NULL : _state1Ptr->getPtr(),
//									 _state2Ptr==NULL ? NULL : _state2Ptr->getPtr(),
//									 _state3Ptr==NULL ? NULL : _state3Ptr->getPtr(),
//									 _state4Ptr==NULL ? NULL : _state4Ptr->getPtr(),
//									 _state5Ptr==NULL ? NULL : _state5Ptr->getPtr(),
//									 _state6Ptr==NULL ? NULL : _state6Ptr->getPtr(),
//									 _state7Ptr==NULL ? NULL : _state7Ptr->getPtr(),
//									 _state8Ptr==NULL ? NULL : _state8Ptr->getPtr(),
//									 _state9Ptr==NULL ? NULL : _state9Ptr->getPtr()}),
//			block_sizes_vector_({BLOCK_0_SIZE,
//								 BLOCK_1_SIZE,
//								 BLOCK_2_SIZE,
//								 BLOCK_3_SIZE,
//								 BLOCK_4_SIZE,
//								 BLOCK_5_SIZE,
//								 BLOCK_6_SIZE,
//								 BLOCK_7_SIZE,
//								 BLOCK_8_SIZE,
//								 BLOCK_9_SIZE})
//		{
//			for (uint i = 0; i<block_sizes_vector_.size(); i++)
//			{
//				if (block_sizes_vector_.at(i) == 0)
//				{
//					block_sizes_vector_.resize(i);
//					state_block_ptr_vector_.resize(i);
//					break;
//				}
//				else
//				{
//					state_block_map_vector_.push_back(Map<VectorXs>(state_block_ptr_vector_.at(i),block_sizes_vector_.at(i)));
//				}
//			}
//		}

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

template <class stateType>
class CorrespondenceGPS2D : public CorrespondenceSparse<2,stateType::BLOCK_SIZE>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		CorrespondenceGPS2D(stateType* _statePtr) :
			CorrespondenceSparse<2,stateType::BLOCK_SIZE>(_statePtr->getPtr())
		{
		}

		virtual ~CorrespondenceGPS2D()
		{
		}

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const
		{
			//std::cout << "adress of x: " << _x << std::endl;

			// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, 2);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, 1);

			// Compute error or residuals
			mapped_residuals = this->measurement_.template cast<T>() - state_map_const;

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

template <class fromStateType, class toStateType>
class Correspondence2DOdometry : public CorrespondenceSparse<3,fromStateType::BLOCK_SIZE,toStateType::BLOCK_SIZE>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		Correspondence2DOdometry(WolfScalar** _blockPtrs) :
			CorrespondenceSparse<3,fromStateType::BLOCK_SIZE,toStateType::BLOCK_SIZE>(_blockPtrs)
		{
		}

		Correspondence2DOdometry(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			CorrespondenceSparse<3,fromStateType::BLOCK_SIZE,toStateType::BLOCK_SIZE>(_block1Ptr, _block2Ptr)
		{
		}

		Correspondence2DOdometry(fromStateType* _state1Ptr, toStateType* _state2Ptr) :
			CorrespondenceSparse<3,fromStateType::BLOCK_SIZE,toStateType::BLOCK_SIZE>(_state1Ptr->getPtr(), _state2Ptr->getPtr())
		{
		}

		Correspondence2DOdometry(StateBase* _state1Ptr, StateBase* _state2Ptr) :
			CorrespondenceSparse<3,fromStateType::BLOCK_SIZE,toStateType::BLOCK_SIZE>(_state1Ptr->getPtr(), _state2Ptr->getPtr())
		{
		}

		virtual ~Correspondence2DOdometry()
		{
		}

        template <typename T>
        bool operator()(const T* const _x1, const T* const _x2, T* _residuals) const
        {
        	// print inputs
        	 std::cout << "_x1:" << std::endl;
        	 for (int i=0; i < this->block0Size; i++)
        	 	std::cout << _x1[i] << std::endl;
        	 std::cout << std::endl;
        	 std::cout << "_x2:" << std::endl;
        	 for (int i=0; i < this->block1Size; i++)
        	 	std::cout << _x2[i] << std::endl;
        	 std::cout << std::endl;
        	 std::cout << "measurement:" << std::endl;
        	 for (int i=0; i < this->measurementSize; i++)
        	 	std::cout << this->measurement_(i) << std::endl;
        	 std::cout << std::endl;

        	// LONGITUDINAL AND TRANVERSAL ERROR
        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,2,1>> x1_map(_x1,2);
			Map<const Matrix<T,2,1>> x2_map(_x2,2);

			// Expected measurement
			Matrix<T,2,2> Rinv1; // Rotation matrix of the negative angle of the first pose
			Rinv1 << _x1[2], _x1[3], -_x1[3], _x1[2];
			Matrix<T,2,1> expected_meas = Rinv1 * (x1_map - x2_map);

			// Map residuals
			Map<Matrix<T,2,1>> x_residual(_residuals,2);

			// Compute residuals
			Vector2s x_meas = this->measurement_.head(2);
			x_residual = (x_meas).cast<T>() - expected_meas;

			// ORIENTATION ERROR
			_residuals[2] = T(this->measurement_(2)) - atan2(-_x1[2]*_x2[3] + _x1[3]*_x2[2],
															  _x1[2]*_x2[2] + _x1[3]*_x2[3]);

			// print outputs
			 std::cout << "expected:" << std::endl;
			 for (int i=0; i < 2; i++)
			 	std::cout << expected_meas(i) << std::endl;
			 std::cout << atan2(-_x1[2]*_x2[3] + _x1[3]*_x2[2], _x1[2]*_x2[2] + _x1[3]*_x2[3]);
			 std::cout << std::endl;
			 std::cout << "_residuals:" << std::endl;
			 for (int i=0; i < this->measurementSize; i++)
			 	std::cout << _residuals[i] << std::endl;
			 std::cout << std::endl << std::endl;

			return true;
        }

        virtual correspondenceType getType() const
        {
        	return CORR_2D_ODOMETRY;
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
        	std::cout << "Adding state unit to the wolf list..." << std::endl;
        	state_units_.push_back(_stPtr);
        	std::cout << "Added!" << std::endl;
        }

//        void removeStateUnit(StateBase* _stPtr)
//        {
//        	state_units_.remove(_stPtr);
//        }
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

		template <class CorrespondenceType>
		void addCorrespondence(CorrespondenceType* _corr_ptr)
		{
			std::cout << "Adding a Correspondence to wolf_problem..." << std::endl;
			wolf_problem_->addCorrespondence(_corr_ptr);
			std::cout << "Adding a Correspondence to the List..." << std::endl;
			ceres::CostFunction* cost_function_ptr = createCostFunction<CorrespondenceType>(_corr_ptr);

//			switch (_corr_ptr->getType())
//			{
//				case CORR_GPS_3D:
//				{
//					cost_function_ptr = createCostFunction<CorrespondenceGPS3D>(_corr_ptr);
//					break;
//				}
//				case CORR_GPS_2D:
//				{
//					cost_function_ptr = createCostFunction<CorrespondenceGPS2D>(_corr_ptr);
//					break;
//				}
//				case CORR_3D_RANGE:
//				{
//					cost_function_ptr = createCostFunction<Correspondence3DRange>(_corr_ptr);
//					break;
//				}
//				case CORR_2D_RANGE:
//				{
//					cost_function_ptr = createCostFunction<Correspondence2DRange>(_corr_ptr);
//					break;
//				}
//				case CORR_2D_ODOMETRY:
//				{
//					cost_function_ptr = createCostFunction<Correspondence2DOdometry>(_corr_ptr);
//					break;
//				}
//				case CORR_N_BLOCKS:
//				{
//					std::cout << "CORR_N_BLOCK" << std::endl;
//					break;
//				}
//				default:
//					std::cout << "Unknown correspondence type!" << std::endl;
//			}
			correspondence_list_.push_back(CorrespondenceWrapper{_corr_ptr, cost_function_ptr, _corr_ptr->getBlockPtrVector()});
		}

		template <class StateType>
		void addStateUnit(StateType* _st_ptr)
		{
			std::cout << "Adding a State Unit to wolf_problem... " << std::endl;
			_st_ptr->print();

			switch (_st_ptr->getParametrizationType())
			{
				case COMPLEX_ANGLE:
				{
					std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
					local_parametrization_list_.push_back(LocalParametrizationWrapper{_st_ptr, new ComplexAngleParameterization});
					break;
				}
				case PO_2D:
				{
					std::cout << "Adding State PO 2D Local Parametrization to the List... " << std::endl;
					local_parametrization_list_.push_back(LocalParametrizationWrapper{_st_ptr, new StatePOParameterization});

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
					std::cout << "No Local Parametrization to be added" << std::endl;
					break;
				}
				default:
					std::cout << "Unknown  Local Parametrization type!" << std::endl;
			}
			std::cout << "Local Parametrization List... " << std::endl;
			for (std::list<LocalParametrizationWrapper>::iterator it=local_parametrization_list_.begin(); it!=local_parametrization_list_.end(); ++it)
				it->st_ptr_->print();

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
		ceres::CostFunction* createCostFunction(CorrespondenceDerived* _corrBasePtr)
		{
			//CorrespondenceDerived* corrCastedPtr = static_cast<CorrespondenceDerived*>(_corrBasePtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceDerived,
													_corrBasePtr->measurementSize,
													_corrBasePtr->block0Size,
													_corrBasePtr->block1Size,
													_corrBasePtr->block2Size,
													_corrBasePtr->block3Size,
													_corrBasePtr->block4Size,
													_corrBasePtr->block5Size,
													_corrBasePtr->block6Size,
													_corrBasePtr->block7Size,
													_corrBasePtr->block8Size,
													_corrBasePtr->block9Size>(_corrBasePtr);
		}

		void applyLocalParametrizations()
		{
			std::cout << "Applying local parametrizations..." << std::endl;
			for (std::list<LocalParametrizationWrapper>::iterator it=local_parametrization_list_.begin(); it!=local_parametrization_list_.end(); ++it)
				ceres_problem_.SetParameterization(it->st_ptr_->getPtr(), it->local_parametrization_ptr_);

			std::cout << "Local parametrizations applied!" << std::endl;
		}
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= 2D Robot with odometry, GPS and Range only between poses ===========" << std::endl << std::endl;
    
    //user input
	if (argc!=2)
	{
		std::cout << "Please call me with: [./test_ceres_wrapper_states NI], where:" << std::endl;
		std::cout << "       - NI is the number of iterations" << std::endl;
		std::cout << "EXIT due to bad user input" << std::endl << std::endl;
		return -1;
	}
	unsigned int N_STATES = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

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

    VectorXs state_(4*N_STATES);
    state_.head(4) << 0, 0, 0, 1; // initial pose
    VectorXs actualState_(4);
    actualState_ << 0, 0, 0, 1; // initial pose
	//std::vector<WolfScalar*> frame_ptr_vector_;
	std::vector<StateBase*> state_ptr_vector_;
	for (uint st=0; st < N_STATES; st++)
	{
		Vector2s actualDisplacement;
		WolfScalar actualRotation;
		std::cout << " ========= STEP " << st << "===========" << std::endl << std::endl;


		if (st>0)
		{
			// NEW STATE
			// Actual odometry
			actualDisplacement << 1 + distribution_odometry(generator), 0.1 + distribution_odometry(generator);
			actualRotation = 0.1 * M_PI + distribution_odometry(generator);
			std::cout << "last state : " << state_.segment(4*st, 4).transpose() << std::endl << std::endl;
			std::cout << "lastActualState : " << actualState_.tail(4).transpose() << std::endl << std::endl;
			std::cout << "actualDisplacement : " << actualDisplacement.transpose() << std::endl << std::endl;
			std::cout << "actualRotation : " << actualRotation << std::endl << std::endl;
			// Actual state
			Vector4s newActualState;
			Vector4s lastActualState = actualState_.tail(4);
			Matrix2s R; // Rotation matrix of the angle of the first pose
			R << lastActualState(2), lastActualState(3), -lastActualState(3), lastActualState(2);
			newActualState.head(2) = lastActualState.head(2) + R * actualDisplacement;
			newActualState(2) = lastActualState(2) * cos(actualRotation) - lastActualState(3) * sin(actualRotation);
		    newActualState(3) = lastActualState(2) * sin(actualRotation) + lastActualState(3) * cos(actualRotation);
			actualState_.conservativeResize(actualState_.size()+4);
			actualState_.tail(4) = newActualState;
			actualState_.tail(2) /= actualState_.tail(2).norm(); //normalize
			std::cout << "newActualState : " << newActualState.transpose() << std::endl << std::endl;
		}
		// state resize
		if (st>0)
		{
			//state_.conservativeResize(state_.size()+4);
			//state_.tail(4).setRandom(); // initialize with random values
			//state_.tail(2) /= state_.tail(2).norm(); //normalize

			state_.segment(4*st, 4).setRandom(); // initialize with random values
			state_.segment(4*st+2, 2) /= state_.segment(4*st+2, 2).norm(); //normalize
		}
		std::cout << "resized state_ : " << std::endl << state_.transpose() << std::endl << std::endl;

		// FRAME
		//frame_ptr_vector_.push_back(state_.data() + state_.size() - 4);
		//std::cout << "frame added, size = " << frame_ptr_vector_.size() << std::endl;

		// STATE UNITS
		// PO
		std::cout << "state_ptr_vector_ BEFORE: " << std::endl;
		for (int j = 0; j < state_ptr_vector_.size(); j++)
		{
			std::cout << j << " th:" <<  std::endl;
			state_ptr_vector_.at(j)->print();
		}
		ceres_wrapper.addStateUnit<StatePO2D>(new StatePO2D(state_.data() + st * 4));
		std::cout << "state_ptr_vector_ AFTER: " << std::endl;
		for (int j = 0; j < state_ptr_vector_.size(); j++)
		{
			std::cout << j << " th:" <<  std::endl;
			state_ptr_vector_.at(j)->print();
		}
		std::cout << "AFTER state_ : " << std::endl << state_.transpose() << std::endl << std::endl;
		state_ptr_vector_.push_back(new StatePO2D(state_.data() + st * 4));
		std::cout << "New state ptr in the vector: " << std::endl;
		for (int j = 0; j < state_ptr_vector_.size(); j++)
		{
			std::cout << j << ":" <<  std::endl;
			state_ptr_vector_.at(j)->print();
		}
		// CORRESPONDENCE ODOMETRY
		if (st > 0)
		{
			std::cout << "1st: " << std::endl;
			state_ptr_vector_.at(st-1)->print();
			std::cout << "2nd: " << std::endl;
			state_ptr_vector_.back()->print();
			Correspondence2DOdometry<StatePO2D,StatePO2D>* corrOdomPtr = new Correspondence2DOdometry<StatePO2D,StatePO2D>(state_ptr_vector_.at(st-1),state_ptr_vector_.back());
			// Invent measurement
			Vector3s actualMeasurement;
			actualMeasurement.head(2) = actualDisplacement;
			actualMeasurement(2) = actualRotation;
			corrOdomPtr->inventMeasurement(actualMeasurement,generator,distribution_odometry);
			// Add correspondence
			ceres_wrapper.addCorrespondence<Correspondence2DOdometry<StatePO2D,StatePO2D>>(corrOdomPtr);
		}

//		// CORRESPONDENCE RANGE ONLY
//		for (uint st_from=0; st_from < st; st_from++)
//		{
//			//std::cout << "Range only from " << st_from << " (" << st_from*DIM << "-" << st_from*DIM+DIM-1 << ")";
//			//std::cout << " to " << st_to << " (" << st_to*DIM << "-" << st_to*DIM+DIM-1 << ")" << std::endl;
//			Correspondence2DRange* corrRangeOnlyPtr = new Correspondence2DRange(frame_ptr_vector_.at(st_from),frame_ptr_vector_.back());
//			// Invent measurement
//			Map<Vector2s> actualFrom(frame_ptr_vector_.at(st_from));
//			Map<Vector2s> actualTo(frame_ptr_vector_.back());
//			VectorXs actualMeasurement = ((actualFrom - actualTo).transpose() * (actualFrom - actualTo)).cwiseSqrt();
//			corrRangeOnlyPtr->inventMeasurement(actualMeasurement,generator,distribution_range_only);
//			// Add correspondence
//			ceres_wrapper.addCorrespondence(corrRangeOnlyPtr);
//		}
//
//		// CORRESPONDENCE GPS (2D)
//		CorrespondenceGPS2D* corrGPSPtr = new CorrespondenceGPS2D(frame_ptr_vector_.back());
//		// Invent measurement
//		corrGPSPtr->inventMeasurement(actualState_.tail(4).head(2),generator,distribution_GPS);
//		// Add correspondence
//		ceres_wrapper.addCorrespondence(corrGPSPtr);

		// SOLVE CERES PROBLEM
		if (st > 0)
		{
			ceres_wrapper.solve(true);
			std::cout << "state_ : " << std::endl << state_.transpose() << std::endl;
			std::cout << "actualState_ : " << std::endl << actualState_.transpose() << std::endl;
			std::cout << "error : " << std::endl << (actualState_ - state_).transpose() << std::endl << std::endl;
		}

	}

//    // CORRESPONDENCES
//    // SENSOR A: Absolute measurements of the whole state
//    for(uint mA=0; mA < N_MEAS_A; mA++)
//    {
//    	std::cout << "Correspondences A set: " << mA << std::endl;
//    	std::cout << "state_unit_ptr_vector_.size() = " << state_unit_ptr_vector_.size() << std::endl;
//    	for (uint st=0; st < N_STATES; st++)
//		{
//        	CorrespondenceGPS2D* corrAPtr = new CorrespondenceGPS2D(state_unit_ptr_vector_.at(st));
//			VectorXs actualMeasurement = actualState.segment(st*DIM,DIM);
//        	std::cout << "State = " << actualMeasurement.transpose() << std::endl;
//			corrAPtr->inventMeasurement(actualMeasurement,generator,distribution_A);
//			wolf_problem->addCorrespondence(corrAPtr);
//			ceres_wrapper.addCorrespondence(corrAPtr);
//		}
//    }
//    std::cout << "Correspondences A created!" << std::endl;
//
//	// SENSOR B: Relative distances between points
//    for(uint mB=0; mB < N_MEAS_B; mB++)
//	{
//    	for (uint st_from=0; st_from < N_STATES-1; st_from++)
//    	{
//    		for (uint st_to=st_from+1; st_to < N_STATES; st_to++)
//			{
//    			//std::cout << "Range only from " << st_from << " (" << st_from*DIM << "-" << st_from*DIM+DIM-1 << ")";
//    			//std::cout << " to " << st_to << " (" << st_to*DIM << "-" << st_to*DIM+DIM-1 << ")" << std::endl;
//    			Correspondence3DRange* corrBPtr = new Correspondence3DRange(state_unit_ptr_vector_[st_from],state_unit_ptr_vector_[st_to]);
//				VectorXs actualMeasurement = ((actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM)).transpose() * (actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM))).cwiseSqrt();
//				corrBPtr->inventMeasurement(actualMeasurement,generator,distribution_B);
//				wolf_problem->addCorrespondence(corrBPtr);
//				ceres_wrapper.addCorrespondence(corrBPtr);
//			}
//    	}
//	}
//
//	// run Ceres Solver
//	ceres_wrapper.solve(true);
    
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

