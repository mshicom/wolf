
#ifndef CORRESPONDENCE_SPARSE_H_
#define CORRESPONDENCE_SPARSE_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_base.h"

//TODO: 
// - public static const may be are not necessary, since sizes are already kept in CorrespondenceBase::state_block_sizes_vector_
// - measurement_ptr can be set from FeatureBase::measurement_, once this correspondence is up-linked to a feature. 
//   May be a simple get is //enough to access this data.
// - 

//template class CorrespondenceBase
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
            CorrespondenceXBase(_measurementPtr),
            block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            for (uint i = 0; i<block_sizes_vector_.size(); i++)
            {
                if (block_sizes_vector_.at(i) == 0)
                {
                    block_sizes_vector_.resize(i);
                    break;
                }
                else
                    state_block_ptr_vector_.push_back(_blockPtrArray[i]);
            }
        }

        CorrespondenceSparse(WolfScalar* _measurementPtr,
                             WolfScalar* _state0Ptr,
                             WolfScalar* _state1Ptr = nullptr,
                             WolfScalar* _state2Ptr = nullptr,
                             WolfScalar* _state3Ptr = nullptr,
                             WolfScalar* _state4Ptr = nullptr,
                             WolfScalar* _state5Ptr = nullptr,
                             WolfScalar* _state6Ptr = nullptr,
                             WolfScalar* _state7Ptr = nullptr,
                             WolfScalar* _state8Ptr = nullptr,
                             WolfScalar* _state9Ptr = nullptr ) :
            CorrespondenceXBase(_measurementPtr),
            state_block_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            for (uint i = 0; i<block_sizes_vector_.size(); i++)
            {
                if (block_sizes_vector_.at(i) == 0)
                {
                    block_sizes_vector_.resize(i);
                    state_block_ptr_vector_.resize(i);
                    break;
                }
            }

            //TODO: Check if while size OK, pointers NULL
        }

        virtual ~CorrespondenceSparse()
        {
        }

        virtual correspondenceType getType() const = 0;

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

        CorrespondenceGPS2D(WolfScalar* _measurementPtr, const StateXShPtr& _statePtr) :
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
            return GPS_2D;
        }
};