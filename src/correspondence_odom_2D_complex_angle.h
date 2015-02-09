
#ifndef CORRESPONDENCE_ODOM_2D_COMPLEX_ANGLE_H_
#define CORRESPONDENCE_ODOM_2D_COMPLEX_ANGLE_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_sparse.h"

class CorrespondenceOdom2DComplexAngle: public CorrespondenceSparse<2,2,2,2,2>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		CorrespondenceOdom2DComplexAngle(const FeatureBasePtr& _ftr_ptr,  WolfScalar* _block0Ptr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr);

		CorrespondenceOdom2DComplexAngle(const FeatureBasePtr& _ftr_ptr, const StateBaseShPtr& _state0Ptr, const StateBaseShPtr& _state1Ptr, const StateBaseShPtr& _state2Ptr, const StateBaseShPtr& _state3Ptr);

		virtual ~CorrespondenceOdom2DComplexAngle();

		template <typename T>
		bool operator()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2, T* _residuals) const;
};
#endif
