
#ifndef CORRESPONDENCE_ODOM_2D_THETA_H_
#define CORRESPONDENCE_ODOM_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_sparse.h"

class CorrespondenceOdom2DTheta: public CorrespondenceSparse<2,2,1,2,1>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		CorrespondenceOdom2DTheta(const FeatureBaseShPtr& _ftr_ptr, WolfScalar* _statePtr);

		CorrespondenceOdom2DTheta(const FeatureBaseShPtr& _ftr_ptr, const StateBaseShPtr& _statePtr);

		virtual ~CorrespondenceOdom2DTheta();

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const;
};
#endif
