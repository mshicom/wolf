
#ifndef CORRESPONDENCE_GPS_2D_H_
#define CORRESPONDENCE_GPS_2D_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_sparse.h"

class CorrespondenceGPS2D: public CorrespondenceSparse<2,2>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		CorrespondenceGPS2D(const FeatureBaseShPtr& _ftr_ptr, WolfScalar* _statePtr);

		CorrespondenceGPS2D(const FeatureBaseShPtr& _ftr_ptr, const StateBaseShPtr& _statePtr);

		virtual ~CorrespondenceGPS2D();

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const;
};
#endif
