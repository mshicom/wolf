#include "feature_odom_2D.h"

FeatureOdom2D::FeatureOdom2D(const CaptureBasePtr& _capt_ptr, unsigned int _dim_measurement) :
    FeatureBase(_capt_ptr, _dim_measurement)
{
    //
}

FeatureOdom2D::FeatureOdom2D(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement) :
	FeatureBase(_capt_ptr, _measurement)
{
	//
}

FeatureOdom2D::FeatureOdom2D(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	FeatureBase(_capt_ptr, _measurement, _meas_covariance)
{
	//
}

FeatureOdom2D::~FeatureOdom2D()
{
    //
}

void FeatureOdom2D::findCorrespondences()
{
	if (getCapturePtr()->getFramePtr()->getOPtr()->getStateType() == ST_THETA)
	{
		CorrespondenceBaseShPtr odom_correspondence(new CorrespondenceOdom2DTheta(this,
																				  getCapturePtr()->getFramePtr()->getPreviousFrame()->getPPtr(),
																				  getCapturePtr()->getFramePtr()->getPreviousFrame()->getOPtr(),
																				  getCapturePtr()->getFramePtr()->getPPtr(),
																				  getCapturePtr()->getFramePtr()->getOPtr()));
		addCorrespondence(odom_correspondence);
	}
	else
	{
		CorrespondenceBaseShPtr odom_correspondence(new CorrespondenceOdom2DComplexAngle(this,
																					 	 getCapturePtr()->getFramePtr()->getPreviousFrame()->getPPtr(),
																						 getCapturePtr()->getFramePtr()->getPreviousFrame()->getOPtr(),
																						 getCapturePtr()->getFramePtr()->getPPtr(),
																						 getCapturePtr()->getFramePtr()->getOPtr()));
		addCorrespondence(odom_correspondence);
	}
}
