#include "sensor_base.h"
#include "state_block.h"
#include "hardware_base.h"
#include "processor_base.h"

SensorBase::SensorBase(const SensorType& _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                       const unsigned int _noise_size, const bool _extr_dyn) :
        NodeLinked(MID, "SENSOR"),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_size),
        noise_cov_(_noise_size, _noise_size)
{
    //
}

SensorBase::SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                       const Eigen::VectorXs & _noise_std, const bool _extr_dyn) :
        NodeLinked(MID, "SENSOR"),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_std),
        noise_cov_(_noise_std.size(), _noise_std.size())
{
    noise_cov_.setZero();
    for (unsigned int i = 0; i < _noise_std.size(); i++)
        noise_cov_(i, i) = noise_std_(i) * noise_std_(i);
}

SensorBase::~SensorBase()
{
    // Remove State Blocks
    if (p_ptr_ != nullptr)
    {
        if (getTop() != nullptr)
            getTop()->removeStateBlockPtr(p_ptr_);
        delete p_ptr_;
    }

    if (o_ptr_ != nullptr)
    {
        if (getTop() != nullptr)
            getTop()->removeStateBlockPtr(o_ptr_);
        delete o_ptr_;
    }

}

void SensorBase::addProcessor(ProcessorBase* _proc_ptr)
{
    addDownNode(_proc_ptr);
}

ProcessorBaseList* SensorBase::getProcessorListPtr()
{
    return getDownNodeListPtr();
}

StateBlock* SensorBase::getPPtr() const
{
    return p_ptr_;
}

StateBlock* SensorBase::getOPtr() const
{
    return o_ptr_;
}

StateBlock* SensorBase::getIntrinsicPtr() const
{
    return intrinsic_ptr_;
}

void SensorBase::fix()
{
    // State Blocks
    if (p_ptr_!=nullptr)
    {
        p_ptr_->fix();
        if (getTop() != nullptr)
            getTop()->updateStateBlockPtr(p_ptr_);
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->fix();
        if (getTop() != nullptr)
            getTop()->updateStateBlockPtr(o_ptr_);
    }
}

void SensorBase::unfix()
{
    // State Blocks
    if (p_ptr_!=nullptr)
    {
        p_ptr_->unfix();
        if (getTop() != nullptr)
            getTop()->updateStateBlockPtr(p_ptr_);
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->unfix();
        if (getTop() != nullptr)
            getTop()->updateStateBlockPtr(o_ptr_);
    }
}

bool SensorBase::isExtrinsicDynamic() {
	return extrinsic_dynamic_;
}


void SensorBase::setNoise(const Eigen::VectorXs& _noise_std) {
	noise_std_ = _noise_std;
	noise_cov_.setZero();
	for (unsigned int i=0; i<_noise_std.size(); i++)
		noise_cov_(i,i) = _noise_std(i) * _noise_std(i);
}

Eigen::VectorXs SensorBase::getNoiseStd() {
	return noise_std_;
}

Eigen::MatrixXs SensorBase::getNoiseCov() {
	return noise_cov_;
}
