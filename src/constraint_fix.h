
#ifndef CONSTRAINT_FIX_H_
#define CONSTRAINT_FIX_H_

//Wolf includes
#include "constraint_sparse.h"


namespace wolf {

class ConstraintFix: public ConstraintSparse<3,2,1>
{
    public:
        static const unsigned int N_BLOCKS = 2;

        ConstraintFix(FeatureBase* _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1>(CTR_FIX, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),
                                          _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX");
            //std::cout << "creating ConstraintFix: " << std::endl;
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintFix()
        {
            std::cout << "deleting ConstraintFix " << ((ConstraintBase*)this)->id()  << std::endl;
        }

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintFix::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{

    Eigen::Map<Eigen::Matrix<T,3,1>> residuals_map(_residuals);
    Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position_map(_p);

    //std::cout << "computing constraint fix " << ((ConstraintBase*)this)->id() << std::endl;
    //std::cout << "\tmeasurement: " << getMeasurement().transpose() << std::endl;
    //std::cout << "\tstate x:  " << _p[0] << std::endl;
    //std::cout << "\tstate y:  " << _p[1] << std::endl;
    //std::cout << "\tstate th: " << _o[0] << std::endl;

    residuals_map.head(2) = getMeasurement().head(2).cast<T>() - robot_position_map;
    residuals_map(2) = T(getMeasurement()(2)) - _o[0];

    while (residuals_map(2) > T(M_PI))
    	residuals_map(2) = residuals_map(2) - T(2 * M_PI);
    while (residuals_map(2) <= T(-M_PI))
    	residuals_map(2) = residuals_map(2) + T(2 * M_PI);

    residuals_map = getMeasurementSquareRootInformation().cast<T>() * residuals_map;

    //std::cout << "+++++++  fix constraint +++++++" << std::endl;
    //std::cout << "orientation:   " << _o[0] << std::endl;
    //std::cout << "measurement:   " << T(getMeasurement()(2)) << std::endl;
    //std::cout << "residual:      " << _residuals[2] << std::endl << std::endl;
    //std::cout << "constraint fix computed!" << std::endl;
    return true;
}

} // namespace wolf

#endif
