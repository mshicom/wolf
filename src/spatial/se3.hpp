//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_se3_hpp__
#define __se3_se3_hpp__

#include <Eigen/Geometry>
#include "spatial/fwd.hpp"
#include "spatial/skew.hpp"


namespace se3
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  namespace internal 
  {
    template<typename D>
    struct ActionReturn    { typedef D Type; };
  }

  /** The rigid transform aMb can be seen in two ways: 
   *
   * - given a point p expressed in frame B by its coordinate vector Bp, aMb
   * computes its coordinates in frame A by Ap = aMb Bp.
   * - aMb displaces a solid S centered at frame A into the solid centered in
   * B. In particular, the origin of A is displaced at the origin of B: $^aM_b
   * ^aA = ^aB$.

   * The rigid displacement is stored as a rotation matrix and translation vector by:
   * aMb (x) =  aRb*x + aAB
   * where aAB is the vector from origin A to origin B expressed in coordinates A.
   */
  template< class Derived>
  class SE3Base
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      const Angular_t & rotation() const  { return derived().rotation_impl(); }
      const Linear_t & translation() const  { return derived().translation_impl(); }
      Angular_t & rotation()  { return derived().rotation_impl(); }
      Linear_t & translation()   { return derived().translation_impl(); }
      void rotation(const Angular_t & _R) { derived().rotation_impl(_R); }
      void translation(const Linear_t & _R) { derived().translation_impl(_R); }


      Matrix4 toHomogeneousMatrix() const
      {
        return derived().toHomogeneousMatrix_impl();
      }
      operator Matrix4() const { return toHomogeneousMatrix(); }

      Matrix6 toActionMatrix() const
      {
        return derived().toActionMatrix_impl();
      }
      operator Matrix6() const { return toActionMatrix(); }



      void disp(std::ostream & os) const
      {
        static_cast<const Derived_t*>(this)->disp_impl(os);
      }

      Derived_t operator*(const Derived_t & _m2) const    { return derived().__mult__(_m2); }

      /// ay = aXb.act(by)
      template<typename D>
      typename internal::ActionReturn<D>::Type act   (const D & _d) const
      { 
        return derived().act_impl(_d);
      }
      
      /// by = aXb.actInv(ay)
      template<typename D> typename internal::ActionReturn<D>::Type actInv(const D & _d) const
      {
        return derived().actInv_impl(_d);
      }


      Derived_t act   (const Derived_t& _m2) const { return derived().act_impl(_m2); }
      Derived_t actInv(const Derived_t& _m2) const { return derived().actInv_impl(_m2); }


      bool operator == (const Derived_t & _other) const
      {
        return derived().__equal__(_other);
      }

      bool isApprox (const Derived_t & _other, const Scalar_t & prec = Eigen::NumTraits<Scalar_t>::dummy_precision()) const
      {
        return derived().isApprox_impl(_other, prec);
      }

      friend std::ostream & operator << (std::ostream & os,const SE3Base<Derived> & _X)
      { 
        _X.disp(os);
        return os;
      }

  }; // class SE3Base


  template<typename T, int U>
  struct traits< SE3Tpl<T, U> >
  {
    typedef T Scalar_t;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<T,U> Quaternion_t;
    typedef SE3Tpl<T,U> SE3;
    typedef MotionTpl<T,U> Motion;
    typedef Symmetric3Tpl<T,U> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits SE3Tpl

  template<typename _Scalar, int _Options>
  class SE3Tpl : public SE3Base< SE3Tpl< _Scalar, _Options > >
  {

  public:
    friend class SE3Base< SE3Tpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(SE3Tpl);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    SE3Tpl(): rot_(), trans_() {};


    template<typename M3,typename v3>
    SE3Tpl(const Eigen::MatrixBase<M3> & _R, const Eigen::MatrixBase<v3> & _p)
    : rot_(_R), trans_(_p)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(v3,3)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M3,3,3)
    }

    template<typename M4>
    SE3Tpl(const Eigen::MatrixBase<M4> & _m)
    : rot_(_m.template block<3,3>(LINEAR,LINEAR)), trans_(_m.template block<3,1>(LINEAR,ANGULAR))
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M4,4,4);
    }

    SE3Tpl(int) : rot_(Matrix3::Identity()), trans_(Vector3::Zero()) {}

    template<typename S2, int O2>
    SE3Tpl( const SE3Tpl<S2,O2> & _clone )
    : rot_(_clone.rotation()),trans_(_clone.translation()) {}


    template<typename S2, int O2>
    SE3Tpl & operator= (const SE3Tpl<S2,O2> & _other)
    {
      rot_ = _other.rotation ();
      trans_ = _other.translation ();
      return *this;
    }

    static SE3Tpl Identity()
    {
      return SE3Tpl(1);
    }

    SE3Tpl & setIdentity () { rot_.setIdentity (); trans_.setZero (); return *this;}

    /// aXb = bXa.inverse()
    SE3Tpl inverse() const
    {
      return SE3Tpl(rot_.transpose(), -rot_.transpose()*trans_);
    }

    static SE3Tpl Random()
    {
      Quaternion_t q(Vector4::Random());
      q.normalize();
      return SE3Tpl(q.matrix(),Vector3::Random());
    }

    SE3Tpl & setRandom ()
    {
      Quaternion_t q(Vector4::Random());
      q.normalize ();
      rot_ = q.matrix ();
      trans_.setRandom ();

      return *this;
    }
    
    Matrix4 toHomogeneousMatrix_impl() const
    {
      Matrix4 M;
      M.template block<3,3>(LINEAR,LINEAR) = rot_;
      M.template block<3,1>(LINEAR,ANGULAR) = trans_;
      M.template block<1,3>(ANGULAR,LINEAR).setZero();
      M(3,3) = 1;
      return M;
    }

    /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
    Matrix6 toActionMatrix_impl() const
    {
      typedef Eigen::Block<Matrix6,3,3> Block3;
      Matrix6 M;
      M.template block<3,3>(ANGULAR,ANGULAR)
      = M.template block<3,3>(LINEAR,LINEAR) = rot_;
      M.template block<3,3>(ANGULAR,LINEAR).setZero();
      Block3 B = M.template block<3,3>(LINEAR,ANGULAR);
      
      B.col(0) = trans_.cross(rot_.col(0));
      B.col(1) = trans_.cross(rot_.col(1));
      B.col(2) = trans_.cross(rot_.col(2));
      return M;
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  R =\n" << rot_ << std::endl
      << "  p = " << trans_.transpose() << std::endl;
    }

    /// --- GROUP ACTIONS ON M6, F6 and I6 --- 

    /// ay = aXb.act(by)
    template<typename D>
    typename internal::ActionReturn<D>::Type act_impl   (const D & _d) const
    { 
      return _d.se3Action(*this);
    }
    /// by = aXb.actInv(ay)
    template<typename D> typename internal::ActionReturn<D>::Type actInv_impl(const D & _d) const
    {
      return _d.se3ActionInverse(*this);
    }

    Vector3 act_impl   (const Vector3& _p) const { return rot_*_p+trans_; }
    Vector3 actInv_impl(const Vector3& _p) const { return rot_.transpose()*(_p-trans_); }

    SE3Tpl act_impl    (const SE3Tpl& _m2) const { return SE3Tpl( rot_*_m2.rot_,trans_+rot_*_m2.trans_);}
    SE3Tpl actInv_impl (const SE3Tpl& _m2) const { return SE3Tpl( rot_.transpose()*_m2.rot_, rot_.transpose()*(_m2.trans_-trans_));}


    SE3Tpl __mult__(const SE3Tpl & _m2) const { return this->act(_m2);}

    bool __equal__( const SE3Tpl & _m2 ) const
    {
      return (rotation_impl() == _m2.rotation() && translation_impl() == _m2.translation());
    }

    bool isApprox_impl (const SE3Tpl & m2, const Scalar_t & prec = Eigen::NumTraits<Scalar_t>::dummy_precision()) const
    {
      return rot_.isApprox(m2.rot_, prec) && trans_.isApprox(m2.trans_, prec);
    }

    const Angular_t & rotation_impl() const { return rot_; }
    Angular_t & rotation_impl() { return rot_; }
    void rotation_impl(const Angular_t & _R) { rot_ = _R; }
    const Linear_t & translation_impl() const { return trans_;}
    Linear_t & translation_impl() { return trans_;}
    void translation_impl(const Linear_t & _p) { trans_= _p; }

  protected:
    Angular_t rot_;
    Linear_t trans_;
    
  }; // class SE3Tpl

  typedef SE3Tpl<double,0> SE3;

} // namespace se3

#endif // ifndef __se3_se3_hpp__

