/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.h
 * @brief   3*3 matrix representation of SO(3)
 */

#pragma once

//#include <Matrix.h> some functions will have to be created in here !
#include <Eigen/Dense>
#include "Lie.h"
#include"wolf.h"

#include <cmath>

/**
 *  True SO(3), i.e., 3*3 matrix subgroup
 *  We guarantee (all but first) constructors only generate from sub-manifold.
 *  However, round-off errors in repeated composition could move off it...
 */

namespace wolf {


class SO3: public Eigen::Matrix3s, public LieGroup<SO3, 3> {

protected:

public:
  enum {
    dimension = 3
  };

  /// @name Constructors
  /// @{

  /// Constructor from AngleAxisd
  SO3() :
      Eigen::Matrix3s(I_3x3) {
  }

  /// Constructor from Eigen Matrix
  template<typename Derived>
  SO3(const MatrixBase<Derived>& R) :
      Eigen::Matrix3s(R.eval()) {
  }

  /// Constructor from AngleAxisd
  SO3(const Eigen::AngleAxisd& angleAxis) :
      Eigen::Matrix3s(angleAxis) {
  }

  /// Static, named constructor TODO think about relation with above
  static SO3 AxisAngle(const Eigen::Vector3s& axis, double theta);

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s) const {
    std::cout << s << *this << std::endl;
  }

  bool equals(const SO3 & R, WolfScalar tol=WolfConstants::EPS) const {
    return equal_with_abs_tol(*this, R, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /// identity rotation for group operation
  static SO3 identity() {
    return I_3x3;
  }

  /// inverse of a rotation = transpose
  SO3 inverse() const {
    return this->Eigen::Matrix3s.transpose(); //May have a problem here !
  }

  /// @}
  /// @name Lie Group
  /// @{

  /**
   * Exponential map at identity - create a rotation from canonical coordinates
   * \f$ [R_x,R_y,R_z] \f$ using Rodrigues' formula
   */
  /*static SO3 Expmap(const Eigen::Vector3s& omega, ChartJacobian H = boost::none);*/
  static SO3 Expmap(const Eigen::Vector3s& omega, ChartJacobian H);

  /// Derivative of Expmap
  static Eigen::Matrix3s ExpmapDerivative(const Eigen::Vector3s& omega);

  /**
   * Log map at identity - returns the canonical coordinates
   * \f$ [R_x,R_y,R_z] \f$ of this rotation
   */
  /*static Eigen::Vector3s Logmap(const SO3& R, ChartJacobian H = boost::none);*/
  static Eigen::Vector3s Logmap(const SO3& R, ChartJacobian H);

  /// Derivative of Logmap
  static Eigen::Matrix3s LogmapDerivative(const Eigen::Vector3s& omega);

  Eigen::Matrix3s AdjointMap() const {
    return *this;
  }

  /*
  // Chart at origin
  struct ChartAtOrigin {
    static SO3 Retract(const Eigen::Vector3s& omega, ChartJacobian H = boost::none) {
      return Expmap(omega, H);
    }
    static Eigen::Vector3s Local(const SO3& R, ChartJacobian H = boost::none) {
      return Logmap(R, H);
    }
    */

  // Chart at origin
  struct ChartAtOrigin {
    static SO3 Retract(const Eigen::Vector3s& omega, ChartJacobian H) {
      return Expmap(omega, H);
    }
    static Eigen::Vector3s Local(const SO3& R, ChartJacobian H) {
      return Logmap(R, H);
    }
  };

  using LieGroup<SO3, 3>::inverse;

  /// @}
};

// This namespace exposes two functors that allow for saving computation when
// exponential map and its derivatives are needed at the same location in so<3>
// The second functor also implements dedicated methods to apply dexp and/or inv(dexp)
namespace so3 {

/// Functor implementing Exponential map
class ExpmapFunctor {
 protected:
  const double theta2;
  Eigen::Matrix3s W, K, KK;
  bool nearZero;
  double theta, sin_theta, one_minus_cos;  // only defined if !nearZero

  void init(bool nearZeroApprox = false);

 public:
  /// Constructor with element of Lie algebra so(3)
  ExpmapFunctor(const Eigen::Vector3s& omega, bool nearZeroApprox = false);

  /// Constructor with axis-angle
  ExpmapFunctor(const Eigen::Vector3s& axis, double angle, bool nearZeroApprox = false);

  /// Rodrigues formula
  SO3 expmap() const;
};

/// Functor that implements Exponential map *and* its derivatives
class DexpFunctor : public ExpmapFunctor {
  const Eigen::Vector3s omega;
  double a, b;
  Eigen::Matrix3s dexp_;

 public:
  /// Constructor with element of Lie algebra so(3)
  DexpFunctor(const Eigen::Vector3s& omega, bool nearZeroApprox = false);

  // NOTE: Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   expmap(omega + v) \approx expmap(omega) * expmap(dexp * v)
  // This maps a perturbation v in the tangent space to
  // a perturbation on the manifold Expmap(dexp * v) */
  const  Eigen::Matrix3s& dexp() const { return dexp_; }
/*
  /// Multiplies with dexp(), with optional derivatives
  Eigen::Vector3s applyDexp(const Eigen::Vector3s& v, OptionalJacobian<3, 3> H1 = boost::none,
                    OptionalJacobian<3, 3> H2 = boost::none) const;

  /// Multiplies with dexp().inverse(), with optional derivatives
  Eigen::Vector3s applyInvDexp(const Eigen::Vector3s& v,
                       OptionalJacobian<3, 3> H1 = boost::none,
                       OptionalJacobian<3, 3> H2 = boost::none) const;
*/

  /// Multiplies with dexp(), with optional derivatives
  Eigen::Vector3s applyDexp(const Eigen::Vector3s& v, OptionalJacobian<3, 3> H1,
                    OptionalJacobian<3, 3> H2) const;

  /// Multiplies with dexp().inverse(), with optional derivatives
  Eigen::Vector3s applyInvDexp(const Eigen::Vector3s& v,
                       OptionalJacobian<3, 3> H1,
                       OptionalJacobian<3, 3> H2) const;
};
}  //  namespace so3

template<>
struct traits<SO3> : public internal::LieGroup<SO3> {
};

template<>
struct traits<const SO3> : public internal::LieGroup<SO3> {
};

} //namespace wolf
