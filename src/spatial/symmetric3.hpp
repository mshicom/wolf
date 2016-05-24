
// Copyright LAAS-CNRS, 2014-2016

// This file is originally copied from metapod/tools/spatial/lti.hh.
// Authors: Olivier Stasse (LAAS, CNRS) and Sébastien Barthélémy (Aldebaran Robotics)
// The file was modified in pinocchio by Nicolas Mansard (LAAS, CNRS)

// metapod is free software, distributed under the terms of the GNU Lesser
// General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef __se3__symmetric3_hpp__
#define __se3__symmetric3_hpp__

#include <ostream>

namespace se3
{

template<typename _Scalar, int _Options>
class Symmetric3Tpl
{
    public:
        typedef _Scalar Scalar;
        enum { Options = _Options };
        typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
        typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
        typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
        typedef Eigen::Matrix<Scalar,2,2,Options> Matrix2;
        typedef Eigen::Matrix<Scalar,3,2,Options> Matrix32;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        public:
            Symmetric3Tpl(): data_() {}
        template<typename Sc,int _N,int _Opt>
        explicit Symmetric3Tpl(const Eigen::Matrix<Sc,_N,_N,_Opt> & _I)
        {
            assert( (_I.rows()==3)&&(_I.cols()==3) );
            assert( (_I-_I.transpose()).isMuchSmallerThan(_I) );
            data_(0) = _I(0,0);
            data_(1) = _I(1,0); data_(2) = _I(1,1);
            data_(3) = _I(2,0); data_(4) = _I(2,1); data_(5) = _I(2,2);
        }
        explicit Symmetric3Tpl(const Eigen::MatrixBase<Matrix3> &_I)
        {
            assert( (_I-_I.transpose()).isMuchSmallerThan(_I) );
            data_(0) = _I(0,0);
            data_(1) = _I(1,0); data_(2) = _I(1,1);
            data_(3) = _I(2,0); data_(4) = _I(2,1); data_(5) = _I(2,2);
        }
        explicit Symmetric3Tpl(const Vector6 &_I) : data_(_I) {}
        Symmetric3Tpl(const double & _a0,const double & _a1,const double & _a2,
                      const double & _a3,const double & _a4,const double & _a5)
        { data_ << _a0,_a1,_a2,_a3,_a4,_a5; }

        static Symmetric3Tpl Zero()     { return Symmetric3Tpl(Vector6::Zero()  );  }
        void setZero() { data_.setZero(); }

        static Symmetric3Tpl Random()   { return RandomPositive();  }
        void setRandom()
        {
            double
                    a = double(std::rand())/RAND_MAX*2.0-1.0,
                    b = double(std::rand())/RAND_MAX*2.0-1.0,
                    c = double(std::rand())/RAND_MAX*2.0-1.0,
                    d = double(std::rand())/RAND_MAX*2.0-1.0,
                    e = double(std::rand())/RAND_MAX*2.0-1.0,
                    f = double(std::rand())/RAND_MAX*2.0-1.0;

            data_ << a, b, c, d, e, f;
        }

        static Symmetric3Tpl Identity() { return Symmetric3Tpl(1, 0, 1, 0, 0, 1);  }
        void setIdentity() { data_ << 1, 0, 1, 0, 0, 1; }

        /* Requiered by Inertia::operator== */
        bool operator== (const Symmetric3Tpl & S2 ) const { return data_ == S2.data_; }

        void fill(const Scalar _value) { data_.fill(_value); }

        struct SkewSquare
        {
                const Vector3 & v_;
                SkewSquare( const Vector3 & _v ) : v_(_v) {}
                operator Symmetric3Tpl () const
                {
                    const double & x = v_[0], & y = v_[1], & z = v_[2];
                    return Symmetric3Tpl( -y*y-z*z,
                                          x*y    ,  -x*x-z*z,
                                          x*z    ,   y*z    ,  -x*x-y*y );
                }
        }; // struct SkewSquare
        Symmetric3Tpl operator- (const SkewSquare & _v) const
        {
            const double & x = _v.v_[0], & y = _v.v_[1], & z = _v.v_[2];
            return Symmetric3Tpl( data_[0]+y*y+z*z,
                    data_[1]-x*y    ,  data_[2]+x*x+z*z,
                    data_[3]-x*z    ,  data_[4]-y*z    ,  data_[5]+x*x+y*y );
        }
        Symmetric3Tpl& operator-= (const SkewSquare & _v)
        {
            const double & x = _v.v_[0], & y = _v.v_[1], & z = _v.v_[2];
            data_[0]+=y*y+z*z;
            data_[1]-=x*y    ;  data_[2]+=x*x+z*z;
            data_[3]-=x*z    ;  data_[4]-=y*z    ;  data_[5]+=x*x+y*y;
            return *this;
        }

        struct AlphaSkewSquare
        {
                const double & m_;  const Vector3 & v_;
                AlphaSkewSquare( const double & _m, const SkewSquare & _v ) : m_(_m),v_(_v.v_) {}
                operator Symmetric3Tpl () const
                {
                    const double & x = v_[0], & y = v_[1], & z = v_[2];
                    return Symmetric3Tpl( -m_*(y*y+z*z),
                                          m_* x*y     ,  -m_*(x*x+z*z),
                                          m_* x*z     ,   m_* y*z     ,  -m_*(x*x+y*y) );
                }
        };
        friend AlphaSkewSquare operator* (const double & _m, const SkewSquare & _sk )
        { return AlphaSkewSquare(_m,_sk); }
        Symmetric3Tpl operator- (const AlphaSkewSquare & _v) const
        {
            const double & x = _v.v_[0], & y = _v.v_[1], & z = _v.v_[2];
            return Symmetric3Tpl( data_[0]+_v.m_*(y*y+z*z),
                    data_[1]-_v.m_* x*y     ,  data_[2]+_v.m_*(x*x+z*z),
                    data_[3]-_v.m_* x*z     ,  data_[4]-_v.m_* y*z     ,  data_[5]+_v.m_*(x*x+y*y) );
        }
        Symmetric3Tpl& operator-= (const AlphaSkewSquare & _v)
        {
            const double & x = _v.v_[0], & y = _v.v_[1], & z = _v.v_[2];
            data_[0]+=_v.m_*(y*y+z*z);
            data_[1]-=_v.m_* x*y     ;  data_[2]+=_v.m_*(x*x+z*z);
            data_[3]-=_v.m_* x*z     ;  data_[4]-=_v.m_* y*z     ;  data_[5]+=_v.m_*(x*x+y*y);
            return *this;
        }

        const Vector6 & data () const {return data_;}
        Vector6 & data () {return data_;}

        // static Symmetric3Tpl SkewSq( const Vector3 & v )
        // {
        //   const double & x = v[0], & y = v[1], & z = v[2];
        //   return Symmetric3Tpl( -y*y-z*z,
        // 			    x*y    ,  -x*x-z*z,
        // 			    x*z    ,   y*z    ,  -x*x-y*y );
        // }

        /* Shoot a positive definite matrix. */
        static Symmetric3Tpl RandomPositive()
        {
            double
                    a = double(std::rand())/RAND_MAX*2.0-1.0,
                    b = double(std::rand())/RAND_MAX*2.0-1.0,
                    c = double(std::rand())/RAND_MAX*2.0-1.0,
                    d = double(std::rand())/RAND_MAX*2.0-1.0,
                    e = double(std::rand())/RAND_MAX*2.0-1.0,
                    f = double(std::rand())/RAND_MAX*2.0-1.0;
            return Symmetric3Tpl(a*a+b*b+d*d,
                                 a*b+b*c+d*e, b*b+c*c+e*e,
                                 a*d+b*e+d*f, b*d+c*e+e*f,  d*d+e*e+f*f );
        }


        Matrix3 matrix() const
        {
            Matrix3 res;
            res(0,0) = data_(0); res(0,1) = data_(1); res(0,2) = data_(3);
            res(1,0) = data_(1); res(1,1) = data_(2); res(1,2) = data_(4);
            res(2,0) = data_(3); res(2,1) = data_(4); res(2,2) = data_(5);
            return res;
        }
        operator Matrix3 () const { return matrix(); }

        Scalar vtiv (const Vector3 & _v) const
        {
            const Scalar & x = _v[0];
            const Scalar & y = _v[1];
            const Scalar & z = _v[2];

            const Scalar xx = x*x;
            const Scalar xy = x*y;
            const Scalar xz = x*z;
            const Scalar yy = y*y;
            const Scalar yz = y*z;
            const Scalar zz = z*z;

            return data_(0)*xx + data_(2)*yy + data_(5)*zz + 2.*(data_(1)*xy + data_(3)*xz + data_(4)*yz);
        }

        Symmetric3Tpl operator+(const Symmetric3Tpl & _s2) const
        {
            return Symmetric3Tpl((data_+_s2.data_).eval());
        }

        Symmetric3Tpl & operator+=(const Symmetric3Tpl & _s2)
        {
            data_ += _s2.data_; return *this;
        }

        Vector3 operator*(const Vector3 &_v) const
        {
            return Vector3(
                        data_(0) * _v(0) + data_(1) * _v(1) + data_(3) * _v(2),
                        data_(1) * _v(0) + data_(2) * _v(1) + data_(4) * _v(2),
                        data_(3) * _v(0) + data_(4) * _v(1) + data_(5) * _v(2)
                        );
        }

        // Matrix3 operator*(const Matrix3 &a) const
        // {
        //   Matrix3 r;
        //   for(unsigned int i=0; i<3; ++i)
        //     {
        //       r(0,i) = data_(0) * a(0,i) + data_(1) * a(1,i) + data_(3) * a(2,i);
        //       r(1,i) = data_(1) * a(0,i) + data_(2) * a(1,i) + data_(4) * a(2,i);
        //       r(2,i) = data_(3) * a(0,i) + data_(4) * a(1,i) + data_(5) * a(2,i);
        //     }
        //   return r;
        // }

        const Scalar& operator()(const int &_i,const int &_j) const
        {
            return ((_i!=2)&&(_j!=2)) ? data_[_i+_j] : data_[_i+_j+1];
            }

            Symmetric3Tpl operator-(const Matrix3 &_S) const
            {
            assert( (_S-_S.transpose()).isMuchSmallerThan(_S) );
            return Symmetric3Tpl( data_(0)-_S(0,0),
                                  data_(1)-_S(1,0), data_(2)-_S(1,1),
                                  data_(3)-_S(2,0), data_(4)-_S(2,1), data_(5)-_S(2,2) );
        }

        Symmetric3Tpl operator+(const Matrix3 &_S) const
        {
            assert( (_S-_S.transpose()).isMuchSmallerThan(_S) );
            return Symmetric3Tpl( data_(0)+_S(0,0),
                                  data_(1)+_S(1,0), data_(2)+_S(1,1),
                                  data_(3)+_S(2,0), data_(4)+_S(2,1), data_(5)+_S(2,2) );
        }

        /* --- Symmetric R*S*R' and R'*S*R products --- */
    public: //private:

        /** \brief Computes L for a symmetric matrix A.
     */
        Matrix32  decomposeltI() const
        {
            Matrix32 L;
            L <<
                 data_(0) - data_(5),    data_(1),
                    data_(1),              data_(2) - data_(5),
                    2*data_(3),            data_(4) + data_(4);
            return L;
        }

        /* R*S*R' */
        template<typename D>
        Symmetric3Tpl rotate(const Eigen::MatrixBase<D> & _R) const
        {
            assert( (_R.cols()==3) && (_R.rows()==3) );
            assert( (_R.transpose()*_R).isApprox(Matrix3::Identity()) );

            Symmetric3Tpl Sres;

            // 4 a
            const Matrix32 L( decomposeltI() );

            // Y = R' L   ===> (12 m + 8 a)
            const Matrix2 Y( _R.template block<2,3>(1,0) * L );

            // Sres= Y R  ===> (16 m + 8a)
            Sres.data_(1) = Y(0,0)*_R(0,0) + Y(0,1)*_R(0,1);
            Sres.data_(2) = Y(0,0)*_R(1,0) + Y(0,1)*_R(1,1);
            Sres.data_(3) = Y(1,0)*_R(0,0) + Y(1,1)*_R(0,1);
            Sres.data_(4) = Y(1,0)*_R(1,0) + Y(1,1)*_R(1,1);
            Sres.data_(5) = Y(1,0)*_R(2,0) + Y(1,1)*_R(2,1);

            // r=R' v ( 6m + 3a)
            const Vector3 r( -_R(0,0)*data_(4) + _R(0,1)*data_(3),
                             -_R(1,0)*data_(4) + _R(1,1)*data_(3),
                             -_R(2,0)*data_(4) + _R(2,1)*data_(3) );

            // Sres_11 (3a)
            Sres.data_(0) = L(0,0) + L(1,1) - Sres.data_(2) - Sres.data_(5);

            // Sres + D + (Ev)x ( 9a)
            Sres.data_(0) += data_(5);
            Sres.data_(1) += r(2);    Sres.data_(2)+= data_(5);
            Sres.data_(3) +=-r(1);    Sres.data_(4)+=    r(0); Sres.data_(5) += data_(5);

            return Sres;
        }


    private:

        Vector6 data_;
};

typedef Symmetric3Tpl<double,0> Symmetric3;

} // namespace se3

#endif // ifndef __se3__symmetric3_hpp__

