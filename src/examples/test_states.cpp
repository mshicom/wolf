#include <iostream>
#include "state_base.h"
#include "state_point.h"
#include "state_p.h"

using namespace std;
using namespace Eigen;

int main()
{
    cout << "\nStates demo";
    cout << "\n-----------\n" << endl;

    // cout << "check in-class static state size for PQV: " << StatePQV::SIZE_ << endl;
    // cout << "check in-class static state size for IMU: " << StateIMU::SIZE_ << endl;


    VectorXs storage(20);
    unsigned int index, bsize;

    //  cout << "Check StateBase constructor with wrong args: " << endl;
    //  VectorXs smaller(5), equal(storage.size()), larger(storage.size()+5);
    //  StateBase basetest(&storage, 0, equal);
    //  StateBase basetest(&storage, 5, equal);
    //  StateBase basetest(&storage, 25, smaller);
    //  StateBase basetest(&storage, 0, larger);
    //  StateBase basetest(&storage, 17, smaller);
    //  StateBase basetest(&storage, 25, larger);

    cout << "------ StatePoint 2D CONSTRUCTORS:" << endl;
    storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
    index = 0;
    bsize = 4;
    VectorXs VectorA(2);
    VectorA << 20, 21;
    Vector2s VectorB;
    VectorB << 20, 21;
    cout << "storage   : " << storage.transpose() << endl;
    cout << "index     : " << index << endl;
    cout << "bsize     : " << bsize << endl;
    cout << "VectorA (Xs) : " << VectorA.transpose() << endl;
    cout << "VectorB (2s) : " << VectorB.transpose() << endl;

    StatePoint point2D_b(2);
    cout << "Local constructor from size ,  size : " << point2D_b.size() << endl;
    StatePoint point2D_c(VectorA);
    cout << "Local constructor from VectorA,  x : " << point2D_c.x().transpose() << endl;
    StatePoint point2D_e(VectorB);
    cout << "Local constructor from VectorB,  x : " << point2D_e.x().transpose() << endl;
    StatePoint point2D_d(point2D_c);
    cout << "Local constructor from StatePoint,x : " << point2D_d.x().transpose() << endl;
    StatePoint point2D_f(storage, index, 2);
    cout << "Remote constructor from size,     x : " << point2D_f.x().transpose() << endl;
    StatePoint point2D_g(storage, index, VectorA);
    cout << "Remote constructor from VectorA, x : " << point2D_g.x().transpose() << endl;
    StatePoint point2D_h(storage, index, VectorB);
    cout << "Remote constructor from VectorB, x : " << point2D_h.x().transpose() << endl;
    point2D_h.remap(storage, 4);
    cout << "Remap to idx 4,                   x : " << point2D_h.x().transpose() << endl;
    cout << "Print: ";
    point2D_h.print();
    cout << "storage   : " << storage.transpose() << endl;

    cout << "------ StatePoint 3D CONSTRUCTORS:" << endl;
    storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
	index = 0;
	bsize = 4;
	VectorXs VectorC(3);
	VectorC << 40, 41, 42;
	Vector3s VectorD;
	VectorD << 50, 51, 52;
	cout << "storage   : " << storage.transpose() << endl;
	cout << "index     : " << index << endl;
	cout << "bsize     : " << bsize << endl;
	cout << "VectorC (Xs) : " << VectorC.transpose() << endl;
	cout << "VectorD (3s) : " << VectorD.transpose() << endl;

    StatePoint point3D_b(3);
    cout << "Local constructor from size ,  size : " << point3D_b.size() << endl;
    StatePoint point3D_c(VectorC);
    cout << "Local constructor from VectorC,   x : " << point3D_c.x().transpose() << endl;
    StatePoint point3D_e(VectorD);
    cout << "Local constructor from VectorD,   x : " << point3D_e.x().transpose() << endl;
    StatePoint point3D_d(point3D_c);
    cout << "Local constructor from StatePoint,x : " << point3D_d.x().transpose() << endl;
    StatePoint point3D_f(storage, index, 3);
    cout << "Remote constructor from size,     x : " << point3D_f.x().transpose() << endl;
    StatePoint point3D_g(storage, index, VectorC);
    cout << "Remote constructor from VectorA,  x : " << point3D_g.x().transpose() << endl;
    StatePoint point3D_h(storage, index, VectorD);
    cout << "Remote constructor from VectorD,  x : " << point3D_h.x().transpose() << endl;
    point3D_h.remap(storage, 4);
    cout << "Remap to idx 4,                   x : " << point3D_h.x().transpose() << endl;
    cout << "Print: ";
    point3D_h.print();
	cout << "storage   : " << storage.transpose() << endl;
    
    cout << "------ StateP<2> CONSTRUCTORS:" << endl;
	storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
	index = 5;
	bsize = 6;
	VectorXs VectorE(5);
	VectorE << 60, 61, 62, 63, 64;
	cout << "storage   : " << storage.transpose() << endl;
	cout << "index     : " << index << endl;
	cout << "bsize     : " << bsize << endl;
	cout << "VectorA (Xs) : " << VectorA.transpose() << endl;
	cout << "VectorE (Xs) : " << VectorE.transpose() << endl;

	StateP<2> p2D_a;
	cout << "Local constructor ,            size : " << p2D_a.size() << endl;
	StateP<2> p2D_b(bsize);
	cout << "Local constructor from bsize,  size : " << p2D_b.size() << endl;
	StateP<2> p2D_c(VectorA);
	cout << "Local constructor from VectorA,   x : " << p2D_c.x().transpose() << endl;
	StateP<2> p2D_e(VectorE);
	cout << "Local constructor from VectorE,   x : " << p2D_e.x().transpose() << endl;
	StateP<2> p2D_d(p2D_e);
	cout << "Local constructor from StateP,    x : " << p2D_d.x().transpose() << endl;
	StateP<2> p2D_f(storage, index, bsize);
	cout << "Remote constructor from bsize,    x : " << p2D_f.x().transpose() << endl;
	StateP<2> p2D_g(storage, index, VectorA);
	cout << "Remote constructor from VectorA,  x : " << p2D_g.x().transpose() << endl;
	StateP<2> p2D_h(storage, index, VectorE);
	cout << "Remote constructor from VectorE,  x : " << p2D_h.x().transpose() << endl;
	p2D_h.remap(storage, 3);
	cout << "Remap to idx 3,                   x : " << p2D_h.x().transpose() << endl;
	cout << "Print: ";
	p2D_h.print();
	cout << "storage   : " << storage.transpose() << endl;

	cout << "------ StateP<3> CONSTRUCTORS:" << endl;
	storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
	index = 5;
	bsize = 6;
	cout << "storage   : " << storage.transpose() << endl;
	cout << "index     : " << index << endl;
	cout << "bsize     : " << bsize << endl;
	cout << "VectorC (Xs) : " << VectorC.transpose() << endl;
	cout << "VectorE (Xs) : " << VectorE.transpose() << endl;

	StateP<3> p3D_a;
	cout << "Local constructor from bsize,  size : " << p3D_a.size() << endl;
	StateP<3> p3D_b(bsize);
	cout << "Local constructor from bsize,  size : " << p3D_b.size() << endl;
	StateP<3> p3D_c(VectorC);
	cout << "Local constructor from VectorC,   x : " << p3D_c.x().transpose() << endl;
	StateP<3> p3D_e(VectorE);
	cout << "Local constructor from VectorE,   x : " << p3D_e.x().transpose() << endl;
	StateP<3> p3D_d(p3D_e);
	cout << "Local constructor from StateP,    x : " << p3D_d.x().transpose() << endl;
	StateP<3> p3D_f(storage, index, bsize);
	cout << "Remote constructor from bsize,    x : " << p3D_f.x().transpose() << endl;
	StateP<3> p3D_g(storage, index, VectorC);
	cout << "Remote constructor from VectorC,  x : " << p3D_g.x().transpose() << endl;
	StateP<3> p3D_h(storage, index, VectorE);
	cout << "Remote constructor from VectorE,  x : " << p3D_h.x().transpose() << endl;
	p3D_h.remap(storage, 3);
	cout << "Remap to idx 3,                   x : " << p3D_h.x().transpose() << endl;
	cout << "Print: ";
	p3D_h.print();
	cout << "storage   : " << storage.transpose() << endl;



    //cout << "------ StatePose<3> CONSTRUCTORS:" << endl;
    //StatePose<3> pose3D_a;
    //cout << "Default local constructor,    size : " << pose3D_a.size() << endl;
    // VectorXs pqvstate(StatePQV::SIZE_);
    // pqvstate << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;
    // VectorXs imustate(StateIMU::SIZE_);
    // imustate << 50, 51, 52, 53, 54, 55;
    // StatePQV pqv(storage, index + bsize, pqvstate);
    // StateIMU imu(storage, index + bsize + StatePQV::SIZE_, imustate);
    // cout << "storage   : " << storage.transpose() << endl;
    // cout << "pqv  state: " << pqv.x().transpose() << endl;
    // cout << "pqv  pos  : " << pqv.p().transpose() << endl;
    // cout << "pqv  vel  : " << pqv.v().transpose() << endl;
    // cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
    // cout << "imu  state: " << imu.x().transpose() << endl;
    // cout << "imu  abias: " << imu.ab().transpose() << endl;
    // cout << "imu  wbias: " << imu.wb().transpose() << endl;
    // cout << "doing 'base.x() *= 2'" << endl;

//     cout << "doing 'pqv.x() *= 3'" << endl;
//     pqv.x() *= 3;
//     cout << "pqv  state: " << pqv.x().transpose() << endl;
//     cout << "pqv  pos  : " << pqv.p().transpose() << endl;
//     cout << "pqv  vel  : " << pqv.v().transpose() << endl;
//     cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
//     cout << "doing 'pqv.p().array() += 2' ; 'pqv.v() *= 2' ; 'pqv.q().normalize()'" << endl;
//     pqv.p().array() += 2;
//     pqv.v() *= 2;
//     pqv.q().normalize();
//     cout << "pqv  pos  : " << pqv.p().transpose() << endl;
//     cout << "pqv  vel  : " << pqv.v().transpose() << endl;
//     cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
//     cout << "pqv  R    : \n" << pqv.q().matrix() << endl;
//     cout << "pqv  R*vel: " << (pqv.q().matrix() * pqv.v()).transpose() << endl;
//     cout << "pqv  q*vel: " << (pqv.q() * pqv.v()).transpose() << endl;
//     cout << "pqv  state: " << pqv.x().transpose() << endl;
//     cout << "storage   : " << storage.transpose() << endl;
//     cout << "doing 'storage *= 2'" << endl;
//     storage *= 2;
//     cout << "quat norm : " << pqv.q().norm() << endl;
//     cout << "doing 'pqv.q().setIdentity()'" << endl;
//     pqv.q().setIdentity();
//     cout << "pqv  quat : " << pqv.q().coeffs().transpose();
//     cout << endl;

//     cout << "\n---Testing local and remote states---" << endl;
//     cout << "imustate  : " << imustate.transpose() << endl;
//     StateIMU imulocal(imustate);
//     cout << "imulocal  : " << imulocal.x().transpose() << endl;
//     cout << "imulocal ab, wb: " << imulocal.ab().transpose() << " , " << imulocal.wb().transpose() << endl;
//     cout << "imu       : " << imu.x().transpose() << endl;
//     cout << "doing 'imulocal.abias() *= 3'" << endl;
//     imulocal.ab() *= 3;
//     cout << "imulocal  : " << imulocal.x().transpose() << endl;
//     cout << "imu       : " << imu.x().transpose() << endl;
//     cout << "doing imu = imulocal" << endl;
//     imu = imulocal;
//     cout << "imu       : " << imu.x().transpose() << endl;
//     cout << "storage   : " << storage.transpose() << endl;
//     StatePQV pqvlocal(pqvstate);
//     cout << "pqvlocal  : " << pqvlocal.x().transpose() << endl;
//     cout << "pqvlocal p, v, q: " << pqvlocal.p().transpose() << " , " << pqvlocal.v().transpose() << " , "
//             << pqvlocal.q().coeffs().transpose() << endl;
//     cout << "pqv       : " << pqv.x().transpose() << endl;
//     cout << "doing 'pqvlocal.v() *= 2'" << endl;
//     pqvlocal.v() *= 2;
//     cout << "pqvlocal  : " << pqvlocal.x().transpose() << endl;
//     cout << "pqv       : " << pqv.x().transpose() << endl;
//     cout << "doing pqv = pqvlocal" << endl;
//     pqv = pqvlocal;
//     cout << "pqv       : " << pqv.x().transpose() << endl;
//     cout << "storage   : " << storage.transpose() << endl;

//     // Miscelaneous
//     Quaternions q(1, 2, 3, 4);
//     cout << "\nAttention!!! constructing Quaternions q(1,2,3,4) gives coeffs order: " << q.coeffs().transpose();
//     cout << endl;

// //    cout << "\n--- Test composite state StatePQVBB, with PQV and IMU ---" << endl;
// //
// //    cout << "--- 1. With local states ---" << endl;
// //    StateCompPQVBB pqvbblocal(pqvstate, imustate);
// //    cout << "storage   : " << storage.transpose() << endl;
// //    cout << "pqvbblocal p  : " << pqvbblocal.p().transpose() << endl;
// //    cout << "pqvbblocal v  : " << pqvbblocal.v().transpose() << endl;
// //    cout << "pqvbblocal q  : " << pqvbblocal.q().coeffs().transpose() << endl;
// //    cout << "pqvbblocal ab : " << pqvbblocal.abias().transpose() << endl;
// //    cout << "pqvbblocal wb : " << pqvbblocal.wbias().transpose() << endl;
// //    cout << "pqvbblocal pqv: " << pqvbblocal.StatePQV::x().transpose() << endl;
// //    cout << "pqvbblocal bb : " << pqvbblocal.StateIMU::x().transpose() << endl;
// //
// //    cout << "--- 2. With remote states ---" << endl;
// //    StateCompPQVBB pqvbb(storage, index + bsize, index + bsize + StatePQV::SIZE_, pqvstate, imustate);
// //    cout << "storage   : " << storage.transpose() << endl;
// //    cout << "doing 'storage *= 2'" << endl;
// //    storage *= 2;
// //    cout << "storage   : " << storage.transpose() << endl;
// //    cout << "pqvbb p  : " << pqvbb.p().transpose() << endl;
// //    cout << "pqvbb v  : " << pqvbb.v().transpose() << endl;
// //    cout << "pqvbb q  : " << pqvbb.q().coeffs().transpose() << endl;
// //    cout << "pqvbb ab : " << pqvbb.abias().transpose() << endl;
// //    cout << "pqvbb wb : " << pqvbb.wbias().transpose() << endl;
// //    cout << "pqvbb pqv: " << pqvbb.StatePQV::x().transpose() << endl;
// //    cout << "pqvbb bb : " << pqvbb.StateIMU::x().transpose() << endl;
// //
// //    cout << "pqvbb     SIZE_ : " << pqvbb.SIZE_ << endl;
// //    cout << "pqvbb pqv SIZE_ : " << pqvbb.StatePQV::SIZE_ << endl;
// //    cout << "pqvbb bb  SIZE_ : " << pqvbb.StateIMU::SIZE_ << endl;

//     cout << "\n---Testing local error states---" << endl;
//     VectorXs pvqerror;
//     pvqerror = VectorXs::Constant(StateErrorPQV::SIZE_ERROR_, 0.001);
//     StateErrorPQV epqv( pqvstate );
//     epqv.qn().normalize();
//     epqv.xe() = pvqerror;
//     cout << "epvq pn: " << epqv.pn().transpose() << endl;
//     cout << "epvq qn: " << epqv.qn().coeffs().transpose() << endl;
//     cout << "epvq vn: " << epqv.vn().transpose() << endl;
//     cout << "epvq pe: " << epqv.pe().transpose() << endl;
//     cout << "epvq qe: " << epqv.qe().transpose() << endl;
//     cout << "epvq ve: " << epqv.ve().transpose() << endl;
//     cout << "epvq pc: " << epqv.pc().transpose() << endl;
//     cout << "epvq qc: " << epqv.qc().coeffs().transpose() << endl;
//     cout << "epvq vc: " << epqv.vc().transpose() << endl;

//     cout << "----------------------------" << endl;
    return 0;
}

