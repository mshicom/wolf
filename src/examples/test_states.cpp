#include <iostream>
#include "state_base.h"
#include "state_point.h"
#include "state_p.h"
#include "state_orientation.h"
#include "state_po.h"

using namespace std;
using namespace Eigen;

int main()
{
	bool test_point = 0;
	bool test_p = 0;
	bool test_orientation = 0;
	bool test_operators= 0;
	bool test_operators2= 1;
	bool test_po2 = 0;
	bool test_po3 = 0;

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
    if(test_point)
    {
		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "-------- StatePoint 2D CONSTRUCTORS --------" << endl;
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
		cout << "VectorB (2s) : " << VectorB.transpose() << endl << endl;

		cout << "Local constructor from size 2" << endl;
		StatePoint point2D_a(2);
		point2D_a.print();

		cout << "Local constructor from VectorA" << endl;
		StatePoint point2D_b(VectorA);
		point2D_b.print();

		cout << "Local copy constructor" << endl;
		StatePoint point2D_c(VectorB);
		point2D_c.print();

		cout << "Local constructor from StatePoint" << endl;
		StatePoint point2D_d(point2D_c);
		point2D_d.print();

		cout << "Remote constructor from size" << endl;
		StatePoint point2D_e(storage, index, 2);
		point2D_e.print();

		cout << "Remote constructor from VectorA" << endl;
		StatePoint point2D_f(storage, index, VectorA);
		point2D_f.print();

		cout << "Remote constructor from VectorB" << endl;
		StatePoint point2D_g(storage, index, VectorB);
		point2D_g.print();

		cout << "Remap to idx 4" << endl;
		point2D_g.remap(storage, 4);
		point2D_g.print();

		cout << "storage   : " << storage.transpose() << endl;

		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "-------- StatePoint 3D CONSTRUCTORS --------" << endl;
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
		cout << "VectorD (3s) : " << VectorD.transpose() << endl << endl;

		cout << "Local constructor from size" << endl;
		StatePoint point3D_a(3);
		point3D_a.print();

		cout << "Local constructor from VectorC" << endl;
		StatePoint point3D_b(VectorC);
		point3D_b.print();

		cout << "Local constructor from VectorD" << endl;
		StatePoint point3D_c(VectorD);
		point3D_c.print();

		cout << "Local copy constructor" << endl;
		StatePoint point3D_d(point3D_c);
		point3D_d.print();

		cout << "Remote constructor from size" << endl;
		StatePoint point3D_e(storage, index, 3);
		point3D_e.print();

		cout << "Remote constructor from VectorA" << endl;
		StatePoint point3D_f(storage, index, VectorC);
		point3D_f.print();

		cout << "Remote constructor from VectorD" << endl;
		StatePoint point3D_g(storage, index, VectorD);
		point3D_g.print();

		cout << "Remap to idx 4" << endl;
		point3D_g.remap(storage, 4);
		point3D_g.print();

		cout << "storage   : " << storage.transpose() << endl;
    }
    if(test_p)
    {
    	cout << endl;
    	cout << "--------------------------------------------" << endl;
		cout << "---------- StateP<2> CONSTRUCTORS ----------" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 5;
		bsize = 6;
		VectorXs VectorA(2);
		VectorA << 20, 21;
		VectorXs VectorC(3);
		VectorC << 40, 41, 42;
		VectorXs VectorE(5);
		VectorE << 60, 61, 62, 63, 64;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "bsize     : " << bsize << endl;
		cout << "VectorA (Xs) : " << VectorA.transpose() << endl;
		cout << "VectorE (Xs) : " << VectorE.transpose() << endl << endl;

		cout << "Local constructor" << endl;
		StateP<2> p2D_a;
		p2D_a.print();

		cout << "Local constructor from bsize" << endl;
		StateP<2> p2D_b(bsize);
		p2D_b.print();

		cout << "Local constructor from VectorA" << endl;
		StateP<2> p2D_c(VectorA);
		p2D_c.print();

		cout << "Local constructor from VectorE" << endl;
		StateP<2> p2D_d(VectorE);
		p2D_d.print();

		cout << "Local copy constructor" << endl;
		StateP<2> p2D_e(p2D_d);
		p2D_e.print();

		cout << "Remote constructor from bsize" << endl;
		StateP<2> p2D_f(storage, index, bsize);
		p2D_f.print();

		cout << "Remote constructor from VectorA" << endl;
		StateP<2> p2D_g(storage, index, VectorA);
		p2D_g.print();

		cout << "Remote constructor from VectorE" << endl;
		StateP<2> p2D_h(storage, index, VectorE);
		p2D_h.print();

		cout << "Remap to idx 3" << endl;
		p2D_h.remap(storage, 3);
		p2D_h.print();

		cout << "storage   : " << storage.transpose() << endl;

		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "---------- StateP<3> CONSTRUCTORS ----------" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 5;
		bsize = 6;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "bsize     : " << bsize << endl;
		cout << "VectorC (Xs) : " << VectorC.transpose() << endl;
		cout << "VectorE (Xs) : " << VectorE.transpose() << endl << endl;

		cout << "Local constructor from bsize" << endl;
		StateP<3> p3D_a;
		p3D_a.print();

		cout << "Local constructor from bsize" << endl;
		StateP<3> p3D_b(bsize);
		p3D_b.print();

		cout << "Local constructor from VectorC" << endl;
		StateP<3> p3D_c(VectorC);
		p3D_c.print();

		cout << "Local constructor from VectorE" << endl;
		StateP<3> p3D_d(VectorE);
		p3D_d.print();

		cout << "Local copy constructor" << endl;
		StateP<3> p3D_e(p3D_d);
		p3D_e.print();

		cout << "Remote constructor from bsize" << endl;
		StateP<3> p3D_f(storage, index, bsize);
		p3D_f.print();

		cout << "Remote constructor from VectorC" << endl;
		StateP<3> p3D_g(storage, index, VectorC);
		p3D_g.print();

		cout << "Remote constructor from VectorE" << endl;
		StateP<3> p3D_h(storage, index, VectorE);
		p3D_h.print();

		cout << "Remap to idx 3" << endl;
		p3D_h.remap(storage, 3);
		p3D_h.print();

		cout << "storage   : " << storage.transpose() << endl;
    }
    if(test_orientation)
    {
		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "--- StateOrientation<THETA> CONSTRUCTORS ---" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 0;
		VectorXs VectorF(1);
		VectorF << 2.7;
		Matrix<WolfScalar,1,1> VectorG(1);
		VectorG << 3.7;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "VectorF (Xs) : " << VectorF.transpose() << endl;
		cout << "VectorG (1s) : " << VectorG.transpose() << endl << endl;

		cout << "Local constructor from size" << endl;
		StateOrientation<THETA> orientation2D_a;
		orientation2D_a.print();

		cout << "Local constructor from VectorF" << endl;
		StateOrientation<THETA> orientation2D_b(VectorF);
		orientation2D_b.print();

		cout << "Local constructor from VectorG" << endl;
		StateOrientation<THETA> orientation2D_c(VectorG);
		orientation2D_c.print();

		cout << "Local copy constructor" << endl;
		StateOrientation<THETA> orientation2D_d(orientation2D_c);
		orientation2D_d.print();

		cout << "Remote constructor from size" << endl;
		StateOrientation<THETA> orientation2D_e(storage, index);
		orientation2D_e.print();

		cout << "Remote constructor from VectorF" << endl;
		StateOrientation<THETA> orientation2D_f(storage, index, VectorF);
		orientation2D_f.print();

		cout << "Remote constructor from VectorG" << endl;
		StateOrientation<THETA> orientation2D_g(storage, index, VectorG);
		orientation2D_g.print();

		cout << "Remap to idx 4" << endl;
		orientation2D_g.remap(storage, 4);
		orientation2D_g.print();

		cout << "storage   : " << storage.transpose() << endl;


		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "--- StateOrientation<EULER> CONSTRUCTORS ---" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 0;
		VectorXs VectorH(3);
		VectorH << 1.1, 2.2, 3.3;
		Vector3s VectorI;
		VectorI << 4.4, 5.5, 6.6;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "VectorH (Xs) : " << VectorH.transpose() << endl;
		cout << "VectorI (3s) : " << VectorI.transpose() << endl << endl;

		cout << "Local constructor from size" << endl;
		StateOrientation<EULER> orientation3D_a;
		orientation3D_a.print();

		cout << "Local constructor from VectorH" << endl;
		StateOrientation<EULER> orientation3D_b(VectorH);
		orientation3D_b.print();

		cout << "Local constructor from VectorI" << endl;
		StateOrientation<EULER> orientation3D_c(VectorI);
		orientation3D_c.print();

		cout << "Local copy constructor" << endl;
		StateOrientation<EULER> orientation3D_d(orientation3D_c);
		orientation3D_d.print();

		cout << "Remote constructor from size" << endl;
		StateOrientation<EULER> orientation3D_e(storage, index);
		orientation3D_e.print();

		cout << "Remote constructor from VectorH" << endl;
		StateOrientation<EULER> orientation3D_f(storage, index, VectorH);
		orientation3D_f.print();

		cout << "Remote constructor from VectorI" << endl;
		StateOrientation<EULER> orientation3D_g(storage, index, VectorI);
		orientation3D_g.print();

		cout << "Remap to idx 4" << endl;
		orientation3D_g.remap(storage, 4);
		orientation3D_g.print();

		cout << "storage   : " << storage.transpose() << endl;

		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "--StateOrientation<QUATERNION> CONSTRUCTORS-" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 0;
		VectorXs VectorJ(4);
		VectorJ << 1.1, 2.2, 3.3, 4.4;
		Vector4s VectorK;
		VectorK << 5.5, 6.6, 7.7, 8.8;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "VectorJ (Xs) : " << VectorJ.transpose() << endl;
		cout << "VectorK (4s) : " << VectorK.transpose() << endl << endl;

		cout << "Local constructor from size" << endl;
		StateOrientation<QUATERNION> orientation3Dq_a;
		orientation3Dq_a.print();

		cout << "Local constructor from VectorJ" << endl;
		StateOrientation<QUATERNION> orientation3Dq_b(VectorJ);
		orientation3Dq_b.print();

		cout << "Local constructor from VectorK" << endl;
		StateOrientation<QUATERNION> orientation3Dq_c(VectorK);
		orientation3Dq_c.print();

		cout << "Local copy constructor" << endl;
		StateOrientation<QUATERNION> orientation3Dq_d(orientation3Dq_c);
		orientation3Dq_d.print();

		cout << "Remote constructor from size" << endl;
		StateOrientation<QUATERNION> orientation3Dq_e(storage, index);
		orientation3Dq_e.print();

		cout << "Remote constructor from VectorJ" << endl;
		StateOrientation<QUATERNION> orientation3Dq_f(storage, index, VectorJ);
		orientation3Dq_f.print();

		cout << "Remote constructor from VectorK" << endl;
		StateOrientation<QUATERNION> orientation3Dq_g(storage, index, VectorK);
		orientation3Dq_g.print();

		cout << "Remap to idx 4" << endl;
		orientation3Dq_g.remap(storage, 4);
		orientation3Dq_g.print();

		cout << "storage   : " << storage.transpose() << endl;
    }
    if(test_operators)
    {
		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "----------------- OPERATORS ----------------" << endl;

		cout << "----------------- THETA - Local:" << endl;

		VectorXs VectorF2(1);
		VectorF2 << 0.7;
		StateOrientation<THETA> o2D_a(VectorF2);
		cout << "o1" << endl;
		o2D_a.print();
		VectorF2 << -1.2;
		StateOrientation<THETA> o2D_b(VectorF2);
		cout << "o2" << endl;
		o2D_b.print();

		cout << "o3 = o1 * o2" << endl;
		StateOrientation<THETA> o2D_c = o2D_a * o2D_b;
		o2D_c.print();

		cout << "o1 *= o2" << endl;
		o2D_a *= o2D_b;
		o2D_a.print();

		cout << "o4 = o1.inverse()" << endl;
		StateOrientation<THETA> o2D_d = o2D_a.inverse();
		o2D_d.print();

		cout << "o5 = o1 / o2" << endl;
		StateOrientation<THETA> o2D_e = o2D_a / o2D_b;
		o2D_e.print();

		cout << "o6 = o1 * o2.inverse()" << endl;
		StateOrientation<THETA> o2D_f = o2D_a * o2D_b.inverse();
		o2D_f.print();

		cout << "o1 /= o2" << endl;
		o2D_a /= o2D_b;
		o2D_a.print();

		cout << "----------------- THETA - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "o7" << endl;
		StateOrientation<THETA> o2D_g(storage, 0);
		o2D_g.print();

		cout << "o8 from vector" << endl;
		StateOrientation<THETA> o2D_h(storage, 3, VectorF2);
		o2D_h.print();

		cout << "o7 *= o8" << endl;
		o2D_g *= o2D_h;
		o2D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o8.makeInverse()" << endl;
		o2D_h.makeInverse();
		o2D_h.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o7 *= o8" << endl;
		o2D_g *= o2D_h;
		o2D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "----------------- EULER - Local:" << endl;
		VectorXs VectorF3(3);
		VectorF3 << 1.1, 2.2, 3.3;

		StateOrientation<EULER> o3D_a(VectorF3);
		cout << "o1" << endl;
		o3D_a.print();
		VectorF3 << -1.2, 0.2, 2.3;
		StateOrientation<EULER> o3D_b(VectorF3);
		cout << "o2" << endl;
		o3D_b.print();

		cout << "o3 = o1 * o2" << endl;
		StateOrientation<EULER> o3D_c = o3D_a * o3D_b;
		o3D_c.print();

		cout << "o1 *= o2" << endl;
		o3D_a *= o3D_b;
		o3D_a.print();

		cout << "o4 = (o1.inverse()).inverse()" << endl;
		StateOrientation<EULER> o3D_d = (o3D_a.inverse()).inverse();
		o3D_d.print();

		cout << "o5 = o1 / o2" << endl;
		StateOrientation<EULER> o3D_e = o3D_a / o3D_b;
		o3D_e.print();

		cout << "o6 = o1 * o2.inverse()" << endl;
		StateOrientation<EULER> o3D_f = o3D_a * o3D_b.inverse();
		o3D_f.print();

		cout << "o1 /= o2" << endl;
		o3D_a /= o3D_b;
		o3D_a.print();

		cout << "----------------- EULER - Remote:" << endl;
		storage << 0.1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "o7" << endl;
		StateOrientation<EULER> o3D_g(storage, 0);
		o3D_g.print();

		cout << "o8 from vector" << endl;
		StateOrientation<EULER> o3D_h(storage, 3, VectorF3);
		o3D_h.print();

		cout << "o7 *= o8" << endl;
		o3D_g *= o3D_h;
		o3D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o8.makeInverse()" << endl;
		o3D_h.makeInverse();
		o3D_h.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o7 *= o8" << endl;
		o3D_g *= o3D_h;
		o3D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o7.normalize()" << endl;
		o3D_g.normalize();
		o3D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "----------------- QUATERNION - Local:" << endl;
		VectorXs VectorF4(4);
		VectorF4 << 1.1, 2.2, 3.3, 2.1;

		StateOrientation<QUATERNION> o3Dq_a(VectorF4);
		cout << "o1" << endl;
		o3Dq_a.print();
		VectorF4 << -1.2, 0.2, 2.3, 0.9;
		StateOrientation<QUATERNION> o3Dq_b(VectorF4);
		cout << "o2" << endl;
		o3Dq_b.print();

		cout << "o3 = o1 * o2" << endl;
		StateOrientation<QUATERNION> o3Dq_c = o3Dq_a * o3Dq_b;
		o3Dq_c.print();

		cout << "o1 *= o2" << endl;
		o3Dq_a *= o3Dq_b;
		o3Dq_a.print();
		o3Dq_b.print();

		cout << "o4 = (o1.inverse()).inverse()" << endl;
		StateOrientation<QUATERNION> o3Dq_d = (o3Dq_a.inverse()).inverse();
		o3Dq_d.print();

		cout << "o5 = o1 / o2" << endl;
		StateOrientation<QUATERNION> o3Dq_e = o3Dq_a / o3Dq_b;
		o3Dq_e.print();

		cout << "o6 = o1 * o2.inverse()" << endl;
		StateOrientation<QUATERNION> o3Dq_f = o3Dq_a * o3Dq_b.inverse();
		o3Dq_f.print();

		cout << "o1 /= o2" << endl;
		o3Dq_a /= o3Dq_b;
		o3Dq_a.print();

		cout << "----------------- QUATERNION - Remote:" << endl;
		storage << 0.1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "o7" << endl;
		StateOrientation<QUATERNION> o3Dq_g(storage, 0);
		o3Dq_g.print();
		cout << "o7.normalize()" << endl;
		o3Dq_g.normalize();
		o3Dq_g.print();

		cout << "o8 from vector" << endl;
		StateOrientation<QUATERNION> o3Dq_h(storage, 4, VectorF4);
		o3Dq_h.print();

		cout << "o7 *= o8" << endl;
		o3Dq_g *= o3Dq_h;
		o3Dq_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o8.makeInverse()" << endl;
		o3Dq_h.makeInverse();
		o3Dq_h.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o7 *= o8" << endl;
		o3Dq_g *= o3Dq_h;
		o3Dq_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o7.normalize()" << endl;
		o3Dq_g.normalize();
		o3Dq_g.print();
		cout << "storage   : " << storage.transpose() << endl;
    }
    if(test_operators2)
    {
		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "----------------- OPERATORS ----------------" << endl;

		cout << "----------------- POINT - Local:" << endl;

		VectorXs VectorF5(2);
		VectorF5 << 0.7, 5.1;
		StatePoint p2D_a(VectorF5);
		cout << "p1" << endl;
		p2D_a.print();
		VectorF5 << -1.2, 6.4;
		StatePoint p2D_b(VectorF5);
		cout << "p2" << endl;
		p2D_b.print();

		cout << "p3 = p1 + p2" << endl;
		StatePoint p2D_c = p2D_a + p2D_b;
		p2D_c.print();

		cout << "p1 += p2" << endl;
		p2D_a += p2D_b;
		p2D_a.print();

		cout << "p4 = -p1" << endl;
		StatePoint p2D_d = -p2D_a;
		p2D_d.print();

		cout << "p5 = p1 - p2" << endl;
		StatePoint p2D_e = p2D_a - p2D_b;
		p2D_e.print();

		cout << "p6 = p1 + (-p2)" << endl;
		StatePoint p2D_f = p2D_a + (-p2D_b);
		p2D_f.print();

		cout << "p1 -= p2" << endl;
		p2D_a -= p2D_b;
		p2D_a.print();

		cout << "----------------- POINT - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "p7" << endl;
		StatePoint p2D_g(storage, 0, 2);
		p2D_g.print();

		cout << "p8 from vector" << endl;
		StatePoint p2D_h(storage, 3, VectorF5);
		p2D_h.print();

		cout << "p7 += p8" << endl;
		p2D_g += p2D_h;
		p2D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p8.makeOpposite()" << endl;
		p2D_h.makeOpposite();
		p2D_h.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p7 += p8" << endl;
		p2D_g += p2D_h;
		p2D_g.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "----------------- Point 2D - Orientation<THETA> Local:" << endl;

		VectorXs VectorF6(2);
		VectorF6 << 0.7, 5.1;
		StatePoint p2D_i(VectorF6);
		cout << "p1" << endl;
		p2D_i.print();
		VectorXs VectorF7(1);
		VectorF7 << M_PI/2;
		StateOrientation<THETA> oTH_a(VectorF7);
		cout << "o1" << endl;
		oTH_a.print();

		cout << "p2 = o1 * p1" << endl;
		StatePoint p2D_j = oTH_a * p2D_i;
		p2D_j.print();

		cout << "p3 = o1.inverse() * (o1 * p1)" << endl;
		StatePoint p2D_k = oTH_a.inverse() * (oTH_a * p2D_i);
		p2D_k.print();

		cout << "p4 = (o1.inverse() * o1) * p1" << endl;
		StatePoint p2D_l = (oTH_a.inverse() * oTH_a) * p2D_i;
		p2D_l.print();

		cout << "----------------- Point 2D - Orientation<THETA> - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "p5" << endl;
		StatePoint p2D_m(storage, 0, 2);
		p2D_m.print();

		cout << "o2 from vector" << endl;
		StateOrientation<THETA> oTH_b(storage, 2, VectorF7);
		oTH_b.print();

		cout << "p5 *= o2" << endl;
		p2D_m *= oTH_b;
		p2D_m.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o2.makeInverse()" << endl;
		oTH_b.makeInverse();
		oTH_b.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p5 *= o2" << endl;
		p2D_m *= oTH_b;
		p2D_m.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "----------------- Point 3D - Orientation<THETA> Local:" << endl;

		VectorXs VectorF8(3);
		VectorF8 << 0.7, 5.1, 8.3;
		StatePoint p3D_a(VectorF8);
		cout << "p1" << endl;
		p3D_a.print();
		cout << "o1" << endl;
		oTH_a.print();

		cout << "p2 = o1 * p1" << endl;
		StatePoint p3D_b = oTH_a * p3D_a;
		p3D_b.print();

		cout << "p3 = o1.inverse() * (o1 * p1)" << endl;
		StatePoint p3D_c = oTH_a.inverse() * (oTH_a * p3D_a);
		p3D_c.print();

		cout << "p4 = (o1.inverse() * o1) * p1" << endl;
		StatePoint p3D_d = (oTH_a.inverse() * oTH_a) * p3D_a;
		p3D_d.print();

		cout << "----------------- Point 3D - Orientation<THETA> - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "p5" << endl;
		StatePoint p3D_e(storage, 0, 3);
		p3D_e.print();

		cout << "o2 from vector" << endl;
		StateOrientation<THETA> oTH_c(storage, 3, VectorF7);
		oTH_c.print();

		cout << "p5 *= o2" << endl;
		p3D_e *= oTH_c;
		p3D_e.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o2.makeInverse()" << endl;
		oTH_c.makeInverse();
		oTH_c.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p5 *= o2" << endl;
		p3D_e *= oTH_c;
		p3D_e.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "----------------- Point 3D - Orientation<EULER> Local:" << endl;

		cout << "p1" << endl;
		p3D_a.print();
		VectorXs VectorF9(3);
		VectorF9 << M_PI/2, M_PI/2, 0.0;
		StateOrientation<EULER> oEU_a(VectorF9);
		cout << "o1" << endl;
		oEU_a.print();

		cout << "p2 = o1 * p1" << endl;
		StatePoint p3D_f = oEU_a * p3D_a;
		p3D_f.print();

		cout << "p3 = o1.inverse() * (o1 * p1)" << endl;
		StatePoint p3D_g = oEU_a.inverse() * (oEU_a * p3D_a);
		p3D_g.print();

		cout << "p4 = (o1.inverse() * o1) * p1" << endl;
		StatePoint p3D_h = (oEU_a.inverse() * oEU_a) * p3D_a;
		p3D_h.print();

		cout << "----------------- Point 3D - Orientation<EULER> - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "p5" << endl;
		StatePoint p3D_i(storage, 0, 3);
		p3D_i.print();

		cout << "o2 from vector" << endl;
		StateOrientation<EULER> oEU_b(storage, 3, VectorF9);
		oEU_b.print();

		cout << "p5 *= o2" << endl;
		p3D_i *= oEU_b;
		p3D_i.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o2.makeInverse()" << endl;
		oEU_b.makeInverse();
		oEU_b.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p5 *= o2" << endl;
		p3D_i *= oEU_b;
		p3D_i.print();
		cout << "storage   : " << storage.transpose() << endl;



		cout << "----------------- Point 3D - Orientation<QUATERNION> Local:" << endl;

		cout << "p1" << endl;
		p3D_a.print();
		VectorXs VectorF10(4);
		VectorF10 = Quaternions(Matrix3s(oEU_a.getRotationMatrix())).coeffs();
		StateOrientation<QUATERNION> oQU_a(VectorF10);
		cout << "o1" << endl;
		oQU_a.print();

		cout << "p2 = o1 * p1" << endl;
		StatePoint p3D_j = oQU_a * p3D_a;
		p3D_j.print();

		cout << "p3 = o1.inverse() * (o1 * p1)" << endl;
		StatePoint p3D_k = oQU_a.inverse() * (oQU_a * p3D_a);
		p3D_k.print();

		cout << "p4 = (o1.inverse() * o1) * p1" << endl;
		StatePoint p3D_l = (oQU_a.inverse() * oQU_a) * p3D_a;
		p3D_l.print();

		cout << "----------------- Point 3D - Orientation<QUATERNION> - Remote:" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		cout << "storage   : " << storage.transpose() << endl << endl;
		cout << "p5" << endl;
		StatePoint p3D_m(storage, 0, 3);
		p3D_m.print();

		cout << "o2 from vector" << endl;
		StateOrientation<QUATERNION> oQU_b(storage, 3, VectorF10);
		oQU_b.print();

		cout << "p5 *= o2" << endl;
		p3D_m *= oQU_b;
		p3D_m.print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "o2" << endl;
		oQU_b.print();
		cout << "o2.makeInverse()" << endl;
		oQU_b.makeInverse();
		oQU_b.print();
		cout << "o2.Inverse()" << endl;
		oQU_b.inverse().print();
		cout << "storage   : " << storage.transpose() << endl;

		cout << "p5 *= o2" << endl;
		p3D_m *= oQU_b;
		p3D_m.print();
		cout << "storage   : " << storage.transpose() << endl;
    }
    cout << "end" << endl;
    if (test_po2)
	{
    	cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "-------StatePO<2D,THETA> CONSTRUCTORS--------" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 0;
		VectorXs VectorDynamic(4);
		VectorDynamic << 1.1, 2.2, 0.9, 0.1;
		Vector4s VectorStatic;
		VectorStatic << 5.5, 6.6, 7.7, 0.8;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "VectorDynamic : " << VectorDynamic.transpose() << endl;
		cout << "VectorStatic  : " << VectorStatic.transpose() << endl << endl;

		cout << "Local constructor" << endl;
		StatePO<2,THETA> pose2D_a;
		pose2D_a.print();

		cout << "Local constructor from size" << endl;
		StatePO<2,THETA> pose2D_a2(6);
		pose2D_a2.print();

		cout << "Local constructor from VectorDynamic" << endl;
		StatePO<2,THETA> pose2D_b(VectorDynamic);
		pose2D_b.print();

		cout << "Local constructor from VectorStatic" << endl;
		StatePO<2,THETA> pose2D_c(VectorStatic);
		pose2D_c.print();

		cout << "Local constructor from StatePoint and StateOrientation" << endl;
		StatePoint p_aux(VectorStatic.head(2));
		StateOrientation<THETA> o_aux(VectorStatic.tail(2));
		StatePO<2,THETA> pose2D_c2(p_aux, o_aux);
		pose2D_c2.print();

		cout << "Local copy constructor" << endl;
		StatePO<2,THETA> pose2D_d(pose2D_c);
		pose2D_d.print();

		cout << "Remote constructor from size" << endl;
		StatePO<2,THETA> pose2D_e(storage, index, 4);
		pose2D_e.print();

		cout << "Remote constructor from VectorDynamic" << endl;
		StatePO<2,THETA> pose2D_f(storage, index, VectorDynamic);
		pose2D_f.print();

		cout << "Remote constructor from VectorStatic" << endl;
		StatePO<2,THETA> pose2D_g(storage, index, VectorStatic);
		pose2D_g.print();

		cout << "Remap to idx 4" << endl;
		pose2D_g.remap(storage, 4);
		pose2D_g.print();

		cout << "storage   : " << storage.transpose() << endl;
	}
    if (test_po3)
    {
		cout << endl;
		cout << "--------------------------------------------" << endl;
		cout << "-------StatePO<3,EULER> CONSTRUCTORS--------" << endl;
		storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
		index = 0;
		VectorXs VectorDynamic(6);
		VectorDynamic << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
		Matrix<WolfScalar, 6, 1> VectorStatic;
		VectorStatic << 4.4, 5.5, 6.6, 7.7, 8.8, 9.9;
		cout << "storage   : " << storage.transpose() << endl;
		cout << "index     : " << index << endl;
		cout << "VectorDynamic : " << VectorDynamic.transpose() << endl;
		cout << "VectorStatic  : " << VectorStatic.transpose() << endl << endl;

		cout << "Local constructor" << endl;
		StatePO<3,EULER> pose3D_a;
		pose3D_a.print();

		cout << "Local constructor from size" << endl;
		StatePO<3,EULER> pose3D_a2(6);
		pose3D_a2.print();

		cout << "Local constructor from VectorDynamic" << endl;
		StatePO<3,EULER> pose3D_b(VectorDynamic);
		pose3D_b.print();

		cout << "Local constructor from VectorStatic" << endl;
		StatePO<3,EULER> pose3D_c(VectorStatic);
		pose3D_c.print();

		cout << "Local constructor from StatePoint and StateOrientation" << endl;
		StatePoint p_aux(VectorStatic.head(3));
		StateOrientation<EULER> o_aux(VectorStatic.tail(3));
		StatePO<3,EULER> pose3D_c2(p_aux, o_aux);
		pose3D_c2.print();

		cout << "Local copy constructor" << endl;
		StatePO<3,EULER> pose3D_d(pose3D_c);
		pose3D_d.print();

		cout << "Remote constructor from size" << endl;
		StatePO<3,EULER> pose3D_e(storage, index, 6);
		pose3D_e.print();

		cout << "Remote constructor from VectorDynamic" << endl;
		StatePO<3,EULER> pose3D_f(storage, index, VectorDynamic);
		pose3D_f.print();

		cout << "Remote constructor from VectorStatic" << endl;
		StatePO<3,EULER> pose3D_g(storage, index, VectorStatic);
		pose3D_g.print();

		cout << "Remap to idx 4" << endl;
		pose3D_g.remap(storage, 4);
		pose3D_g.print();

		cout << "storage   : " << storage.transpose() << endl;
	}
    return 0;
}

