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
	bool test_operators= 1;
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

