//std
#include <iostream>

//Pinocchio headers
#include <math/sincos.hpp>
#include <spatial/se3.hpp>

//Wolf
#include "wolf.h"

int main()
{
    using namespace wolf;
    using namespace se3;

    std::cout << std::endl << "Pinocchio Spatial test" << std::endl;

    //create random SE3Base object
    Eigen::Matrix3d Mat3d;
    Mat3d = Eigen::MatrixXd::Random(3,3);
    Eigen::Vector3d Vec3d;
    Vec3d << 6, 3, 1;

    Eigen::Vector3d Vec3d_op;
    Vec3d_op << 5, 4, 9;
    Eigen::Vector3d Vec3d_op1;
    Vec3d_op1 << 0, 0, 0;

    se3::SE3 Se3_Object(Mat3d, Vec3d);

    std::cout << "Initial Se3 Object : \n" << std::endl;
    Se3_Object.disp_impl(std::cout);

    //realise a pure translation
    se3::SE3 operation_matrix(Eigen::Matrix3d::Identity(3,3), Vec3d_op);
    Se3_Object=Se3_Object.act(operation_matrix); //left hand operation

    std::cout << "procedeed to B=A*B with A = \n" << std::endl;
    operation_matrix.disp_impl(std::cout);
    std::cout << "\n result is : B = " << std::endl;
    Se3_Object.disp_impl(std::cout);

    //do a pure rotation
    se3::SE3 operation_matrix2(Eigen::Matrix3d::Random(3,3), Vec3d_op1);
    Se3_Object=Se3_Object.act(operation_matrix2); //left hand operation

    std::cout << "procedeed to B=A*B with A = \n" << std::endl;
    operation_matrix2.disp_impl(std::cout);
    std::cout << "\n result is : B = " << std::endl;
    Se3_Object.disp_impl(std::cout);

    //get an homogeneous matrix from SE3 Object :
    Eigen::Matrix4d homogeneous_matrix = Se3_Object.toHomogeneousMatrix();
    std::cout << "\n Corresponding homogeneous matrix : \n" << homogeneous_matrix << std::endl;

    Scalar q1[4];
    Eigen::Map<Eigen::Quaternions> q1_map(q1);
    q1_map.w() = 1;
    q1_map.x() = 2;
    q1_map.y() = 3;
    q1_map.z() = 4;

//    Eigen::Matrix3d rotation0;
//    rotation0 = Eigen::AngleAxisd(q1_map); //get a rotation matrix from a quaternion
//    //So here is the way to create a SE3 Objectusing quaternions
//    //Rotation + translation
//    se3::SE3 operation_matrix3(rotation0, Vec3d_op);
//    Se3_Object=Se3_Object.act(operation_matrix3); //left hand operation

//    std::cout << "\n rotation from quaternion + translation"<< std::endl;
//    operation_matrix3.disp_impl(std::cout);
//    std::cout << "\n result is : B = " << std::endl;
//    Se3_Object.disp_impl(std::cout);

    return 0;
}
