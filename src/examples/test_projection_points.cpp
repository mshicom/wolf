#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//std includes
#include <iostream>

//wolf includes
#include "pinholeTools.h"


int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << " ========= ProjectPoints test ===========" << std::endl << std::endl;

    cv::Point3f points3D;
    points3D.x = 2.0;
    points3D.y = 5.0;
    points3D.z = 6.0;
    std::vector<cv::Point3f> point_in_3D;
    point_in_3D.push_back(points3D);
    points3D.x = 4.0;
    points3D.y = 2.0;
    points3D.z = 1.0;
    point_in_3D.push_back(points3D);

    std::vector<float> rot_mat = {0,0,0};
    std::vector<float> trans_mat = {1,1,1};

    cv::Mat cam_mat(3,3,CV_32F);
    cam_mat.row(0).col(0).setTo(1);
    cam_mat.row(0).col(1).setTo(0);
    cam_mat.row(0).col(2).setTo(2);
    cam_mat.row(1).col(0).setTo(0);
    cam_mat.row(1).col(1).setTo(1);
    cam_mat.row(1).col(2).setTo(2);
    cam_mat.row(2).col(0).setTo(0);
    cam_mat.row(2).col(1).setTo(0);
    cam_mat.row(2).col(2).setTo(1);

    std::cout << "cam_mat[1,2]: " << cam_mat.row(1).col(0) << std::endl;

    std::vector<float> dist_coef = {0,0,0,0,0};
    std::vector<cv::Point2f> points2D;
    cv::projectPoints(point_in_3D,rot_mat,trans_mat,cam_mat,dist_coef,points2D);

    for (auto it : points2D)
    {
        std::cout << "points2D- X: " << it.x << "; Y: " << it.y << std::endl;
    }

    std::cout << std::endl << " ========= PinholeTools DUALITY TEST ===========" << std::endl << std::endl;

    //================================= projectPoint and backprojectPoint to/from NormalizedPlane

    Eigen::Vector3s project_point_normalized_test;
    project_point_normalized_test[0] = 1.06065;
    project_point_normalized_test[1] = 1.06065;
    project_point_normalized_test[2] = 3;
    Eigen::Vector2s project_point_normalized_output;
    Eigen::Vector2s project_point_normalized_output2;
    Scalar project_point_normalized_dist;

    Scalar backproject_point_normalized_depth = 3;
    Eigen::Vector3s backproject_point_normalized_output;

    project_point_normalized_output = pinhole::projectPointToNormalizedPlane(project_point_normalized_test);
    pinhole::projectPointToNormalizedPlane(project_point_normalized_test,project_point_normalized_output2,project_point_normalized_dist);

    backproject_point_normalized_output =
            pinhole::backprojectPointFromNormalizedPlane(project_point_normalized_output,backproject_point_normalized_depth);


    std::cout << "TEST project and backproject PointToNormalizedPlane" << std::endl;
    std::cout << std:: endl << "Original" << std::endl;
    std::cout << "x: " << project_point_normalized_test[0] << "; y: " << project_point_normalized_test[1]
              << "; z: " << project_point_normalized_test[2] << std::endl;
    std::cout << std:: endl << "Project" << std::endl;
    std::cout << "x: " << project_point_normalized_output[0] << "; y: " << project_point_normalized_output[1]
              << "; rows: " << project_point_normalized_output.rows() << "; cols: " << project_point_normalized_output.cols()
              << std::endl;
    std::cout << std:: endl << "Alternate project" << std::endl;
    std::cout << "x: " << project_point_normalized_output[0] << "; y: " << project_point_normalized_output[1]
              << "; rows: " << project_point_normalized_output.rows() << "; cols: " << project_point_normalized_output.cols()
              << "; distance: " << project_point_normalized_dist << std::endl;
    std::cout << std:: endl << "Backproject" << std::endl;
    std::cout << "x: " << backproject_point_normalized_output[0] << "; y: " << backproject_point_normalized_output[1]
              << "; z: " << backproject_point_normalized_output[2] << "; depth: " << backproject_point_normalized_depth << std::endl;


    //================================= projectPoint and backprojectPoint to/from NormalizedPlane WITH JACOBIANS

    Eigen::Vector3s pp_normalized_test;
    pp_normalized_test[0] = 3;
    pp_normalized_test[1] = 3;
    pp_normalized_test[2] = 3;
    Eigen::Vector2s pp_normalized_output;
    Eigen::Vector2s pp_normalized_output2;
    Eigen::Matrix3s pp_normalized_jacobian;
    Eigen::Matrix3s pp_normalized_jacobian2;
    Scalar pp_normalized_distance;

    Scalar bpp_normalized_depth = 3;
    Eigen::Vector3s bpp_normalized_output;
    Eigen::Matrix3s bpp_normalized_jacobian;
    Eigen::Vector3s bpp_normalized_jacobian_depth;


    pinhole::projectPointToNormalizedPlane(pp_normalized_test,pp_normalized_output,pp_normalized_jacobian);
    pinhole::projectPointToNormalizedPlane(pp_normalized_test,pp_normalized_output2,pp_normalized_distance,pp_normalized_jacobian2);

    pinhole::backprojectPointFromNormalizedPlane(pp_normalized_output,bpp_normalized_depth,
                                                 bpp_normalized_output,bpp_normalized_jacobian,bpp_normalized_jacobian_depth);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << "\nTEST project and backproject PointToNormalizedPlane with JACOBIAN" << std::endl;

    std::cout << std:: endl << "Original" << std::endl;
    std::cout << "x: " << pp_normalized_test[0] << "; y: " << pp_normalized_test[1] << "; z: " << pp_normalized_test[2] << std::endl;
    std::cout << std:: endl << "Project" << std::endl;
    std::cout << "x: " << pp_normalized_output[0] << "; y: " << pp_normalized_output[1] << "; rows: " << pp_normalized_output.rows()
              << "; cols: " << pp_normalized_output.cols() << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 pp_normalized_jacobian.row(0).col(0) << " " << pp_normalized_jacobian.row(0).col(1) << " " << pp_normalized_jacobian.row(0).col(2) << " " << std::endl <<
                 pp_normalized_jacobian.row(1).col(0) << " " << pp_normalized_jacobian.row(1).col(1) << " " << pp_normalized_jacobian.row(1).col(2) << " " << std::endl <<
                 pp_normalized_jacobian.row(2).col(0) << " " << pp_normalized_jacobian.row(2).col(1) << " " << pp_normalized_jacobian.row(2).col(2) << " " << std::endl;

    std::cout << std:: endl << "Alternate project" << std::endl;
    std::cout << "x: " << pp_normalized_output2[0] << "; y: " << pp_normalized_output2[1] << "; rows: "
              << pp_normalized_output2.rows() << "; cols: " << pp_normalized_output2.cols() << "; distance: "
              << pp_normalized_distance << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 pp_normalized_jacobian2.row(0).col(0) << " " << pp_normalized_jacobian2.row(0).col(1) << " " << pp_normalized_jacobian2.row(0).col(2) << " " << std::endl <<
                 pp_normalized_jacobian2.row(1).col(0) << " " << pp_normalized_jacobian2.row(1).col(1) << " " << pp_normalized_jacobian2.row(1).col(2) << " " << std::endl <<
                 pp_normalized_jacobian2.row(2).col(0) << " " << pp_normalized_jacobian2.row(2).col(1) << " " << pp_normalized_jacobian2.row(2).col(2) << " " << std::endl;

    std::cout << std:: endl << "Backproject" << std::endl;
    std::cout << "x: " << bpp_normalized_output[0] << "; y: " << bpp_normalized_output[1] << "; z: " << bpp_normalized_output[2]
              << "; depth: " << bpp_normalized_depth << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 bpp_normalized_jacobian.row(0).col(0) << " " << bpp_normalized_jacobian.row(0).col(1) << " " << bpp_normalized_jacobian.row(0).col(2) << " " << std::endl <<
                 bpp_normalized_jacobian.row(1).col(0) << " " << bpp_normalized_jacobian.row(1).col(1) << " " << bpp_normalized_jacobian.row(1).col(2) << " " << std::endl <<
                 bpp_normalized_jacobian.row(2).col(0) << " " << bpp_normalized_jacobian.row(2).col(1) << " " << bpp_normalized_jacobian.row(2).col(2) << " " << std::endl;
    std::cout << "\n--> Jacobian - depth" << std::endl <<
                 bpp_normalized_jacobian_depth[0] << " " << bpp_normalized_jacobian_depth[1] << " " << bpp_normalized_jacobian_depth[2] << " " << std::endl;


    Eigen::Matrix3s test_jacobian;
    test_jacobian =  pp_normalized_jacobian * bpp_normalized_jacobian;

    std::cout << "\n\n\n==> Jacobian Testing" << std::endl <<
                 test_jacobian.row(0).col(0) << " " << test_jacobian.row(0).col(1) << " " << test_jacobian.row(0).col(2) << " " << std::endl <<
                 test_jacobian.row(1).col(0) << " " << test_jacobian.row(1).col(1) << " " << test_jacobian.row(1).col(2) << " " << std::endl <<
                 test_jacobian.row(2).col(0) << " " << test_jacobian.row(2).col(1) << " " << test_jacobian.row(2).col(2) << " " << std::endl;


    //================================= IsInRoi / IsInImage

    Eigen::Vector2s pix;
    pix[0] = 40; // x
    pix[1] = 40; // y

    int roi_x = 30;
    int roi_y = 30;
    int roi_width = 20;
    int roi_height = 20;

    int image_width = 640;
    int image_height = 480;

    bool is_in_roi;
    bool is_in_image;
    is_in_roi = pinhole::isInRoi(pix,roi_x,roi_y,roi_width,roi_height);
    is_in_image = pinhole::isInImage(pix,image_width,image_height);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << "\nTEST isInRoi/isInImage" << std::endl;
    std::cout << std::endl << "Pixel " << std::endl;
    std::cout << "x: " << pix[0] << "; y: " << pix[1] << std::endl;
    std::cout << std::endl << "ROI " << std::endl;
    std::cout << "x: " << roi_x << "; y: " << roi_y << "; width: " << roi_width << "; height: " << roi_height << std::endl;
    std::cout << "is_in_roi: " << is_in_roi << std::endl;
    std::cout << std::endl << "Image " << std::endl;
    std::cout << "width: " << image_width << "; height: " << image_height << std::endl;
    std::cout << "is_in_image: " << is_in_image << std::endl;



    //================================= computeCorrectionModel

    Eigen::Vector2s distortion2;
    distortion2[0] = -0.301701;
    distortion2[1] = 0.0963189;
    Eigen::Vector4f k_test2;
    //k = [u0, v0, au, av]
    k_test2[0] = 516.686; //u0
    k_test2[1] = 355.129; //v0
    k_test2[2] = 991.852; //au
    k_test2[3] = 995.269; //av

    Eigen::Vector2s correction_test2;
    pinhole::computeCorrectionModel(k_test2,distortion2,correction_test2);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << "\nTEST computeCorrectionModel" << std::endl;
    std::cout << std::endl << "distortion" << std::endl;
    std::cout << "d1: " << distortion2[0] << "; d2: " << distortion2[1] << std::endl;
    std::cout << std::endl << "k values" << std::endl;
    std::cout << "u0: " << k_test2[0] << "; v0: " << k_test2[1] << "; au: " << k_test2[2] << "; av: " << k_test2[3] << std::endl;
    std::cout << std::endl << "correction" << std::endl;
    std::cout << "c1: " << correction_test2[0] << "; c2: " << correction_test2[1] << std::endl;



    //================================= distortPoint


    Eigen::Vector2s distorting_point;
    distorting_point[0] = 0.35355;
    distorting_point[1] = 0.35355;

    Eigen::Vector2s distored_point3;
    distored_point3 = pinhole::distortPoint(distortion2,distorting_point);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << "\nTEST distortPoint" << std::endl;
    std::cout << std::endl << "Point to be distorted" << std::endl;
    std::cout << "x: " << distorting_point[0] << "; y: " << distorting_point[1] << std::endl;
    std::cout << std::endl << "Distorted point" << std::endl;
    std::cout << "x: " << distored_point3[0] << "; y: " << distored_point3[1] << std::endl;

    Eigen::Vector2s corrected_point4;
    corrected_point4 = pinhole::undistortPoint(correction_test2,distored_point3);
    std::cout << std::endl << "Corrected point" << std::endl;
    std::cout << "x: " << corrected_point4[0] << "; y: " << corrected_point4[1] << std::endl;

    ////

    Eigen::Vector2s distored_point4;
    Eigen::Matrix2s distortion_jacobian2;
    pinhole::distortPoint(distortion2,distorting_point,distored_point4,distortion_jacobian2);

    std::cout << "\n\nTEST distortPoint, jacobian" << std::endl;
    std::cout << std::endl << "Point to be distorted" << std::endl;
    std::cout << "x: " << distorting_point[0] << "; y: " << distorting_point[1] << std::endl;
    std::cout << std::endl << "Distorted point" << std::endl;
    std::cout << "x: " << distored_point4[0] << "; y: " << distored_point4[1] << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 distortion_jacobian2.row(0).col(0) << " " << distortion_jacobian2.row(0).col(1) << std::endl <<
                 distortion_jacobian2.row(1).col(0) << " " << distortion_jacobian2.row(1).col(1) << std::endl;



    Eigen::Vector2s corrected_point5;
    Eigen::Matrix2s corrected_jacobian2;
    pinhole::undistortPoint(correction_test2,distored_point4,corrected_point5,corrected_jacobian2);

    std::cout << std::endl << "Corrected point" << std::endl;
    std::cout << "x: " << corrected_point5[0] << "; y: " << corrected_point5[1] << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 corrected_jacobian2.row(0).col(0) << " " << corrected_jacobian2.row(0).col(1) << std::endl <<
                 corrected_jacobian2.row(1).col(0) << " " << corrected_jacobian2.row(1).col(1) << std::endl;

    Eigen::Matrix2s test_jacobian_distortion;
    test_jacobian_distortion =  distortion_jacobian2 * corrected_jacobian2;

    std::cout << "\n\n\n==> Jacobian Testing" << std::endl <<
                 test_jacobian_distortion.row(0).col(0) << " " << test_jacobian_distortion.row(0).col(1) << std::endl <<
                 test_jacobian_distortion.row(1).col(0) << " " << test_jacobian_distortion.row(1).col(1) << std::endl;

    //================================= PixelizePoint

    Eigen::Vector2s pixelize_ud;
    pixelize_ud[0] = 45;
    pixelize_ud[1] = 28;

    Eigen::Vector2s pixelize_output3;
    pixelize_output3 = pinhole::pixellizePoint(k_test2,pixelize_ud);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << "\nTEST pixelizePoint; Eigen::Vector2s" << std::endl;
    std::cout << std::endl << "Original" << std::endl;
    std::cout << "x: " << pixelize_ud[0] << "; y: " << pixelize_ud[1] << std::endl;
    std::cout << std::endl << "Pixelized" << std::endl;
    std::cout << "x: " << pixelize_output3[0] << "; y: " << pixelize_output3[1] << std::endl;

    Eigen::Vector2s depixelize_output3;
    depixelize_output3 = pinhole::depixellizePoint(k_test2,pixelize_output3);
    std::cout << std::endl << "Depixelized" << std::endl;
    std::cout << "x: " << depixelize_output3[0] << "; y: " << depixelize_output3[1] << std::endl;


    ////

    Eigen::Vector2s pixelize_output4;
    Eigen::Matrix2s pixelize_jacobian2;
    pinhole::pixellizePoint(k_test2,pixelize_ud,pixelize_output4,pixelize_jacobian2);

    std::cout << std::endl << "TEST pixelizePoint; Jacobians" << std::endl;
    std::cout << std::endl << "Original" << std::endl;
    std::cout << "x: " << pixelize_ud[0] << "; y: " << pixelize_ud[1] << std::endl;
    std::cout << std::endl << "Pixelized" << std::endl;
    std::cout << "x: " << pixelize_output4[0] << "; y: " << pixelize_output4[1] << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 pixelize_jacobian2.row(0).col(0) << " " << pixelize_jacobian2.row(0).col(1) << std::endl <<
                 pixelize_jacobian2.row(1).col(0) << " " << pixelize_jacobian2.row(1).col(1) << std::endl;


    Eigen::Vector2s depixelize_output4;
    Eigen::Matrix2s depixelize_jacobian2;
    pinhole::depixellizePoint(k_test2,pixelize_output4,depixelize_output4,depixelize_jacobian2);

    std::cout << std::endl << "Depixelized" << std::endl;
    std::cout << "x: " << depixelize_output4[0] << "; y: " << depixelize_output4[1] << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 depixelize_jacobian2.row(0).col(0) << " " << depixelize_jacobian2.row(0).col(1) << std::endl <<
                 depixelize_jacobian2.row(1).col(0) << " " << depixelize_jacobian2.row(1).col(1) << std::endl;

    Eigen::Matrix2s test_jacobian_pix;
    test_jacobian_pix =  pixelize_jacobian2 * depixelize_jacobian2;

    std::cout << "\n\n\n==> Jacobian Testing" << std::endl <<
                 test_jacobian_pix.row(0).col(0) << " " << test_jacobian_pix.row(0).col(1) << std::endl <<
                 test_jacobian_pix.row(1).col(0) << " " << test_jacobian_pix.row(1).col(1) << std::endl;




    //================================= projectPoint Complete


//    //distortion
//    distortion2;

//    //k
//    k_test2;

//    //3Dpoint
//    project_point_normalized_test;


    Eigen::Vector2s point2D_test5;
    point2D_test5 = pinhole::projectPoint(k_test2,distortion2,project_point_normalized_test);

    std::cout << "\n--------------------------------------------------------" << std::endl;
    std::cout << std::endl << "TEST projectPoint Complete" << std::endl;
    std::cout << "\nPARAMS" << std::endl;
    std::cout << "\nDistortion:" << std::endl;
    std::cout << "d1: " << distortion2[0] << "; d2: " << distortion2[1] << std::endl << std::endl;
    std::cout << "k values:" << std::endl;
    std::cout << "u0: " << k_test2[0] << "; v0: " << k_test2[1] << "; au: " << k_test2[2] << "; av: " << k_test2[3] << std::endl << std::endl;
    std::cout << "3D Point" << std::endl;
    std::cout << "x: " << project_point_normalized_test[0] << "; y: " << project_point_normalized_test[1]
              << "; z: " << project_point_normalized_test[2] << std::endl;
    std::cout << "\n\n\nFirst function output" << std::endl;
    std::cout << "x: " << point2D_test5[0] << "; y: " << point2D_test5[1] << std::endl;

        //distance
    Eigen::Vector2s point2D_test6;
    Scalar distance_test4;
    pinhole::projectPoint(k_test2,distortion2,project_point_normalized_test,point2D_test6,distance_test4);

    std::cout << std::endl << "Second function output" << std::endl;
    std::cout << "x: " << point2D_test6[0] << "; y: " << point2D_test6[1] << "; dist: " << distance_test4 << std::endl;

        //jacobian
    Eigen::Vector2s point2D_test7;
    Eigen::MatrixXs jacobian_test3(2,3);
    pinhole::projectPoint(k_test2,distortion2,project_point_normalized_test,point2D_test7,jacobian_test3);

    std::cout << std::endl << "Third function output" << std::endl;
    std::cout << "x: " << point2D_test7[0] << "; y: " << point2D_test7[1] << std::endl;
    std::cout << "\n-->Jacobian" << std::endl <<
                 jacobian_test3.row(0).col(0) << " " << jacobian_test3.row(0).col(1) << " " << jacobian_test3.row(0).col(2) << std::endl <<
                 jacobian_test3.row(1).col(0) << " " << jacobian_test3.row(1).col(1) << " " << jacobian_test3.row(1).col(2) << std::endl;

        //jacobian and distance
    Eigen::Vector2s point2D_test8;
    Eigen::MatrixXs jacobian_test4(2,3);
    Scalar distance_test3;
    pinhole::projectPoint(k_test2,distortion2,project_point_normalized_test,point2D_test8,distance_test3,jacobian_test4);

    std::cout << std::endl << "Fourth function output" << std::endl;
    std::cout << "x: " << point2D_test8[0] << "; y: " << point2D_test8[1] << "; dist: " << distance_test3 << std::endl;
    std::cout << "\n-->Jacobian" << std::endl <<
                 jacobian_test4.row(0).col(0) << " " << jacobian_test4.row(0).col(1) << " " << jacobian_test4.row(0).col(2) << std::endl <<
                 jacobian_test4.row(1).col(0) << " " << jacobian_test4.row(1).col(1) << " " << jacobian_test4.row(1).col(2) << std::endl;


    /////////////////////////////

//    //correction
//    correction_test2

//    //2Dpoint
//    point2D_test5

    Scalar depth3 = project_point_normalized_test[2];

    Eigen::Vector3s point3D_backproj5;
    point3D_backproj5 = pinhole::backprojectPoint(k_test2,correction_test2,point2D_test5,depth3);

    std::cout << "\n\nTEST backprojectPoint Complete" << std::endl;
    std::cout << std::endl << "First function output" << std::endl;
    std::cout << "x: " << point3D_backproj5[0] << "; y: " << point3D_backproj5[1] << "; z: " << point3D_backproj5[2] << std::endl;



        //jacobian
    Eigen::Vector3s point3D_backproj4;
    Eigen::MatrixXs jacobian_backproj2(3,2);
    Eigen::Vector3s depth_jacobian2;
    pinhole::backProjectPoint(k_test2,correction_test2,point2D_test7,depth3,point3D_backproj4,jacobian_backproj2,depth_jacobian2);

    std::cout << std::endl << "Second function output" << std::endl;
    std::cout << "x: " << point3D_backproj4[0] << "; y: " << point3D_backproj4[1] << "; z: " << point3D_backproj4[2] << std::endl;
    std::cout << "\n--> Jacobian" << std::endl <<
                 jacobian_backproj2.row(0).col(0) << " " << jacobian_backproj2.row(0).col(1) << std::endl <<
                 jacobian_backproj2.row(1).col(0) << " " << jacobian_backproj2.row(1).col(1) << std::endl <<
                 jacobian_backproj2.row(2).col(0) << " " << jacobian_backproj2.row(2).col(1) << std::endl;
    std::cout << "\n--> Jacobian - depth" << std::endl <<
                 depth_jacobian2[0] << " " << depth_jacobian2[1] << " " << depth_jacobian2[2] << " " << std::endl;




    Eigen::Matrix2s test_jacobian_complete;
    test_jacobian_complete =  jacobian_test4 * jacobian_backproj2;

    std::cout << "\n\n\n==> Jacobian Testing" << std::endl <<
                 test_jacobian_complete.row(0).col(0) << " " << test_jacobian_complete.row(0).col(1) << std::endl <<
                 test_jacobian_complete.row(1).col(0) << " " << test_jacobian_complete.row(1).col(1) << std::endl;


}







