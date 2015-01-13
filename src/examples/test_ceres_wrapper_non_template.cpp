// Testing a full wolf tree avoiding template classes for NodeLinked derived classes

//std includes
#include <iostream>
#include <vector>
#include <random>
// #include <memory>
// #include <typeinfo>

//ceres
#include "ceres/ceres.h"

//Wolf includes
#include "wolf.h"
#include "node_terminus.h"
#include "node_linked.h"

//forward declarations
class TrajectoryBaseX;
class FrameBaseX;
class CaptureBaseX;
class FeatureBaseX;
class CorrespondenceBaseX;

//class TrajectoryBaseX
class TrajectoryBaseX : public NodeLinked<NodeTerminus,FrameBaseX>
{
    protected:
        unsigned int length_; //just something to play
        
    public:
        TrajectoryBaseX(const unsigned int _len) :
            NodeLinked(TOP, "TRAJECTORY"),
            length_(_len)
        {
            //
        };
        
        ~TrajectoryBaseX()
        {
            
        };
};

//class FrameBaseX
class FrameBaseX : public NodeLinked<TrajectoryBaseX,CaptureBaseX>
{
    protected:
        double time_stamp_; //frame ts
        
    public:
        FrameBaseX(double _ts) :
            NodeLinked(MID, "FRAME"),
            time_stamp_(_ts)
        {
            //
        };
        
        ~FrameBaseX()
        {
            
        };
};

//class CaptureBaseX
class CaptureBaseX : public NodeLinked<FrameBaseX,FeatureBaseX>
{
    protected:
        double time_stamp_; //capture ts
        
    public:
        CaptureBaseX(double _ts) :
            NodeLinked(MID, "CAPTURE"),
            time_stamp_(_ts)
        {
            //
        };
        
        ~CaptureBaseX()
        {
            
        };
};

//class FeatureBaseX
class FeatureBaseX : public NodeLinked<CaptureBaseX,CorrespondenceBaseX>
{
    protected:
        
    public:
        FeatureBaseX() :
            NodeLinked(MID, "FEATURE")
        {
            //
        };
        
        ~FeatureBaseX()
        {
            //
        };
};

//class CorrespondenceBaseX
class CorrespondenceBaseX : public NodeLinked<FeatureBaseX,NodeTerminus>
{
    protected:
        unsigned int nblocks_; //number of state blocks in which the correspondence depends on.
        std::vector<unsigned int> block_indexes_; //state vector indexes indicating start of each state block. This vector has nblocks_ size. 
        std::vector<unsigned int> block_sizes_; //sizes of each state block. This vector has nblocks_ size. 
        ceres::CostFunction* cost_function_ptr_;
        
    public:
        CorrespondenceBaseX(const unsigned int _nb, const std::vector<unsigned int> & _bindexes, const std::vector<unsigned int> & _bsizes) :
            NodeLinked(BOTTOM, "CORRESPONDENCE"),
            nblocks_(_nb),
            block_indexes_(_bindexes),
            block_sizes_(_bsizes)
        {
            assert(block_sizes_.size() == nblocks_);
        };
        
        ~CorrespondenceBaseX()
        {
            //
        };
        
        ceres::CostFunction * getCostFunctionPtr()
        {
            return cost_function_ptr_;
        };
        	
        virtual void display() const
        {
                unsigned int ii; 
                std::cout << "number of blocks: " << nblocks_ << std::endl;
                std::cout << "block indexes: ";
                for (ii=0; ii<block_indexes_.size(); ii++) std::cout << block_indexes_.at(ii) << " ";
                std::cout << std::endl;
                std::cout << "block sizes: ";
                for (ii=0; ii<block_sizes_.size(); ii++) std::cout << block_sizes_.at(ii) << " ";
                std::cout << std::endl;
        };
};

class Odom2DFunctor
{
    protected:
        Eigen::Vector2s odom_inc_; //incremental odometry measurements (range, theta). Could be a map to data hold by capture or feature
    
    public:
        //constructor
        Odom2DFunctor(const Eigen::Vector2s & _odom):
            odom_inc_(_odom) 
        {
            //
        };
        
        //destructor
        ~Odom2DFunctor()
        {
            //
        };
        
        //cost function
        template <typename T>
        bool operator()(const T* const _x0, const T* const _x1, T* _residual) const
        {
            T dr, dth;
            
            //expected range and theta increments, given the state points
            dr = sqrt( (_x0[0]-_x1[0])*(_x0[0]-_x1[0]) + (_x0[1]-_x1[1])*(_x0[1]-_x1[1]) ); //sqrt could be avoided
            dth = _x1[2] - _x0[2];
            
            //residuals in range and theta components 
            _residual[0] = T(dr) - T(odom_inc_(0));
            _residual[1] = T(dth) - T(odom_inc_(1));
            
            //return 
            return true;
        };
};

//Specialized correspondence class for 2D odometry
class CorrespondenceOdom2D : public CorrespondenceBaseX
{
    protected:
        Eigen::Map<Eigen::Vector3s> pose_previous_;
        Eigen::Map<Eigen::Vector3s> pose_current_; 
        Eigen::Map<const Eigen::Vector2s> odom_inc_; 
        
    public:
        CorrespondenceOdom2D(WolfScalar * _st, const Eigen::Vector2s & _odom) :
            CorrespondenceBaseX(2,{0,3},{3,3}),
            pose_previous_(_st, block_indexes_.at(0) , block_sizes_.at(0)),
            pose_current_(_st, block_indexes_.at(1) , block_sizes_.at(1)),
            odom_inc_(_odom.data())
        {
            cost_function_ptr_ = new ceres::AutoDiffCostFunction<Odom2DFunctor,2,3,3>(new Odom2DFunctor(_odom));
        };
        
        ~CorrespondenceOdom2D()
        {
            //delete cost_function_ptr_;
        };
                
        double * getPosePreviousPtr()
        {
            return pose_previous_.data();
        };
        
        double * getPoseCurrentPtr()
        {
            return pose_current_.data();
        };        
        
        virtual void display() const
        {
            CorrespondenceBaseX::display();
            std::cout << "pose_previous_: " << pose_previous_.transpose() << std::endl;
            std::cout << "pose_current_: " << pose_current_.transpose() << std::endl;
            std::cout << "odom_inc_: " << odom_inc_.transpose() << std::endl;
        };
        
};

class GPSFixFunctor
{
    protected:
        Eigen::Vector3s gps_fix_; //GPS fix XYZ. Could be a map to data hold by capture or feature
    
    public:
        //constructor
        GPSFixFunctor(const Eigen::Vector3s & _gps_fix):
            gps_fix_(_gps_fix)
        {
            //
        };
        
        //destructor
        ~GPSFixFunctor()
        {
            //
        };
        
        //sets GPS fix
//         void setGPSFix(const Eigen::Vector3s & _fix)
//         {
//             gps_fix_ = _fix;
//         };
        
        //cost function
        template <typename T>
        bool operator()(const T* const _x0, T* _residual) const
        {
            _residual[0] = T( gps_fix_(0) ) - _x0[0];
            _residual[1] = T( gps_fix_(1) ) - _x0[1];
            _residual[2] = T( gps_fix_(2) ) - _x0[2];            
            return true;
        };
};

//Specialized correspondence class for GPS Fix data
class CorrespondenceGPSFix : public CorrespondenceBaseX
{
    protected:
        Eigen::Map<Eigen::Vector3s> location_;
        Eigen::Map<const Eigen::Vector3s> gps_fix_;

    public:
        CorrespondenceGPSFix(WolfScalar * _st, const Eigen::Vector3s & _gps_fix) :
            CorrespondenceBaseX(1,{0},{3}), 
            location_(_st + block_indexes_.at(0) , block_sizes_.at(0)),
            gps_fix_(_gps_fix.data(),3)
        {
            cost_function_ptr_ = new ceres::AutoDiffCostFunction<GPSFixFunctor,3,3>(new GPSFixFunctor(_gps_fix));
        };
        
        ~CorrespondenceGPSFix()
        {
            //delete cost_function_ptr_;
        };
        
        double * getLocation()
        {
            return location_.data();
        }
                
//         ceres::CostFunction * getCostFunctionPtr()
//         {
//             return cost_function_ptr_;
//         };
        
        virtual void display() const
        {
            CorrespondenceBaseX::display();
            std::cout << "location_: " << location_.transpose() << std::endl;
            std::cout << "gps_fix_: " << gps_fix_.transpose() << std::endl;            
        };        
};


int main(int argc, char** argv) 
{    
    //Welcome message
    std::cout << " ========= WOLF-CERES test with non-template classes ===========" << std::endl << std::endl;
    
    //init google log
    google::InitGoogleLogging(argv[0]);

    //variables
    Eigen::Vector3s odom_inc_true(20);
    Eigen::Vector3s pose_true(3);
    Eigen::VectorXs state1(6);
    Eigen::VectorXs state2(30);
    Eigen::Vector2s odom_data;
    Eigen::Vector3s gps_fix_data;
    CorrespondenceOdom2D *odom_corresp;
    CorrespondenceGPSFix *gps_fix_corresp;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    //ceres::ResidualBlockId block_id;
    
    //init true odom and true pose
    odom_inc_true << 0.2,0, 0.3,0.1, 0.3,0.2, 0.3,0, 0.4,0.1, 0.3,0.1, 0.2,0., 0.1,0.1, 0.1,0., 0.05,0.05;
    pose_true << 0,0,0;
    
    //random generators
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution_odom(0.0,0.01);    
    std::normal_distribution<WolfScalar> distribution_gps(0.0,0.02);
        
    //TEST 1
/* 
    std::cout << std::endl << "***** TEST 1. Simple optimizer through a derived class" << std::endl;
    
    //set state
    state1 << 1,2,3,4,5,6;
    std::cout << "INITIAL GUESS IS: " << state1.transpose() << std::endl;
    
    //create odom correspondence
    odom_corresp = new CorrespondenceOdom2D(state1.data());
    odom_corresp->display();
    
    //build problem
    block_id = problem.AddResidualBlock(odom_corresp->getCostFunctionPtr(),nullptr, odom_corresp->getPosePreviousPtr(), odom_corresp->getPoseCurrentPtr());
        
    //set options and solve
    options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);
    
    //display result
    std::cout << "RESULT IS: " << state1.transpose() << std::endl;
 */  

    //TEST 2
    std::cout << std::endl << "***** TEST 2. GPS fix problem" << std::endl;
    
    //clear previous blocks
    //problem.RemoveResidualBlock(block_id);
   
    //test loop
    for (unsigned int ii = 0; ii<10; ii++)
    {
        //just inventing a simple motion and the corresponding noise measurements for odometry and GPS
        pose_true(0) = pose_true(0) + odom_inc_true(ii*2) * cos(pose_true(2)+odom_inc_true(ii*2+1)); 
        pose_true(1) = pose_true(1) + odom_inc_true(ii*2) * sin(pose_true(2)+odom_inc_true(ii*2+1)); 
        odom_data << odom_inc_true(ii*2)+distribution_odom(generator), odom_inc_true(ii*2+1)+distribution_odom(generator);
        gps_fix_data << pose_true(0) + distribution_gps(generator), pose_true(1) + distribution_gps(generator), 0. + distribution_gps(generator);
        std::cout << "pose_true(" << ii << ") = " << pose_true.transpose() << std::endl;
        
        //setting intial guess. State prediction. TODO: use noisy odom for state prediction
        state2.middleRows(ii*3,3) << 0,0,0;
        
        //creating odom correspondence, exceptuating first iteration
        if ( ii !=0 )
        {
            odom_corresp = new CorrespondenceOdom2D(state2.data()+(ii-1)*3, odom_data);
        }
        
        //creating gps correspondence
        gps_fix_corresp = new CorrespondenceGPSFix(state2.data()+ii*3, gps_fix_data);
        gps_fix_corresp->display();
        
        //build problem 
        problem.AddResidualBlock(odom_corresp->getCostFunctionPtr(),nullptr, odom_corresp->getPosePreviousPtr(), odom_corresp->getPoseCurrentPtr());
        problem.AddResidualBlock(gps_fix_corresp->getCostFunctionPtr(),nullptr, gps_fix_corresp->getLocation());
    }
    
    //display initial guess
    std::cout << "INITIAL GUESS IS: " << state2.transpose() << std::endl;
    
    //set options and solve
    options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);
    
    //display result
    std::cout << "RESULT IS: " << state2.transpose() << std::endl;    
  
    //free memory
    delete odom_corresp;
    delete gps_fix_corresp;
    
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}

