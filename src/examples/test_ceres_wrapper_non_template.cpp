// Testing a full wolf tree avoiding template classes for NodeLinked derived classes

//std includes
#include <iostream>
#include <vector>
// #include <memory>
// #include <random>
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
	
	void display() const
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

//an example of a specialized corrspondence class
class CorrespondenceOdom2D : public CorrespondenceBaseX
{
    protected:
        Eigen::Map<Eigen::Vector3s> pose_previous_;
        Eigen::Map<Eigen::Vector3s> pose_current_; 
        ceres::CostFunction* cost_function_ptr_;
        
    public:
        CorrespondenceOdom2D(double * _st) :
            CorrespondenceBaseX(2,{0,3},{3,3}),
            pose_previous_(_st + block_indexes_.at(0) , block_sizes_.at(0)),
            pose_current_(_st + block_indexes_.at(1) , block_sizes_.at(1))
        {
            //
        };
        
        ~CorrespondenceOdom2D()
        {
            //
        };
        
        void init()
        {
            std::cout << "pose_previous_ = " << pose_previous_.transpose() << std::endl;
            std::cout << "pose_current_ = " << pose_current_.transpose() << std::endl;
        };
    
};

int main(int argc, char** argv) 
{
    //Welcome message
    std::cout << " ========= WOLF-CERES test with non-template classes ===========" << std::endl << std::endl;

    //variables
    Eigen::VectorXs state(6);
//     CorrespondenceBaseX *base_corresp;
    CorrespondenceOdom2D *odom_corresp;
        
    //set state
    state << 1,2,3,4,5,6;
    
    //create base correspondence
//     base_corresp = new CorrespondenceBaseX(2,{0,3},{3,3});
//     base_corresp->display();
    
    //create odom correspondence
    odom_corresp = new CorrespondenceOdom2D(state.data());
    odom_corresp->display();
    odom_corresp->init();

    //free memory
//     delete base_corresp;
    delete odom_corresp;
    
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}



