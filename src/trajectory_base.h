
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

class NodeTerminus;
class FrameBase;

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "node_terminus.h"
#include "frame_base.h"
#include "state_base.h"

//class TrajectoryBase
class TrajectoryBase : public NodeLinked<NodeTerminus,FrameBase>
{
    protected:
        unsigned int fixed_size_; // Limits the number of frames forming the trajectory
        
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
        TrajectoryBase() : NodeLinked(TOP, "TRAJECTORY")
        {
            //
        };

        /** \brief Destructor
         *
         * Destructor
         *
         **/        
        ~TrajectoryBase(){};

        /** \brief Prints self info to std out
         *
         * Prints self info to std out
         *
         **/        
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const {};
        
};
#endif
