
#ifndef CORRESPONDENCE_BASE_H_
#define CORRESPONDENCE_BASE_H_

// Forward declarations for node templates
class FeatureBase;
class NodeTerminus;

//std includes
//

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "feature_base.h"
#include "node_terminus.h"

//class CorrespondenceBase
class CorrespondenceBase : public NodeLinked<FeatureBase,NodeTerminus>
{
    protected:
        unsigned int nblocks_; //number of state blocks in which the correspondence depends on.
        std::vector<unsigned int> block_indexes_; //state vector indexes indicating start of each state block. This vector has nblocks_ size. 
        std::vector<unsigned int> block_sizes_; //sizes of each state block. This vector has nblocks_ size. 
        
    public:
        CorrespondenceBase(const unsigned int _nb, const std::vector<unsigned int> & _bindexes, const std::vector<unsigned int> & _bsizes) :
            NodeLinked(BOTTOM, "CORRESPONDENCE"),
            nblocks_(_nb),
            block_indexes_(_bindexes),
            block_sizes_(_bsizes)
        {
            assert(block_sizes_.size() == nblocks_);
        };
        
        virtual ~CorrespondenceBase()
        {
            //
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
#endif
