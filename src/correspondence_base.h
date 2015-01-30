
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
        CorrespondenceType type_; //type of correspondence (types defined at wolf.h)
        std::vector<WolfScalar*> state_block_ptr_vector_;
        std::vector<unsigned int> state_block_sizes_vector_;
        
    public:
        /** \brief Conructor
         * 
         * Conructor
         * 
         **/                
        CorrespondenceBase(CorrespondenceType _tp);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~CorrespondenceBase();

        /** \brief Returns the correspondence type
         * 
         * Returns the correspondence type
         * 
         **/
        CorrespondenceType getType() const;
        
        /** \brief Returns a pointer to the measurement getBlockPtrVector
         * 
         * Returns a pointer to the measurement getBlockPtrVector in which this correspondence depends
         * Measurement is owned by upper-level feature
         * 
         **/
        const std::vector<WolfScalar*> * getStateBlockPtrVector();
};
#endif
