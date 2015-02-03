
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
        
    public:
        /** \brief Constructor
         * 
         * Constructor
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
        
        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the correspondence
		 *
		 * Returns a vector of scalar pointers to the first element of all state blocks involved in the correspondence.
		 *
		 **/
        virtual const std::vector<WolfScalar*> getBlockPtrVector() = 0;

};
#endif
