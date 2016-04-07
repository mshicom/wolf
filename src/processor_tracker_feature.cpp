/*
 * \processor_tracker.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker_feature.h"

ProcessorTrackerFeature::ProcessorTrackerFeature(ProcessorType _tp) :
        ProcessorTracker(_tp)
{
}

ProcessorTrackerFeature::~ProcessorTrackerFeature()
{
    // FIXME: This test with nullptr is not fail safe. Only the class design can make it safe, by ensuring
    // at all times that whenever incoming_ptr_ is not used, it points to nullptr.
    // See both flavors of reset(), and advance().
    if (incoming_ptr_ != nullptr)
        delete incoming_ptr_;
}

void ProcessorTrackerFeature::process(CaptureBase* const _incoming_ptr)
{
    // 1. First we track the known Features and create new constraints as needed
    incoming_ptr_ = _incoming_ptr;
    processKnown();

    // 2. Then we see if we want and we are allowed to create a KeyFrame
    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // 2.a. Detect new Features, initialize Landmarks, create Constraints
        processNew();
        // Make KeyFrame
        makeKeyFrame();
        // Reset the Tracker
        reset();
    }
    else
    {   // We did not create a KeyFrame:
        // 2.b. Update the tracker's last and incoming pointers one step ahead
        advance();
    }
}

unsigned int ProcessorTrackerFeature::processNew()
{
    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on this Capture to detect new Features,
     * eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark.
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */
    // We first need to populate the \b last Capture with new Features
    unsigned int n = detectNewFeatures();
//    if (usesLandmarks())
//    {
//        for (FeatureBase* feature_ptr : new_features_list_last_)
//        {
//            LandmarkBase* lmk_ptr = createLandmark(feature_ptr);
//            ConstraintBase* constr_ptr = createConstraint(feature_ptr, lmk_ptr);
//            getWolfProblem()->addLandmark(lmk_ptr);
//            feature_ptr->addConstraint(constr_ptr);
//        }
//    } // Done with Landmark creation

    track(new_features_list_last_, new_features_list_incoming_);

    // Append all new Features to the Capture's list of Features
    last_ptr_->getFeatureListPtr()->splice(last_ptr_->getFeatureListPtr()->end(), new_features_list_last_);
    incoming_ptr_->getFeatureListPtr()->splice(incoming_ptr_->getFeatureListPtr()->end(), new_features_list_incoming_);

    // return the number of new features detected in \b last
    return n;
}

void ProcessorTrackerFeature::makeKeyFrame()
{
    // Create a new non-key Frame in the Trajectory with the incoming Capture
    getWolfProblem()->createFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
    getWolfProblem()->getLastFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
    // Make the last Capture's Frame a KeyFrame so that it gets into the solver
    last_ptr_->getFramePtr()->setKey();
}
