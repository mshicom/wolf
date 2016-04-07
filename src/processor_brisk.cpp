// Wolf includes
#include "processor_brisk.h"

// OpenCV includes

// other includes
#include <bitset>

/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols,
                               unsigned int _grid_width, unsigned int _grid_height, unsigned int _min_features_th,
                               int _threshold, int _octaves, float _pattern_scales) :
    ProcessorTrackerFeature(PRC_TRACKER_BRISK),
    brisk_(_threshold, _octaves, _pattern_scales),
    act_search_grid_(_image_rows,_image_cols,_grid_width, _grid_height),
    min_features_th_(_min_features_th)
{
    brisk_.create("Feature2D.BRISK");
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnown()
{

    std::cout << std::endl << "<---- processKnownFeatures ---->" << std::endl << std::endl;

    act_search_grid_.renew();

    //////// Poner aquí lo de que se asignen los features de last en la lista para dibujar en ROS

    unsigned int n_tracks = 0;
    std::cout << "Nbr of features in incoming: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;
    n_tracks = track(*(last_ptr_->getFeatureListPtr()), *(incoming_ptr_->getFeatureListPtr()));
    std::cout << "Nbr of features in incoming: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;

    return n_tracks;
}

void ProcessorBrisk::drawFeaturesLastFrame(cv::Mat _image, Eigen::Vector2i _feature_point_last)
{
    cv::Point point;
    point.x = _feature_point_last[0];
    point.y = _feature_point_last[1];
    cv::circle(_image,point,2,cv::Scalar(250.0,180.0,70.0),-1,8,0);
    cv::imshow("Keypoint drawing",_image);
}

void ProcessorBrisk::drawRoiLastFrame(cv::Mat _image,cv::Rect _roi)
{
    cv::rectangle(_image,_roi,cv::Scalar(88.0,70.0,254.0),1,8,0);
    cv::imshow("Keypoint drawing",_image);
}


void ProcessorBrisk::drawFeatures(CaptureBase* const _last_ptr)
{
    cv::KeyPoint _kp;
    cv::Mat image = image_last_;

    //TODO: Why is it 0?
    std::cout << "size: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    for (std::list<FeatureBase*>::iterator feature_iter=last_ptr_->getFeatureListPtr()->begin();feature_iter != last_ptr_->getFeatureListPtr()->end(); ++feature_iter)
    {

        _kp = ((FeaturePointImage*)*feature_iter)->getKeypoint();
        cv::Point point;
        point.x = _kp.pt.x;
        point.y = _kp.pt.y;
        cv::circle(image,point,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
        std::cout << "keypoint: " << _kp.pt << std::endl;
    }
    cv::imshow("Keypoint drawing",image);

}


unsigned int ProcessorBrisk::briskDetect(cv::Mat _image, cv::Rect &_roi, std::vector<cv::KeyPoint> &_new_keypoints, cv::Mat & new_descriptors)
{
    std::cout << std::endl << "<---- briskImplementation ---->" << std::endl << std::endl;

    cv::Mat _image_roi = _image(_roi);

    //Brisk Algorithm
    //brisk_.create("Feature2D.BRISK");  //TODO: look if this can be done in the constructor
    brisk_.detect(_image_roi, _new_keypoints);
    brisk_.compute(_image_roi, _new_keypoints,new_descriptors);

    //double dist_ham = cv::norm(new_descriptors,new_descriptors,'NORM_HAMMING');

    if(_new_keypoints.size()!=0)
    {
            for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
            {
                _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
                _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
            }

            return new_descriptors.rows;  //number of features
    }
    else
    {
        return 0;
    }
}

void ProcessorBrisk::addNewFeaturesInCapture(std::vector<cv::KeyPoint> _new_keypoints, cv::Mat new_descriptors)
{
    for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
    {
        FeaturePointImage* point_ptr = new FeaturePointImage(_new_keypoints[i],new_descriptors.row(i),false);
        addNewFeature(point_ptr);
    }
    std::cout << "size of new list: " << getNewFeaturesList().size() << std::endl;
}


// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    std::cout << std::endl << "<---- detectNewFeatures ---->" << std::endl << std::endl;

    cv::Rect roi;

    ///START OF THE SEARCH
    unsigned int n_features = 0;
    unsigned int n_iterations = 0;
    while((n_features < 1) && (n_iterations <50))
    {
        bool roi_exists = act_search_grid_.pickRoi(roi);

        if(roi_exists==false)
        {
            break;
        }

        std::vector<cv::KeyPoint> new_keypoints;
        cv::Mat new_descriptors;

        n_features = briskDetect(image_last_, roi, new_keypoints, new_descriptors);

        if (n_features == 0)
        {
            act_search_grid_.blockCell(roi);
        }
        else
        {
            //Escoger uno de los features encontrados
            addNewFeaturesInCapture(new_keypoints,new_descriptors);
            act_search_grid_.blockCell(roi); //hace hitCell (con el punto seleccionado)
        }


        std::cout << "NFL size: " << getNewFeaturesList().size() << std::endl;

        /** To read the values in the new list */

        /**
        for (FeatureBase* feature_ptr : getNewFeaturesList())
        {
            std::cout << "newFeaturesList Feature, keypoint: " << ((FeaturePointImage*)feature_ptr)->getKeypoint().pt << std::endl;
            std::cout << "newFeaturesList Feature, descriptor: [";
            for (float desc_ptr : ((FeaturePointImage*)feature_ptr)->getDescriptor())
            {
                std::cout << ", " << desc_ptr;
            }
            std::cout << "]" << std::endl;
        }
        */

        n_iterations++;
    }
    std::cout << "n_features: " << n_features << std::endl;

    //////// Poner aquí lo de que se asignen los features obtenidos (y guardados en la lista "new_features_list_last_"
    ///  en una lista para dibujar en ROS

    return n_features;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    std::cout << "Thereshold: " << min_features_th_ << std::endl;
    std::cout << "Feature_list size: " << ((CaptureImage*)incoming_ptr_)->getFeatureListPtr()->size() << std::endl;
    std::cout << "<--------------------------------> voteForKeyFrame?: " << (((CaptureImage*)incoming_ptr_)->getFeatureListPtr()->size() < min_features_th_) << std::endl;
    return (incoming_ptr_->getFeatureListPtr()->size() < min_features_th_);
    //return 0;
}

void ProcessorBrisk::process(CaptureBase* const _incoming_ptr)
{
    std::cout << std::endl << "<---- process ---->" << std::endl << std::endl;
    image_incoming_ = ((CaptureImage*)_incoming_ptr)->getImage();
    image_last_ = ((CaptureImage*)last_ptr_)->getImage();
    ProcessorTrackerFeature::process(_incoming_ptr);
    std::cout << std::endl << "<---- end process ---->" << std::endl << std::endl;
    drawFeatures(last_ptr_);
    cv::waitKey(1000);
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{
    return new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feat_or_lmk_ptr)
{
    return nullptr;
}


/** Private */

unsigned int ProcessorBrisk::track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out)
{
    std::cout << std::endl << "<---- processFeaturesForMatching ---->" << std::endl << std::endl;

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int feature_roi_width = 30;
    unsigned int feature_roi_heigth = 30;

    unsigned int feature_roi_x = 0;
    unsigned int feature_roi_y = 0;

    unsigned int n_candidates = 0;
    //int tolerance = 20;

    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;

    std::cout << "---------------------Number of features to match: " << _feature_list_in.size() << std::endl;
    unsigned int n_last_capture_feat = 0;
    for(auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)feature_base_ptr;
        Eigen::Vector2i feature_point = {feature_ptr->getKeypoint().pt.x,feature_ptr->getKeypoint().pt.y}; //TODO: Look if this is the right order
        act_search_grid_.hitCell(feature_point);
//        std::cout << std::endl << "Last feature keypoint: " << feature_ptr->getKeypoint().pt << std::endl;
        std::cout << "------------------- Searching for feature : " << feature_ptr->nodeId() << std::endl;

        feature_roi_x = (feature_ptr->getKeypoint().pt.x)-(feature_roi_heigth/2);
        feature_roi_y = (feature_ptr->getKeypoint().pt.y)-(feature_roi_width/2);
        cv::Rect roi_for_matching(feature_roi_x,feature_roi_y,feature_roi_width,feature_roi_heigth);

        drawFeaturesLastFrame(image_incoming_,feature_point);
        drawRoiLastFrame(image_incoming_,roi_for_matching);


        n_candidates = briskDetect(image_incoming_, roi_for_matching, new_keypoints, new_descriptors);
        std::cout << "------------------ Number of detected candidates: " << n_candidates << std::endl;


        // Comparison of position

        if(n_candidates != 0)
        {
            std::vector<float> feature_descriptor = feature_ptr->getDescriptor();

            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada

                cv::BFMatcher matcher(cv::NORM_HAMMING);
                std::vector<cv::DMatch> matches;
                cv::Mat f_descriptor = feature_ptr->getDescriptor();

                matcher.match(f_descriptor, new_descriptors, matches);

                std::cout << "------------------ Number of matches: " << matches.size() << std::endl;
                std::cout << "------------------ Hamiing distance: " << matches[0].distance << std::endl;

            if(matches.size() >= 1)
            {
                if(matches[0].distance < 200)
                {
                    FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[matches[0].trainIdx],(new_descriptors.row(matches[0].trainIdx)),true);
                    _feature_list_out.push_back(point_ptr);
                    n_last_capture_feat++;
                }
            }
        }
    }
    std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;
    std::cout << "n_last_capture_size: " << _feature_list_in.size() << std::endl;






    return n_last_capture_feat;
}
