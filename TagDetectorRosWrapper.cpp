#include <apriltag2_ros/TagDetectorRosWrapper.h>

TagDetectorRosWrapper::TagDetectorRosWrapper(const std::string &config_file)
{
    readConfig(config_file);
    if (params_.family == "tag36h11")
        family_ = tag36h11_create();
    else if (params_.family == "tag36h10")
        family_ = tag36h10_create();
    else if (params_.family == "tag36artoolkit")
        family_ = tag36artoolkit_create();
    else if (params_.family == "tag25h9")
        family_ = tag25h9_create();
    else if (params_.family == "tag25h7")
        family_ = tag25h7_create();
    else
        throw(std::runtime_error("Unsupported/not implemented family"));

    detector_ = apriltag_detector_create();
    detector_->quad_decimate = params_.decimate;
    detector_->quad_sigma = params_.blur;
    detector_->nthreads = params_.threads;
    detector_->refine_edges = params_.refine_edges;
    detector_->refine_decode = params_.refine_decode;
    detector_->refine_pose = params_.refine_pose;
    apriltag_detector_add_family(detector_, family_);
}

TagDetectorRosWrapper::TagDetectorRosWrapper(const std::string &config_file, const ros::NodeHandle &nh)
{
    nh_ = std::make_shared<ros::NodeHandle>(nh);
    it_ = std::make_shared<image_transport::ImageTransport>(nh);
    readConfig(config_file);
    if (params_.family == "tag36h11")
        family_ = tag36h11_create();
    else if (params_.family == "tag36h10")
        family_ = tag36h10_create();
    else if (params_.family == "tag36artoolkit")
        family_ = tag36artoolkit_create();
    else if (params_.family == "tag25h9")
        family_ = tag25h9_create();
    else if (params_.family == "tag25h7")
        family_ = tag25h7_create();
    else
        throw(std::runtime_error("Unsupported/not implemented family"));
    detector_ = apriltag_detector_create();
    detector_->quad_decimate = params_.decimate;
    detector_->quad_sigma = params_.blur;
    detector_->nthreads = params_.threads;
    detector_->refine_edges = params_.refine_edges;
    detector_->refine_decode = params_.refine_decode;
    detector_->refine_pose = params_.refine_pose;
    apriltag_detector_add_family(detector_, family_);
    initPublishers();
    initSubscribers();
}

TagDetectorRosWrapper::~TagDetectorRosWrapper()
{
    if (detector_)
    {
        apriltag_detector_destroy(detector_);
    }
    if (family_)
    {
        if (params_.family == "tag36h11")
            tag36h11_destroy(family_);
        else if (params_.family == "tag36h10")
            tag36h10_destroy(family_);
        else if (params_.family == "tag36artoolkit")
            tag36artoolkit_destroy(family_);
        else if (params_.family == "tag25h9")
            tag25h9_destroy(family_);
        else if (params_.family == "tag25h7")
            tag25h7_destroy(family_);
    }
}

void TagDetectorRosWrapper::initSubscribers()
{
    image_transport::TransportHints hints("raw", ros::TransportHints(), *nh_);
    sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe("image_raw", 2, &TagDetectorRosWrapper::callback,
                                                                        this, hints));
}

void TagDetectorRosWrapper::initPublishers()
{
    detections_pub_ = nh_->advertise<apriltag2_ros::Apriltags>("tags", 1);
    debug_img_pub_ = nh_->advertise<sensor_msgs::Image>("debug", 1);
}

apriltag2_ros::Apriltag TagDetectorRosWrapper::detectionToApriltagMsg(const apriltag_detection_t *detection)
{
    apriltag2_ros::Apriltag tag;
    // Gather basic information
    tag.id = detection->id;
    tag.family = params_.family;
    tag.hamming_distance = detection->hamming;
    tag.center.x = detection->c[0];
    tag.center.y = detection->c[1];
    tag.size = params_.tag_size;
    for (size_t i = 0; i < 4; ++i)
    {
        geometry_msgs::Point point;
        point.x = detection->p[i][0];
        point.y = detection->p[i][1];
        tag.corners.push_back(point);
    }
    // Omitting estimating pose from PnP here, since this is just a detection node
    return tag;
}

void TagDetectorRosWrapper::callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat debug_img;
    if (params_.do_debug)
        cv::cvtColor(cv_ptr->image, debug_img, CV_GRAY2BGR);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {.width = cv_ptr->image.cols,
                     .height = cv_ptr->image.rows,
                     .stride = cv_ptr->image.cols,
                     .buf = cv_ptr->image.data};

    zarray_t *detections = apriltag_detector_detect(detector_, &im);

    // Process detection
    if (zarray_size(detections) != 0)
    {
        apriltag2_ros::Apriltags detections_msg;
        detections_msg.header = image_msg->header;
        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // Actual processing
            detections_msg.apriltags.push_back(detectionToApriltagMsg(det));

            if (params_.do_debug)
            {
                cv::line(debug_img, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Scalar(0, 0xff, 0), 2);
                cv::line(debug_img, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0, 0, 0xff), 2);
                cv::line(debug_img, cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Scalar(0xff, 0, 0), 2);
                cv::line(debug_img, cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0xff, 0, 0), 2);

                std::stringstream ss;
                ss << det->id;
                std::string text = ss.str();
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                                    &baseline);
                cv::putText(debug_img, text, cv::Point(det->c[0] - textsize.width / 2,
                                                       det->c[1] + textsize.height / 2),
                            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
            }
        }
        if (params_.do_debug)
        {
            sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(image_msg->header, "bgr8", debug_img).toImageMsg();
            debug_img_pub_.publish(debug_img_msg);
        }
        detections_pub_.publish(detections_msg);
    }
    apriltag_detections_destroy(detections);
}

void TagDetectorRosWrapper::readConfig(const std::string &config_file)
{
    params_.read_config(config_file);
    std::cout << params_;
}