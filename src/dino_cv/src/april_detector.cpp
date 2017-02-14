#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#include "std_msgs/Bool.h"

static const std::string OPENCV_WINDOW = "Image window";
bool VIEW=false;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher start_pub_;

  AprilTags::TagDetector* tagDetector;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    ros::NodeHandle n;
    
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    AprilTags::TagCodes tagCodes = AprilTags::tagCodes36h11;
    tagDetector = new AprilTags::TagDetector(tagCodes);

    if(VIEW)
      cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    if(VIEW)
      cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;

    cv::Mat gray;  
    cvtColor(image,gray,CV_RGB2GRAY);  
    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(gray);
    for (int i=0; i<detections.size(); i++) {
      detections[i].draw(image);

      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      detections[i].getRelativeTranslationRotation(0.05, 600, 600, 640/2, 480/2, translation, rotation);
    }
    
    // Update GUI Window
    if(VIEW)    
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  if(argc >1 && strcmp(argv[1], "--view") == 0) {
    printf("view enabled\n");
    VIEW = true;
  }

  ImageConverter ic;
  ros::spin();
  return 0;
}
