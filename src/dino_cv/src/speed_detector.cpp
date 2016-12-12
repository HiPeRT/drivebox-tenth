#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/video.hpp"

#include "std_msgs/Bool.h"

static const std::string OPENCV_WINDOW = "Image window";
bool VIEW=false;


using namespace cv;
using namespace std;

Point2f point;
bool addRemovePt = false;
bool needToInit = false;
const int MAX_COUNT = 500;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher start_pub_;

  Mat gray, prevGray;
  vector<Point2f> points[2];

  
public:
  ImageConverter()
    : it_(nh_)
  {
    ros::NodeHandle n;
    
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/zed/left/image_raw_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    start_pub_ = n.advertise<std_msgs::Bool>("/eStop", 1);

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

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    cv::Mat image = cv_ptr->image;

    // Retrieve left color image
    cvtColor(image, gray, COLOR_BGR2GRAY);

    if( needToInit ) {
            // automatic initialization
            printf("init\n");
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
    }
    else if( !points[0].empty() )
    {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                    gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
            3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ ) {
                    if(addRemovePt){
                            if( norm(point - points[1][i]) <= 5 ) {
                                    addRemovePt = false;
                                    continue;
                            }
                    }
            
                    if( !status[i] )
                    continue;
                    points[1][k++] = points[1][i];
                    circle( image, points[1][i], 10, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
    }
    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT) {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
    }
    needToInit = false;

    // Update GUI Window
    if(VIEW)    
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    char c = (char)waitKey(10);
    if( c == 27 )
      return;

    switch( c ) {
    case 'r':
            needToInit = true;
            break;
    case 'c':
            points[0].clear();
            points[1].clear();
            break;
    case 'n':
            break;
    }
    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);

    
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
