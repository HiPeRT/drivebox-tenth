#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>

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
  
  zbar::ImageScanner scanner;

  
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
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1); 
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

    cv::Mat grey;  
    cvtColor(image,grey,CV_RGB2GRAY);  
    int width = image.cols;   
    int height = image.rows;   
    uchar *raw = (uchar *)grey.data;   
    zbar::Image img(width, height, "Y800", raw, width * height);   
    int n = scanner.scan(img);   

    // extract results   
    for(zbar::Image::SymbolIterator symbol = img.symbol_begin();  
        symbol != img.symbol_end();   
        ++symbol) {   
      
      cv::vector<cv::Point> vp;   
      // do something useful with results   
      std::cout << "decoded " << symbol->get_type_name()
                <<" symbol \"" << symbol->get_data() << '\"' <<" ";   

      if(symbol->get_data() == "Start") {
        //PUB MESSAGE
         std_msgs::Bool msg;
        msg.data = false;
        start_pub_.publish(msg);

        std::cout<<"OK!!! sending auto mode message";
      }

      std::cout<<"\n";

      int n = symbol->get_location_size();   
      for(int i=0;i<n;i++){   
        vp.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));   
      }   
    
      cv::RotatedRect r = minAreaRect(vp);   
      cv::Point2f pts[4];   
      r.points(pts);   
      for(int i=0;i<4;i++)   
        line(image,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),3);   
     
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
