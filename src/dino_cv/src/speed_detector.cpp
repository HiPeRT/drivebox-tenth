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
const int MAX_COUNT = 500;

class ImageConverter {

private:    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher start_pub_;

    cv::Mat gray, prevGray;
    cv::vector<cv::Point2f> points[2];
  
public:
    ImageConverter() : it_(nh_) {
        
        ros::NodeHandle n;
    
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/zed/left/image_raw_color", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        start_pub_ = n.advertise<std_msgs::Bool>("/eStop", 1);

        if(VIEW)
            cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        if(VIEW)
            cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        
        cv_bridge::CvImagePtr cv_ptr;
        
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;

        float w = image.size().width;
        float h = image.size().height;
        cv::Point2f p1(w/4, h/4);
        cv::Point2f p2(w - p1.x, h - p1.y);
        
        cv::Mat mask(image.size(), CV_8UC1);
        mask.setTo(cv::Scalar::all(1));
        rectangle(mask, p1, p2, cv::Scalar(0,0,0), -1, 8, 0);


        cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
        cv::Size subPixWinSize(10,10), winSize(31,31);

        cvtColor(image, gray, cv::COLOR_BGR2GRAY);


        // search for new points
        int ask = MAX_COUNT - points[0].size();
        if(ask > MAX_COUNT /10) {
            printf("new points\n");
            goodFeaturesToTrack(gray, points[1], ask, 0.01, 2, mask, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);
        
            int old_size = points[0].size();
            points[0].resize(old_size + points[1].size());
            for(int i=0; i<points[1].size(); i++)
                points[0][old_size + i] = points[1][i];
        }
        
        if(!points[0].empty()) {
            cv::vector<uchar> status;
            cv::vector<float> err;
            
            if(prevGray.empty())
                gray.copyTo(prevGray);
            
            calcOpticalFlowPyrLK(   prevGray, gray, points[0], points[1], 
                                    status, err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k;
            for(i=k=0; i<points[1].size(); i++) {
            
                if(!status[i])
                    continue;
           
                points[1][k++] = points[1][i];
                line(image, points[0][i], points[1][i], cv::Scalar(0,0,255), 1, 8, 0);
                circle(image, points[1][i], 5, cv::Scalar(0,255,0), 1, 8);
            }

            points[1].resize(k);
        }

        rectangle(image, p1, p2, cv::Scalar(255,0,0), 1, 8, 0);

        // Update GUI Window
        if(VIEW)    
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);

        cv::waitKey(10);

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
