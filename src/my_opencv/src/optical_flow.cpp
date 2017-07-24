#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// NS-code
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "iostream"
#include "math.h"
#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
using namespace std;
using namespace cv;
int frame_count=0;

static const std::string OPENCV_WINDOW = "Image window";

cv::Mat gray;//current fream
cv::Mat gray2;//previous frame

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
//    image_sub_ = it_.subscribe("/camera/image_raw", 1,
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
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

    // Draw an example circle on the video stream
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // NS-code
    cv::cvtColor( cv_ptr->image, gray, CV_BGR2GRAY );//make an image monochrome 

    image_pub_.publish( cv_bridge::CvImage( std_msgs::Header(), "bgr8", gray).toImageMsg());

    
    // NS-code OpticalFlow
    Mat disp = gray.clone();//copy
    if( frame_count == 0 ) gray2 = gray.clone();//exception handling

    if( frame_count > 0 )
    {
    	vector<cv::Point2f> prev_pts;//point
    	
    	Size flowSize(100,100);//interval between the flows
    	
    	Point2f center = cv::Point(gray.cols/2., gray.rows/2.);
      for(int i=0; i<flowSize.width; ++i)
      {
      	for(int j=0; j<flowSize.width; ++j)
      	{
      		Point2f p(i*float(gray.cols)/(flowSize.width-1), j*float(gray.rows)/(flowSize.height-1));
          prev_pts.push_back((p-center)*0.95f+center);//store a calculation result
        }
      }
    
    	Mat flow;
         
    	calcOpticalFlowFarneback(gray2, gray, flow, 0.8, 10, 15, 3, 5, 1.1, 0);
            
      	// オプティカルフローの表示
    	std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
    	for(; p!=prev_pts.end(); ++p)
    	{
    		const cv::Point2f& fxy = flow.at<cv::Point2f>(p->y, p->x);
        double val_flow = flow.at<double>(p->y,p->x);
        if( val_flow > 10.0) cv::line(disp, *p, *p+fxy*8, cv::Scalar(0),1);//designation of the size
    	}
    	
      gray2 = gray.clone();
            
      imshow("vector", disp);//result image
      imshow("source", gray);//former image
            
      int c = waitKey(1);
     }
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
    // NS-code
    frame_count++;
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
