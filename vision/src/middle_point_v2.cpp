#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

#define PI 3.14159265

/* ---------------------------------- Parameters ----------------------------------*/
bool houghP = 0;
int minAngle = 30;
Mat frame, frame_org, frame_gray, frame_canny, frame_open, frame_erode, frame_open_canny,
    frame_erode_canny, dst_frame;
std::vector<Vec4i> linesP;
std::vector<Vec2f> lines;
// image dimensions
int width = 640;
int height = 480;

// ransac parameters
int N_iterations = 10; // # of iterations for ransac
int threshold_ransac = 20; // distance within which the hypothesis is classified as an inlier

// low pass parameters
const int lowPass_window = 4;
int window [lowPass_window][2] = {};
int lp_pointer = 0;
int running_sum [2] = {};

// other params
int erosion_size = 3;

/* ---------------------------------- Functions ----------------------------------*/
void detectVP(int, void*);
void verifyLine(vector<Vec4i>&);


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void call_back(const sensor_msgs::ImageConstPtr& msg) {

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

          namedWindow("please", CV_WINDOW_AUTOSIZE);
//          processFrame(cv_ptr->image);
          imshow("please", cv_ptr->image);


//          geometry_msgs::Twist steering = calculateMovement(VPcell, grid_size);
//          chatter_pub.publish(msg);

}

void scanCallback (const std_msgs::String::ConstPtr& scan_in)
{

    int p[4];
    std::size_t pos1;
    std::size_t pos2;
    std::string tmp = scan_in->data;
    std::string tmp_pos;
    for (int i=0; i <4; ++i)
    {
        pos1 = tmp.find(":");
        pos2 = tmp.find(",");
        tmp_pos = tmp.substr(pos1+1, pos2-pos1-1);
//        tmp.erase(pos1);
//        tmp.erase(pos2);
        tmp.erase(pos1, pos2);
        cout<<tmp<<endl;
//        tmp.erase.(pos1, pos2);

        p[i] = std::atoi(tmp_pos.c_str());

        cout<<tmp<<endl;
    }


    ROS_INFO("I heard: [%s]", scan_in->data.c_str());
    cout<<p[0]<<" "<<p[1]<<" "<<p[2]<<" "<<p[3]<<endl;
    cout<<endl;


//    ROS_INFO("I heard: [%s]", scan_in->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
//  cv::namedWindow("view");
//  cv::startWindowThread();
//  image_transport::ImageTransport it(nh);convert_LaserScan_to_MultiEchoLaserScan
//  image_transport::Subscriber sub = it.subscribe("/right/image_raw_color", 2, call_back);

//  image_transport::Subscriber sub = it.subscribe("/right/image_rect_color", 2, imageCallback, ros::VoidPtr(),image_transport::TransportHints("compressed"));
  ros::Subscriber sub = nh.subscribe("/ultrasonic_data", 2, scanCallback);
  ros::spin();
//  cv::destroyWindow("view");
}
