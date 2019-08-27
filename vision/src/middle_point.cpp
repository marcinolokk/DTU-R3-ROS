#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


using namespace cv;
using namespace std;

#define PI 3.14159265
#define pdd pair<double, double>

/* ---------------------------------- Parameters ----------------------------------*/
bool houghP = 1;
int minLineAngle = 6, maxLineAngle = 80, min_deg_between_lines = 10;
int VPcell, grid_size=7;
cv::Mat frame, frame_org, frame_gray, frame_canny, test;
std::vector<Vec4i> linesP, lines_ext;
std::vector<Vec2f> lines;
vector<Point> intersections;
double twist_const = 1;

// image dimensions
int width;
int height;

// ransac parameters
int N_iterations = 10; // # of iterations for ransac
int threshold_ransac = 20; // distance within which the hypothesis is classified as an inlier

// low pass parameters
const int lowPass_window = 4;
int window [lowPass_window][2] = {};
int lp_pointer = 0;
int running_sum [2] = {};

/* ---------------------------------- Functions ----------------------------------*/
void detectVP(int, void*);
void verifyLine(vector<Vec4i>&);
void extendLines(std::vector<Vec4i>&);
void drawPoints(cv::Mat&, vector<Point>);
void processFrame(cv::Mat);
void drawGrid(cv::Mat&, int, int);
pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D);
int findVanishingPointCell(int, vector<Point>);
void imageCallback(const sensor_msgs::ImageConstPtr&);
void depthImageCallback(const sensor_msgs::ImageConstPtr&);


geometry_msgs::Twist calculateMovement(int, int);
//void AutoExp(const sensor_msgs::LaserScan::ConstPtr&);

// points agregation param
int counter=0, frames_delay = 5;



/*--------------------------------------------------------------------------------*/


class Robot
{
public:
    int p[4]= {0,0,0,0};
    
    void scanCallback (const std_msgs::String::ConstPtr& scan_in);
};

void Robot::scanCallback(const std_msgs::String::ConstPtr& scan_in)
{
    std::size_t pos1;
    std::size_t pos2;
    std::string tmp = scan_in->data;
    std::string tmp_pos;
    for (int i=0; i <4; ++i)
    {
        pos1 = tmp.find(":");
        pos2 = tmp.find(",");
        tmp_pos = tmp.substr(pos1+1, pos2-pos1-1);
        tmp.erase(pos1, pos2);

        p[i] = std::atoi(tmp_pos.c_str());
    }
}

int main(int argc, char **argv) {
//  ROS
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(n);
    Robot robot;

    image_transport::Subscriber sub = it.subscribe("/right/image_raw_color", 2,
                                      imageCallback, ros::VoidPtr(),image_transport::TransportHints("compressed"));
//    image_transport::Subscriber sub = it.subscribe("/depth/depth_registered", 2,
//                                      depthImageCallback);


    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    ros::Subscriber sub_robot = n.subscribe<std_msgs::String>("/ultrasonic_data", 2, &Robot::scanCallback, &robot);



    int count = 0;
    int cof = 1;
    Mat blank(320, 240, CV_8UC3, Scalar(0,0,0));
    imshow("blank", blank);

    /// rate of publishing
    ros::Rate loop_rate(33);
    while (ros::ok())
    {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//        ROS_INFO("%s", msg.data.c_str());

        geometry_msgs::Twist steering;
        steering.angular.z = -cof * 0.2;
//        chatter_pub.publish(steering);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        int key = cv::waitKey(1);
        if (key== 32){
            cof = cof*(-1);
            ROS_INFO("%s", "clicked");
        }
        if (key == 27)
        {
            steering.angular.z = 0;
            chatter_pub.publish(steering);
            return 0;
        }

        if (count%10 == 0)
            cout<<robot.p[0]<<" "<<robot.p[1]<<" "<<robot.p[2]<<" "<<robot.p[3]<<endl;


    }

    ros::spin();

    return 0;
}


/* ---------------------------------- Ros Part ---------------------------------*/

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

    //          namedWindow("please", CV_WINDOW_AUTOSIZE);
              processFrame(cv_ptr->image);
    //          imshow("please", cv_ptr->image);


    //          geometry_msgs::Twist steering = calculateMovement(VPcell, grid_size);
    //          chatter_pub.publish(msg);
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::imshow("view", cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


//void call_back1(const sensor_msgs::ImageConstPtr& msg) {

    


//          geometry_msgs::Twist steering = calculateMovement(VPcell, grid_size);
////          chatter_pub.publish(msg);

//}

//void AutoExp::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
//     //scan->ranges[] are laser readings
//     cout<<scan->ranges[1];
//}

/* ------------------------------------------------------------------------------*/

geometry_msgs::Twist calculateMovement(int VPcell, int grid_size)
{
    geometry_msgs::Twist tmp;
    if (VPcell != 0)
    {
        tmp.linear.x = 0.5;
        tmp.angular.z = double(VPcell - (grid_size + 1)/2 )*twist_const/4;
        return tmp;
    }
    else
    {
        tmp.linear.x=0;
        tmp.angular.z=0;
        return tmp;
    }
}

void processFrame(cv::Mat curr_frame)
{

    frame = curr_frame;
    width = frame.cols;
    height = frame.rows;

    frame_org = frame;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    blur(frame_gray, frame_gray, Size(3, 3));
    cvtColor(frame_gray, frame, CV_GRAY2BGR);
    Canny(frame, frame_canny, 50, 200);


// probabilistic hough transform
    if (houghP == 1){
     HoughLinesP(frame_canny, linesP, 1, CV_PI/180, 100, 40, 40);
     verifyLine(linesP);
     extendLines(linesP);
     for( size_t i = 0; i < linesP.size(); i++ )
     {
         Point pt1(linesP[i][0], linesP[i][1]);
         Point pt2(linesP[i][2], linesP[i][3]);

         clipLine(frame_gray.size(), pt1, pt2);
         line(frame_canny, pt1, pt2, Scalar(255,0,0),2);
     }

     for( size_t i = 0; i < linesP.size(); i++ )
     {
         for (int j = i+1; j < linesP.size()-1; j++)
         {
             Point pt1(linesP[i][0], linesP[i][1]);
             Point pt2(linesP[i][2], linesP[i][3]);
             Point pt3(linesP[j][0], linesP[j][1]);
             Point pt4(linesP[j][2], linesP[j][3]);
             pdd pdPoint = lineLineIntersection(make_pair(pt1.x, pt1.y), make_pair(pt2.x, pt2.y), make_pair(pt3.x, pt3.y), make_pair(pt4.x, pt4.y));
             if (pdPoint.first < width && pdPoint.first > 0 && pdPoint.second < height && pdPoint.second > 0)
                intersections.push_back(Point(pdPoint.first, pdPoint.second));
         }
     }

    }

// standard hough transform
    else{
        HoughLines(frame_canny, lines, 1, CV_PI/360, 100);
        std::cout<<"lines.size()="<<linesP.size()<<std::endl;
        for( size_t i = 0; i < linesP.size(); i++)
        {
            float rho = linesP[i][0];
            float theta = linesP[0][i];

            double a = cos(theta);
            double b = sin(theta);
            double x0 = a*rho;
            double y0 = b*rho;

            Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

            clipLine(frame_gray.size(), pt1, pt2);
            if (!frame_gray.empty())
                line(frame_canny, pt1, pt2, Scalar(255,0,0),1,8);
        }
    }


    // detecting VP every 5th frame updates in order to avoid noise
    counter++;
    if (counter==frames_delay)
    {
//        cout<<"clearing"<<endl<<endl;
        VPcell = findVanishingPointCell(width/grid_size, intersections);
        intersections.clear();
        counter=0;
//        cout<<"VPcell="<<VPcell<<endl;

    }

    drawPoints(frame, intersections);
    drawGrid(frame, grid_size, VPcell);

    //detectVP(0,0);
    imshow("canny", frame_canny);
    imshow("frame_gray", frame_gray);
    imshow("points", frame);

//    if (waitKey(33) >= 0) break;
}

void drawPoints(cv::Mat &myframe, vector<Point> pts)
{
    for (int i=0; i<pts.size(); i++)
    {
        circle(myframe, pts.at(i), 2, Scalar(255,0,0), 2);
    }
}

void drawGrid(cv::Mat &myframe, int grid_size, int VPcell)
{
    int thickness = 1;
    for (int i = 0; i<=grid_size; i++)
    {
        int x = (myframe.cols/grid_size)*(i);
        if (i==VPcell-1 || i==VPcell){
            cv::line(myframe, Point(x, 0), Point(x, myframe.rows), Scalar(0,255,0), thickness+3);
        }
        else
            cv::line(myframe, Point(x, 0), Point(x, myframe.rows), Scalar(0,255,0), thickness);
    }
}

pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D)
{
    // Line AB represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.second - C.second;
    double b2 = C.first - D.first;
    double c2 = a2*(C.first)+ b2*(C.second);

    double determinant = a1*b2 - a2*b1;
    double first_line_degree = (double)atan(double(a1)/(double(b1))) * 180/PI;
    double second_line_degree = (double)atan(double(a2)/(double(b2))) * 180/PI;


    if (determinant == 0 || (first_line_degree - second_line_degree < min_deg_between_lines))
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return make_pair(FLT_MAX, FLT_MAX);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        return make_pair(x, y);
    }
}

void extendLines(std::vector<Vec4i> &linesP){
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Point vec = Point(linesP[i][0]-linesP[i][2] , linesP[i][1]-linesP[i][3]);
        linesP.at(i) = Vec4i(linesP[i][0] + 100*vec.x, linesP[i][1] + 100*vec.y,
                linesP[i][2] - 100*vec.x, linesP[i][3] - 100*vec.y);
    }


}

int findVanishingPointCell(int grid_pixels, vector<Point> intersections)
{

    if (intersections.empty())
        return 0;

//    int grid_rows = (height / 3);
    int grid_columns = ( width/ grid_pixels);
    int VP[grid_columns] = {0,0,0,0,0};

    for (int i = 1; i < grid_columns + 1; i++)
    {
        for (vector<Point>::iterator p = intersections.begin(); p!=intersections.end(); ++p)
        {
            if (p->x < i*grid_pixels && p->x >= (i-1)*grid_pixels)
            {
                VP[i-1]++;
            }
        }
    }


    int best_cell = 0;
    int best_result = 0;

    for (int i = 0; i < grid_columns; i++)
    {
        if (VP[i] > best_result){
            best_cell = i;
            best_result = VP[i];
        }
    }

//    cout<<"grid_columns="<<grid_columns<<endl;
    return best_cell + 1;
}

void verifyLine(vector<Vec4i>& currLines){
    int counter = 0;
    for( size_t i = 0; i < currLines.size(); i++ ){
        Vec4i l = currLines[i];
        int dx = l(0)-l(2);
        int dy = l(1)-l(3);
        ///different x position ~dividinig by delta x
        if (dx!=0){
            if (atan(abs(dy/dx)) > maxLineAngle * PI/180 ||
                    (double)atan(abs(double(dy))/abs(double(dx))) < minLineAngle * PI/180 ){
                currLines.erase(currLines.begin() + i);
                i--;
                counter++;
            }
        }
        else {
            currLines.erase(currLines.begin() + i);
            i--;
        }
    }

//    cout<<"deleted lines: "<<counter<<endl;
}


/// --------------- OLD ----------------
///
///
int findInliers(const vector<Vec2f>& lines, const Point& intersectingPt, Mat tempImg)
{
  int inliers = 0;
  //print//cout << "Distance: ";
  for (int i = 0; i < static_cast<int>(lines.size()); i++) {
    Mat tmp = tempImg.clone();
    // find error: shortest distance between intersectingPt and line
    float r = lines[i][0], t = lines[i][1];
    double a = cos(t), b = sin(t);
    int x = intersectingPt.x, y = intersectingPt.y;
    double d = abs(a*x + b*y - r) / sqrt(pow(a,2) + pow(b,2));

     // debugging
     double alpha = 1000;
     double cos_t = cos(t), sin_t = sin(t);
     double x0 = r*cos_t, y0 = r*sin_t;
     Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
     Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
     line( tmp, pt1, pt2, Scalar(0,255,255), 2, CV_AA);

//     imshow("temp_", tmp);
//     waitKey(0);

    // find inliers
    if (d < threshold_ransac) { inliers++; }
  }
  //print//cout << endl;
  return inliers;
}

/* -------------------------------------- findIntersectingPt --------------------------------------------*/
// find intersecting point between two lines using crammer's rule.
// if no intersecting point (parallel lines/same line) return false
// i/p: lines: [rho_1;theta_1] & [rho_2;theta_2] and intersectingPt
bool findIntersectingPoint(float r_1, float t_1, float r_2, float t_2, Point& intersectingPt)
{
  double determinant = (cos(t_1) * sin(t_2)) - (cos(t_2) * sin(t_1));
  if (determinant != 0) {
    intersectingPt.x = (int) (sin(t_2)*r_1 - sin(t_1)*r_2) / determinant;
    intersectingPt.y = (int) (cos(t_1)*r_2 - cos(t_2)*r_1) / determinant;
    return true;
  }
  // else no point found (parallel lines/same line)
  return false;
}

/* -----------------------------------------LPF --------------------------------------------*/
 // // low pass parameters
 // const int lowPass_window = 20;
 // int window [lowPass_window][2] = {};
 // int lp_pointer = 0;
 // int sum [2] = {};
int* lowPass (int vp_x, int vp_y)
{
  // remove element from running sum
  running_sum[0] -= window[lp_pointer][0];
  running_sum[1] -= window[lp_pointer][1];

  // update window with new vanishing point
  window[lp_pointer][0] = vp_x;
  window[lp_pointer][1] = vp_y;

  // update sum
  running_sum[0] += window[lp_pointer][0];
  running_sum[1] += window[lp_pointer][1];

  // update running avg
  int *avg = (int *) malloc(sizeof (int) * 2);
  avg[0] = (int) (float)running_sum[0]/lowPass_window;
  avg[1] = (int) (float)running_sum[1]/lowPass_window;

  // increment lp_pointer and keep within bounds
  lp_pointer++;
  if (lp_pointer >= lowPass_window) lp_pointer = 0;

  return avg;
}

void detectVP(int, void *){
// 3. RANSAC if > 2 lines available
if (lines.size() > 1)
{
    cout<<"ransac";
  Mat tempImg;

  int maxInliers = 0;
  Point vp, vp_lp;
  for (int i = 0; i<N_iterations; i++)
  {
    tempImg = frame.clone();;
    // 1. randomly select 2 lines
    int a = rand() % static_cast<int>(lines.size());
    int b = rand() % static_cast<int>(lines.size());

    // 2. find intersecting point (x_v, y_v)
    Point intersectingPt;
    float r_1 = lines[a][0], t_1 = lines[a][1];
    float r_2 = lines[b][0], t_2 = lines[b][1];
    bool found= findIntersectingPoint(r_1, t_1, r_2, t_2, intersectingPt);

    // skip if not found
    if (!found) continue;

  // else
  // {
  //   //print//cout << "Point: " << intersectingPt.x << ", " << intersectingPt.y << endl;
  // }

  // // // draw for debugging purposes
  // double alpha = 1000;
  // double cos_t = cos(t_1), sin_t = sin(t_1);
  // Point pt1( cvRound(r_1*cos_t + alpha*(-sin_t)), cvRound(r_1*sin_t + alpha*cos_t) );
  // Point pt2( cvRound(r_1*cos_t - alpha*(-sin_t)), cvRound(r_1*sin_t - alpha*cos_t) );
  // line(tempImg, pt1, pt2, Scalar(255,255,0), 2, CV_AA);

  // cos_t = cos(t_2); sin_t = sin(t_2);
  // Point pt11( cvRound(r_2*cos_t + alpha*(-sin_t)), cvRound(r_2*sin_t + alpha*cos_t) );
  // Point pt22( cvRound(r_2*cos_t - alpha*(-sin_t)), cvRound(r_2*sin_t - alpha*cos_t) );
  // line(tempImg, pt11, pt22, Scalar(255,255,0), 2, CV_AA);

  // circle(tempImg, intersectingPt, 2,  Scalar(0,0,255), 2, 8, 0 );
  // imshow("temp", tempImg);
  // waitKey(0);

    // 3. find error for each line (shortest distance b/w point above and line: perpendicular bisector)
    // 4. find # inliers (error < threshold)
    int inliers = findInliers(lines, intersectingPt, tempImg);
    //print//cout << "Num of inliers: " << inliers << endl;

    // 5. if # inliers > maxInliers, save model
    if (inliers > maxInliers)
    {
      maxInliers = inliers;
      vp.x = intersectingPt.x;
      vp.y = intersectingPt.y;
    }

  } // end of ransac iterations

  // limit vanishing point to be within image bounds
  if (vp.x > width) vp.x = width;
  if (vp.y > height) vp.y = height;

  // apply low pass filter over frames
  int *avg = lowPass (vp.x, vp.y);
  vp_lp.x = *(avg);
  vp_lp.y = *(avg + 1);
  free(avg); // free avg to avoid memory leaks

  // compute error signal
  int error = (vp_lp.x - cvRound(width/2.0));
  cout << "Vanishing point = " << vp.x << "," << vp.y << "| Inliers: " << maxInliers << "| error: "<< error << endl;

  // plot vanishing point on images
  circle(frame, vp, 2,  Scalar(0,0,255), 2, 8, 0 );
  circle(frame_org, vp, 3,  Scalar(0,0,255), 2, 8, 0 );
  circle(frame, vp_lp, 2,  Scalar(0,255,0), 2, 8, 0 );
  circle(frame_org, vp_lp, 3,  Scalar(0,255,0), 2, 8, 0 );
  cout<<vp.x<<endl;

} // end of ransac (if > 2 lines available)
// draw cross hair
Point pt1_v( cvRound(width/2.0), 0);
Point pt2_v( cvRound(width/2.0), height);
line( frame, pt1_v, pt2_v, Scalar(0,255,255), 1, CV_AA);
Point pt1_h( 0, cvRound(height/2.0));
Point pt2_h( width, cvRound(height/2.0));
line( frame, pt1_h, pt2_h, Scalar(0,255,255), 1, CV_AA);

imshow( "frame_modified", frame );
imshow("Original", frame_org);
}
