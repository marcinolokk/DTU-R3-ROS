#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>


using namespace cv;

int crop_width = 50;
int crop_height = 50;

int crops_norm(Mat, Mat);
std::vector<uchar> matTo1d(Mat);


int main() {
    VideoCapture stream1(1);
    if (!stream1.isOpened()){
		std::cout << "cannot open camera";
		return 1;
	}

	namedWindow("Camera output",1);
	Mat frame, frame_gray, frame_canny;
	std::vector<Vec2f> linesP;
	std::vector<Vec2f> lines;

	while (true)
	{
		stream1 >> frame;
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
		//blur(frame_gray, frame_gray, Size(3, 3));
		Canny(frame_gray, frame_canny, 50, 200, 3);


		HoughLines(frame_canny, lines, 1, CV_PI / 90, 120);
		//HoughLinesP(frame_canny, linesP, 1, CV_PI / 90, 120);

        /*
		for (size_t i = 0; i < lines.size(); i++)
		{
			float rho = lines[i][0], theta = lines[i][1];
			//if (( 360*(theta/CV_PI) > 180 || 360 * (theta / CV_PI) < 60)) {
			//	std::cout << 360*theta / CV_PI << "  " << std::endl;
			//	//system("pause");
			//	continue;
			//}
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(frame, pt1, pt2, Scalar(0, 0, 255), 2, CV_AA);
		}
        */

		imshow("Camera output", frame);
		if (waitKey(33) >= 0) break;
	}
	/*
	IplImage *iplimg;

	Mat img_r = cv::imread("pic1_r.png", 0);
	Mat img_l = cv::imread("pic1_l.png", 0);
	imshow("l", img_l);
	imshow("r", img_r);

	if (crop_width > img_r.cols || crop_height > img_r.rows) {
		std::cout << "crop size too big";
		waitKey(0);
		return 0;
	}

	Rect crop = Rect(0, 0, crop_width, crop_height);
	Mat cropped_org = Mat(img_r, crop);


	//Mat cropped_1 = Mat(crop_height, crop_width, img_r.type());
	Mat crop_1 = Mat(img_l, crop);
	Mat crop_2 = Mat(img_l, Rect(img_l.cols - crop_width,	0,							crop_width, crop_height));
	Mat crop_3 = Mat(img_l, Rect(img_l.cols - crop_width,	img_l.rows - crop_height,	crop_width, crop_height));
	Mat crop_4 = Mat(img_l, Rect(0,							img_l.rows - crop_height,	crop_width, crop_height));

	imshow("crop_1", crop_1);
	imshow("crop_2", crop_2);
	imshow("crop_3", crop_3);
	imshow("crop_4", crop_4);


	int m1 = crops_norm(cropped_org, crop_1);
	int m2 = crops_norm(cropped_org, crop_2);
	int m3 = crops_norm(cropped_org, crop_3);
	int m4 = crops_norm(cropped_org, crop_4);

	std::cout << "  m1: " << m1 << "  m2: " << m2 << "  m3: " << m3 << "  m4: " << m4;

	waitKey(0);


*/
	}

int crops_norm(Mat crop_org, Mat crop) {
	int m=0;
	std::vector<uchar> crop_org_vec = matTo1d(crop_org);
	std::vector<uchar> crop_vec = matTo1d(crop);
	
	for (int i = 0; i < crop_org_vec.size(); i++)
		m += int(abs(crop_org_vec.at(i) - crop_vec.at(i)));
	return m;
}

std::vector<uchar> matTo1d(Mat mat) {
	std::vector<uchar> mat_vec;
	for (int i = 0; i < mat.rows; i++)
		for (int j = 0; j < mat.cols; j++)
			mat_vec.push_back(mat.at<uchar>(Point(i, j)));
	return mat_vec;
}
