#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>
#include <stdlib.h>


// Variables
int leftSlider, leftSliderMax;
int rightSlider, rightSliderMax;
int topSlider, topSliderMax;
int bottomSlider, bottomSliderMax;
int widthSlider, widthSliderMax;
int heightSlider, heightSliderMax;
size_t imgWidth;
size_t imgHeight;
cv::Mat image;
size_t count;
std::string dirname;
std::string objectName;
int width, height;

void imageCallback(const sensor_msgs::ImageConstPtr imageMsg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
	image = cv_ptr->image;

	int left = leftSlider;
	int width = left + widthSlider < imgWidth ? widthSlider : imgWidth - left;

	int top = topSlider;
	int height = top + heightSlider < imgHeight ? heightSlider : imgHeight - top;

	cv::Rect roi = cv::Rect(left, top, width, height);
	//ROS_INFO_STREAM(left << ", " << width << ", " << top << ", " << height);
	image = image(roi);

	cv::imshow("Image", image);
	int key = cv::waitKey(1);
        //std::cout << key << "\n";
	if (key == 32) // space bar
	{
		std::stringstream ss;
		ss << dirname << "/" << count << ".jpg";
		cv::imwrite(ss.str(), image);
		ROS_INFO_STREAM("Saving image: " << count);
		++count;
	}

}

void onLeftSliderChange(int, void*)
{
	//ROS_INFO_STREAM(leftSlider);
}

void onRightSliderChange(int, void*)
{
//	ROS_INFO_STREAM(rightSlider);
}

void onTopSliderChange(int, void*)
{
	height = topSlider;
//	ROS_INFO_STREAM(topSlider);
}

void onBottomSliderChange(int, void*)
{
//	ROS_INFO_STREAM(bottomSlider);
}

void onWidthSliderChange(int, void*)
{
	if (widthSlider <= 0)
	{
		widthSlider = 1;
		cv::setTrackbarPos("width", "Controls", widthSlider);
	}
	/*
	if (xSlider > imgWidth - width)
	{
		xSlider = imgWidth - width;
		cv::setTrackbarPos("x", "Controls", xSlider);
	}
	if (xSlider < width)
	{
		xSlider = width;
		cv::setTrackbarPos("x", "Controls", xSlider);
	}
	*/

}

void onHeightSliderChange(int, void*)
{
	if (heightSlider <= 0)
	{
		heightSlider = 1;
		cv::setTrackbarPos("height", "Controls", heightSlider);
	}
	/*
	if (ySlider > imgHeight - height)
		{
			ySlider = imgHeight - height;
			cv::setTrackbarPos("y", "Controls", ySlider);
		}
		if (ySlider < height)
		{
			ySlider = height;
			cv::setTrackbarPos("y", "Controls", ySlider);
		}
	*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_modeler");
	ros::NodeHandle nh("object_modeler");

	nh.getParam("save_folder", dirname);
	//mkdir(dirname.c_str());
	std::stringstream cdir;
	cdir << "mkdir -p " << dirname.c_str();
	system(cdir.str().c_str());
        std::cout << "Save folder: " << dirname << "\n";

	nh.getParam("object", objectName);

	std::stringstream ss;
	ss << dirname << objectName;
	mkdir(ss.str().c_str(), S_IRWXU);
	dirname = ss.str();

	std::string topicName;
	nh.getParam("topic", topicName);

	if (topicName.find("camera") != std::string::npos)
	{
		imgWidth = 640;
		imgHeight = 480;
	}
	else
	{
		imgWidth = 1920;
		imgHeight = 1080;
	}

	nh.getParam("width", width);
	nh.getParam("height", height);

	cv::namedWindow("Controls", 1);

	count = 0;

	leftSlider = 0;
	leftSliderMax = imgWidth - 1;
	cv::createTrackbar("Left", "Controls", &leftSlider, leftSliderMax, onLeftSliderChange);

	topSlider = 0;
	topSliderMax = imgHeight - 1;
	cv::createTrackbar("Top", "Controls", &topSlider, topSliderMax, onTopSliderChange);

	widthSlider = imgWidth;
	widthSliderMax = imgWidth - 1;
	cv::createTrackbar("width", "Controls", &widthSlider, widthSliderMax, onWidthSliderChange);

	heightSlider = imgHeight;
	heightSliderMax = imgHeight - 1;
	cv::createTrackbar("height", "Controls", &heightSlider, heightSliderMax, onHeightSliderChange);

	ros::Subscriber imageSub;
	imageSub = nh.subscribe<sensor_msgs::Image>(topicName, 1, imageCallback);

	ros::spin();
}
