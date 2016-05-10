//Credits to Bryan Chung at http://www.magicandlove.com/blog/2011/08/26/people-detection-in-opencv-again/

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <vector>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
 
using namespace std;
using namespace cv;

#define OPENCV_WINDOW "Test Window"
#define OUT_WINDOW "Out Window"

#define MIN_DIFF_FOR_MOVE 25
#define MAX_YAW_VELOCITY .5

#define MIN_AREA_DIFF 50
#define MAX_LINEAR_VELOCITY .75

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	HOGDescriptor hog;  
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // <---- might cause a compile error

	double prevArea;
	
	Rect findBestMatchForPreviousHistogram(cv::Mat img, vector<Rect> foundRects)
	{
		return foundRects[0];
	}

public:
	//constructor, creating an ImageConev
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, 
    								&ImageConverter::imageCb, this);
    	
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

		prevArea = -1.0;

		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow(OUT_WINDOW);
  	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OUT_WINDOW);
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
    
 
		cv::Mat Img;
		Img = cv_ptr->image.clone(); 


		vector<Rect> found, found_filtered;
		hog.detectMultiScale(Img, found, 0, Size(8,8), Size(32,32), 1.05, 2);

		size_t i, j;
		for (i=0; i<found.size(); i++)
		{
			Rect r = found[i];
			for (j=0; j<found.size(); j++)
				if (j!=i && (r & found[j])==r)
					break;
			if (j==found.size())
				found_filtered.push_back(r);
		}

		for (i=0; i<found_filtered.size(); i++)
		{
			Rect r = found_filtered[i];
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.06);
			r.height = cvRound(r.height*0.9);
			rectangle(Img, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
		}

		//person location/movement code
		double rectangleCenter;

		if (!found_filtered.empty())
		{
			Rect bestFit = findBestMatchForPreviousHistogram(Img, found_filtered);

			rectangleCenter = (bestFit.x + bestFit.width/2);
			double rectangleArea; //used to determine how far the person is from the robot
			rectangleArea = bestFit.width * bestFit.height;
	
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.header.frame_id = "/base_link";

			double middleX = ((double)Img.cols)/2;
	
			double difference = rectangleCenter - middleX

			double yaw = 0.0;

			if (difference < -MIN_DIFF_FOR_MOVE || difference > MIN_DIFF_FOR_MOVE)
			{
				yaw = MAX_YAW_VELOCITY * difference / middleX;
			}

			double nextX = 0.0

			if (prevArea > 0)
			{
				double areaDifference = rectangleArea - prevArea;

				if (areaDifference < -MIN_AREA_DIFF || areaDifference > MIN_AREA_DIFF)
				{
					nextX = MAX_LINEAR_VELOCITY * areaDifference / prevArea;
				}
			}

			goal.target_pose.pose.position.x = nextX;
			goal.target_pose.pose.position.y = 0.0;
			goal.target_pose.pose.position.z = 0.0;
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

			prevArea = rectangleArea;
		}

		found.clear();
		found_filtered.clear();

		cv::imshow(OUT_WINDOW, Img);

		if (waitKey(20) >= 0)
		{
			return;
		}

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

 
int main (int argc, char * argv[])
{
	ros::init(argc, argv, "PersonTracking");
	ImageConverter ic;
	ros::spin();

	return 0;
}
