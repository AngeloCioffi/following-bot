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
#define PI 3.14159265359


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";
float x_vals;
float y_vals;
float num_rows;
float num_cols;
float threshold = PI/10;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  
public:
	//constructor, creating an ImageConev
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, 
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
    
    
    //declare output image
    cv::Mat outImg;
	//cv::Mat original_image = cv_ptr->image;
	
	//whenever the object is a pointer, use the -> to replace the . to access and modify the object
	
	
	outImg = cv_ptr->image.clone(); 
	
	std::vector<cv::Vec2i> pts;
	
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			//vec3 stores 3 values, BGR, this line is grabbing the value at [0] for point i,j for the vec3
			//outImg.at<cv::Vec3b>(i,j)[0] = 0;  //set blue to 0
			int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
			int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
			int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			
			if((r_ij>=215 && r_ij<=240) && (g_ij>=5 && g_ij<=25) && (b_ij>=140 && b_ij<=170))
			{
				pts.push_back(cv::Vec2i(i, j));
				//outImg.at<cv::Vec3b>(i,j)[0] = 255;
				//outImg.at<cv::Vec3b>(i,j)[1] = 255;
				//outImg.at<cv::Vec3b>(i,j)[2] = 255;
			
			}
			else
			{
				outImg.at<cv::Vec3b>(i,j)[0] = 0;
				outImg.at<cv::Vec3b>(i,j)[1] = 0;
				outImg.at<cv::Vec3b>(i,j)[2] = 0;
			}
		
		}
	}
	
	
		cv::Vec2f mid_pnt;
		if(pts.size() == 0)
		{
			mid_pnt[0] = -1;
			mid_pnt[1] = -1;
		}
		else
		{
		
		for(unsigned int i = 0; i < pts.size(); i++)
		{
			
				mid_pnt[0] += pts[i][0]/ pts.size();
				mid_pnt[1]+=pts[i][1]/pts.size();
		
		}}
		x_vals = mid_pnt[1];
		y_vals = mid_pnt[0];
		
	//make the diameter a fixed size or square root of the size
	cv::circle(outImg, cv::Point(mid_pnt[1], mid_pnt[0]), 40, CV_RGB(0,255,0));
	//show input
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
	//show output
	cv::imshow(OUT_WINDOW, outImg);
    
	//pause for 3 ms
    cv::waitKey(3);
    
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
   /* actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
 
  move_base_msgs::MoveBaseGoal goal;
  
  
  double x = 0;
  double yaw = 0;
  
  while (ros::ok())
  {
    //set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_link";
    float center_x = num_rows/2;
    float center_y = num_cols/2;
    
    if(( (center_x - x_vals < threshold) || (x_vals - center_x < threshold)) || x_vals == 0)
		{
			yaw = 0;
		}
    
    
    //entire thing is 57 degrees 
    //left half is 28 degrees
    //right half is 28 degrees
    else if (x_vals < center_x && x_vals > (num_rows/4))
    {
		yaw = 0.24;
		//ROS_INFO("Yaw: " + yaw);
		
	}
	
	else if (x_vals < center_x)
    {
	
			yaw = 0.48;
			//ROS_INFO("Yaw: " + yaw);
	}
	
	else if (x_vals > center_x && x_vals  < (3*num_rows)/4)
	{
			yaw = -0.24;
			//ROS_INFO("Yaw: " + yaw);
	}
	
	else
	{
			
			yaw = -0.48;
			//ROS_INFO("Yaw: " + yaw);
		
	}	
		//ROS_INFO("Hat position: " + x_vals);
		//ROS_INFO("Yaw: " + yaw);
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	

	//send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();
   ros::spinOnce();

  }*/
  
    
    
    
    
  }
};






int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
 
  move_base_msgs::MoveBaseGoal goal;
  
  
  double x = 0;
  double yaw = 0;
  
  while (ros::ok())
  {
    //set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_link";
    float center_x = num_rows/2;
    float center_y = num_cols/2;
    
    if(( (center_x - x_vals < threshold) || (x_vals - center_x < threshold)) || x_vals == 0)
		{
			yaw = 0;
		}
    
    
    //entire thing is 57 degrees 
    //left half is 28 degrees
    //right half is 28 degrees
    else if (x_vals < center_x && x_vals > (num_rows/4))
    {
		yaw = 0.24;
		//ROS_INFO("Yaw: " + yaw);
		
	}
	
	else if (x_vals < center_x)
    {
	
			yaw = 0.48;
			//ROS_INFO("Yaw: " + yaw);
	}
	
	else if (x_vals > center_x && x_vals  < (3*num_rows)/4)
	{
			yaw = -0.24;
			//ROS_INFO("Yaw: " + yaw);
	}
	
	else
	{
			
			yaw = -0.48;
			//ROS_INFO("Yaw: " + yaw);
		
	}	
		//ROS_INFO("Hat position: " + x_vals);
		//ROS_INFO("Yaw: " + yaw);
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	

	//send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    //ac.waitForResult();
   ros::spinOnce();

  }
  
  return 0;
}















