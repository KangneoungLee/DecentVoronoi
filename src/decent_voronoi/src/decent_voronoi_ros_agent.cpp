/*********************************************************************
MIT License

Copyright (c) 2022 Kangneoung Lee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 * Author: Kangneoung Lee
 *********************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <sensor_msgs/CameraInfo.h>
#include <boost/bind.hpp>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <decent_voronoi/robot_info.h>

#include "decent_voronoi/decent_voronoi.h"

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

#define NO_INFORMATION_CELL -1
#define FREE_CELL 0

/************Function architecture*******************/
std::ofstream PartInfoTxt;
std::mutex _lock;
int global_cnt = 0;

struct agent_info
{
	float x_m;
	float y_m;
	int index;
	bool is_renew;
	bool is_real;   // 0 : virtual agent, 1: real agent
	bool is_dropout;
	float robot_cov_radius;
	float coverage_metric;
};

struct agent_prev_goal
{
	float prev_x_m;
	float prev_y_m;
	int index;
};

class DecentVoronoiRosAgent{
	
	private:
		 ros::NodeHandle main_nh_;
         ros::NodeHandle param_nh_;
	     ros::Rate* loop_rate_;

		 //image_transport::Subscriber depth_sub_;
		 //ros::Subscriber rgb_cam_info_sub_;
		 //ros::Subscriber depth_cam_info_sub_;
		 
		 int map_height_;
		 int map_width_;
		 
		 int robot_index_ = 0;
		 float robot_maxspd_;
		 float robot_cov_radius_; 
		 float min_dist_goal_send_;
		 int explore_enable_cnt_th_;
		 
		 bool is_img_save_; 
		 std::string img_save_dir_;
		 std::string img_color_label_dir_;
		 float map_resolution_;
		 
		 bool is_in_obs_ = false;
		 		 
		 bool y_axis_swap_;
		 float max_agent_num_;
		 int no_agent_internal_param_;
		 
		 unsigned char* densityPtr_ = NULL;
		 
		 bool InhbtDropout_ = true;
		 bool is_dropout_use_ = false;
		 
		 DecentVoronoi* DecentVoroHandle_;
		 
		 bool display_debug_msg_ = false;
		 bool ego_pose_ok_ = false;
		 bool ego_is_dropout_ = false;
		 float ego_pos_x_;
		 float ego_pos_y_;
		 
		 float dropout_terminate_criteria_ = 0.001;
		 float coverage_metric_ = 10;
		 float coverage_metric_prev_;
		 float coverage_metric_diff_prev_ = 100;
		 float coverage_metric_diff_ ;
		 float cm_diff_rate_ = 0.1;
		 
		 std::vector<float> coverage_metric_vec_; 
		 
		 ros::Time init_time_;
		 
		 bool text_log_enable_;
		 std::string text_log_dir_;
		 std::ofstream txt_file_;
		 
		 float update_rate_;
		  
		 std::vector<ros::Subscriber> RobotInfoSubVec_;
		 ros::Subscriber pose_sub_;
		 ros::Publisher robot_info_pub_;
		 ros::Publisher goal_pub_;
		 
		 struct agent_info* agent_info_array_;
		 struct agent_info* agent_info_array_prev_;
		 	
	public:
	
	    void run();
		
		void read_density(unsigned char*  density_array, std::string img_dir, int height, int width);
		void IntializedAgents();
		void RealTimeVoroProcess(int cur_robot_num);
		void RobotInfoSubCallback(const decent_voronoi::robot_info::ConstPtr& msg , int index);
		void PosSubCallback(const nav_msgs::Odometry::ConstPtr& msg, int index);

	 
		/*constructor and destructor*/
	    DecentVoronoiRosAgent(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~DecentVoronoiRosAgent();
	

}; // class DynamicVoronoiRosSimpleGoal end

DecentVoronoiRosAgent::DecentVoronoiRosAgent(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh_(m_nh),param_nh_(p_nh)
{

	 float update_rate = 10;  //hz
	
	 param_nh_.getParam("update_rate",update_rate);

	 this->update_rate_ = update_rate;


     this->loop_rate_ = new ros::Rate(this->update_rate_);	


	 float map_resolution = 0.1;	 
     param_nh_.getParam("map_resolution",map_resolution);	 
	 this->map_resolution_ = map_resolution;
	 	 
	
	 int map_height = 200;
	 int map_width = 200;	 
	 	 
   	 param_nh_.getParam("map_height",map_height);
	 param_nh_.getParam("map_width",map_width);
	 
     this->map_height_ = map_height;
	 this->map_width_ = map_width;
	 
	 float robot_maxspd = 0.8;
	 float robot_cov_radius = 16;
	 int robot_index = 0;
	 
	 param_nh_.getParam("robot_max_speed",robot_maxspd);
	 param_nh_.getParam("robot_cov_radius",robot_cov_radius);
	 param_nh_.getParam("robot_index",robot_index);
	 
	 this->robot_maxspd_ = robot_maxspd;
	 this->robot_cov_radius_ = robot_cov_radius;
	 this->robot_index_ = robot_index;

	 float min_dist_goal_send = 0.5; // 0.5m

	 param_nh_.getParam("min_dist_goal_send",min_dist_goal_send);
	 
     this->min_dist_goal_send_ = min_dist_goal_send;
	 this->explore_enable_cnt_th_ =  5;

	 bool is_hete_cov_radius = false;
	 bool is_dropout_use = true;
	 bool is_propa_connected_area = false;
	 bool is_img_save = false;
     std::string img_save_dir = "/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/opt_animation/";
	 std::string img_color_label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";
	 bool work_eff_flag = false;	
	 bool is_uniform_density = false;
	
	 float max_agent_num = 20; 
	 bool y_axis_swap = true;
	
	 param_nh_.getParam("is_hete_cov_radius",is_hete_cov_radius);
	 param_nh_.getParam("is_dropout_use",is_dropout_use);
	 param_nh_.getParam("is_img_save",is_img_save);
	 param_nh_.getParam("img_save_dir",img_save_dir);
	 param_nh_.getParam("img_color_label_dir",img_color_label_dir);
	 param_nh_.getParam("is_propa_connected_area",is_propa_connected_area);
	 
	 bool display_debug_msg = false;
	 param_nh_.getParam("display_debug_msg",display_debug_msg);
	 this->display_debug_msg_ = display_debug_msg;

	 param_nh_.getParam("max_agent_num",max_agent_num);
	 param_nh_.getParam("y_axis_swap",y_axis_swap);
	 // if y_axis_swap is true, map coordinate in dynamic voronoi follows image coordinate (E : x, S :y) and real agent follows ENU coordinate (E : x, N : y)
	 // if y_axis_swap is false, map coordinate in dynamic voronoi and real agents follow image coordinate (E : x, S :y)
	 
	 this->is_img_save_ = is_img_save;
	 this->img_save_dir_ = img_save_dir;
	 this->img_color_label_dir_ = img_color_label_dir;
	 this->y_axis_swap_ = y_axis_swap;
	 this->max_agent_num_ = max_agent_num;
	 
	 if(is_dropout_use == true)  this->InhbtDropout_ =  false; 
	 else this->InhbtDropout_ = true;
	 this->is_dropout_use_ = is_dropout_use;
	 
	 float DropOutWeight_init = (float)this->max_agent_num_/(float)(this->max_agent_num_-1);
	 
	 
	 float dropout_terminate_criteria = 0.002;
	 param_nh_.getParam("dropout_terminate_criteria", dropout_terminate_criteria);
	 this->dropout_terminate_criteria_ = dropout_terminate_criteria;
	 
	 float cm_diff_rate = 0.2;
	 param_nh_.getParam("low_pass_filter_coeff", cm_diff_rate);
	 this->cm_diff_rate_ = cm_diff_rate;
	 
	 std::string density_img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/density/density_200_200_ver3.png"; 
     param_nh_.getParam("density_img_dir",density_img_dir);
	 
	 bool text_log_enable = true;
	 param_nh_.getParam("text_log_enable", text_log_enable);
	 this->text_log_enable_ = text_log_enable;
	 
	 std::string text_log_dir = "";
	 param_nh_.getParam("text_log_dir", text_log_dir);
	 text_log_dir = text_log_dir + "agent" + std::to_string(robot_index) + ".txt";
	 this->text_log_dir_ = text_log_dir;
	 this->txt_file_.open(this->text_log_dir_);
	 this->txt_file_ << " time " << "pos_x (m) "<< "pos_y (m) " << "is_dropout "   << "is_in_obs " <<   "robot_cov_radius " << "coverage_metric "  << "coverage_metric_diff " <<  std::endl;
	 
	 densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	 
	 this->read_density(densityPtr_, density_img_dir, this->map_height_ , this->map_width_); //read density map from image file

	 DecentVoroHandle_ = new DecentVoronoi(this->map_height_, this->map_width_, DropOutWeight_init, is_uniform_density, is_dropout_use, 
	                                                                       is_hete_cov_radius, is_propa_connected_area, is_img_save, this->display_debug_msg_, densityPtr_);
																		   
	 agent_info_array_ = new struct agent_info[static_cast<int>(this->max_agent_num_)];
	 agent_info_array_prev_ = new struct agent_info[static_cast<int>(this->max_agent_num_)];
	 
	 this->IntializedAgents();	
	 
	 /**generate subscriber for pose of robots, create publisher for goal, explore enable, costmap for exploration,  intiailize the cost map for exploration*/
	 std::string robot_str = "robot"; 
	 for(int index=0; index< this->max_agent_num_; index++)
	 {
		 if(index != this->robot_index_)
		 {
			 
			 std::string sub_topic = robot_str+std::to_string(index)+"/robot_info";
			 ros::Subscriber robot_info_sub = main_nh_.subscribe<decent_voronoi::robot_info>(sub_topic, 3, boost::bind(&DecentVoronoiRosAgent::RobotInfoSubCallback, this,_1,index));
			 RobotInfoSubVec_.push_back(robot_info_sub);
		 }			 
		
		float x_pos = 0;
		float y_pos = 0;
		float v_travel = 0;
		float cov_radius = 1;
		DecentVoroHandle_->PushPoint(x_pos, y_pos, v_travel, cov_radius); 
		
	 }

	 std::string sub_topic = robot_str+std::to_string(this->robot_index_)+"/odom";
	 this->pose_sub_ = main_nh_.subscribe<nav_msgs::Odometry>(sub_topic, 3, boost::bind(&DecentVoronoiRosAgent::PosSubCallback, this, _1, 0));
		
	 std::string pub_topic = robot_str+std::to_string(this->robot_index_)+"/robot_info";
	 this->robot_info_pub_ = main_nh_.advertise<decent_voronoi::robot_info>(pub_topic, 3);
	 
	 pub_topic = robot_str+std::to_string(this->robot_index_)+"/goal";
	 this->goal_pub_ = main_nh_.advertise<geometry_msgs::PoseStamped>(pub_topic, 3);
	 
	 this->coverage_metric_prev_ = this->coverage_metric_;
	 
	 this->init_time_ = ros::Time::now();
}

DecentVoronoiRosAgent::~DecentVoronoiRosAgent()
{
	
	delete this->loop_rate_;
	delete this->DecentVoroHandle_;
    delete[] this->densityPtr_;
	delete[] this->agent_info_array_;
	delete[] this->agent_info_array_prev_;
	 this->txt_file_.close();
}

void DecentVoronoiRosAgent::read_density(unsigned char*  density_array, std::string img_dir, int height, int width)
{
	cv::Mat density_img = imread(img_dir, cv::IMREAD_GRAYSCALE);
	
	cv::resize(density_img, density_img, cv::Size(width, height));
    cv::Mat  density_img_thresh;

    //cv::threshold(density_img, density_img_thresh, 254, 254, cv::THRESH_TRUNC);
    density_img_thresh = density_img.clone();
	
	cv::Mat mat_255(cv::Size(density_img_thresh.cols, density_img_thresh.rows), CV_8UC1, cv::Scalar(255));

	cv::Mat converted_density;
	cv::absdiff(mat_255, density_img_thresh, converted_density);

	cv::Mat density_resized;
	cv::resize(converted_density, density_resized, cv::Size(width, height));

	density_resized.convertTo(density_resized, CV_32FC1);

	for(int row=0; row<density_resized.rows; row++)  
	{
      for(int col=0; col<density_resized.cols; col++)
	  {
		   int index =col + row*width;
		   density_array[index] =  (unsigned char)density_resized.at<float>(row, col);
	  }
	}		
	if (this->display_debug_msg_ == true) std::cout<<" ************Read density information ***************" <<std::endl;
	
	return;
	
}

void DecentVoronoiRosAgent::IntializedAgents()
{
 
	 for (int i = 0; i < this->max_agent_num_; i++)
	 {
		 agent_info_array_[i].index = i;
		 agent_info_array_[i].is_renew = false;
		 agent_info_array_[i].is_dropout = false;
		 
	 }
}


void DecentVoronoiRosAgent::RobotInfoSubCallback(const decent_voronoi::robot_info::ConstPtr& msg , int index)
{
	if(agent_info_array_[index].is_renew == false)
	{
	    agent_info_array_[index].x_m = msg->odom.pose.pose.position.x;
	    agent_info_array_[index].y_m= msg->odom.pose.pose.position.y;
	    agent_info_array_[index].index =index;
		agent_info_array_[index].is_dropout = msg->is_dropout.data;
		agent_info_array_[index].robot_cov_radius = msg->robot_cov_radius.data;
		agent_info_array_[index].coverage_metric = msg->coverage_metric.data;
	    agent_info_array_[index].is_renew =true;
	}
}

void DecentVoronoiRosAgent::PosSubCallback(const nav_msgs::Odometry::ConstPtr& msg, int index)
{

	 ego_pos_x_ = msg->pose.pose.position.x;
	 ego_pos_y_= msg->pose.pose.position.y;
	 ego_pose_ok_ = true;
}



void DecentVoronoiRosAgent::RealTimeVoroProcess(int cur_robot_num)
{
    bool is_in_obs  = DecentVoroHandle_->CheckCurPosInObs();
	DecentVoroHandle_->ExpandedVoronoi(cur_robot_num);
	DecentVoroHandle_->CentroidCal();
	DecentVoroHandle_->MoveAgents();
	this->coverage_metric_ = DecentVoroHandle_->CoverageMetric();
	
	if (this->display_debug_msg_ == true) ROS_INFO("ego coverage metric : %lf,  cur robot num : %d", this->coverage_metric_, cur_robot_num);
	if(this->is_img_save_ == true)
	{
		std::string img_dir =this->img_save_dir_ + "robot" + std::to_string(this->robot_index_) + "_" + std::to_string(global_cnt) + ".png";
		DecentVoroHandle_->Colorized(img_dir, this->img_color_label_dir_);
	}
	DecentVoroHandle_->InitializeCell();
	bool dropout_precheck = DecentVoroHandle_->AgentDropOut();
	
	
	for(int i = 0; i < this->coverage_metric_vec_.size(); i++)
	{
		float coverage_metric = this->coverage_metric_vec_[i];
		this->coverage_metric_ = this->coverage_metric_ + coverage_metric;
	}
	
	if (this->display_debug_msg_ == true) ROS_INFO("sum of coverage metric : %lf,  cur robot num : %d", this->coverage_metric_, cur_robot_num);
	
	this->coverage_metric_ = this->coverage_metric_/cur_robot_num;
	this->coverage_metric_diff_ =  this->coverage_metric_prev_ - this->coverage_metric_;
	this->coverage_metric_diff_ = this->cm_diff_rate_* this->coverage_metric_diff_ + (1 - this->cm_diff_rate_)* this->coverage_metric_diff_prev_;
	
	
	if ((this->coverage_metric_diff_ >= this->dropout_terminate_criteria_)&&(this->InhbtDropout_ == false)&&(dropout_precheck == true))
	{
		this->ego_is_dropout_ = true;
	}
	else if((is_in_obs == false)&&(this->coverage_metric_diff_ < this->dropout_terminate_criteria_)&&(this->InhbtDropout_ == false))
	{
		this->InhbtDropout_ = true;
	}
	else
	{
		this->ego_is_dropout_ = false;
	}
	
	this->is_in_obs_ = is_in_obs;
	this->coverage_metric_prev_ = this->coverage_metric_;
	this->coverage_metric_diff_prev_ = this->coverage_metric_diff_;
	global_cnt++;
}


void DecentVoronoiRosAgent::run()
{  
    while(ros::ok())
	{		
		if((ros::Time::now()-this->init_time_  ) < ros::Duration(3.0))
		{
		     ros::spinOnce();
			 this->loop_rate_->sleep();
		}

		bool ready = true;
		int cur_robot_num = 1;
		int all_robot_num = 1;

		for(int index=0; index < this->max_agent_num_; index++)
		{		 
			 if(index == this->robot_index_)
			 {
				 float x_cell, y_cell;
				 x_cell = this->ego_pos_x_/this->map_resolution_;
				
				 if(this->y_axis_swap_ == true) y_cell = (float)this->map_height_ - this->ego_pos_y_/this->map_resolution_;
				 else y_cell = this->ego_pos_y_/this->map_resolution_;
				
				 if(x_cell >= (float)this->map_width_)  x_cell = (float)this->map_width_ - 1.0;
				 if(y_cell >= (float)this->map_height_) y_cell = (float)this->map_height_ - 1.0;
				
				 if(this->ego_pose_ok_ == true)
				 {
					 bool ego_is_dropout = false;
					 bool success = DecentVoroHandle_->AgentPoseUpdate(x_cell,  y_cell, ego_is_dropout, this->robot_cov_radius_, 0);
					 if(success == false) 
					 {
						 if (this->display_debug_msg_ == true) ROS_INFO("pose of ego robot is not updated");
						 ready = false;
						 break;
					 }
					 if (this->display_debug_msg_ == true) ROS_INFO("pose of ego robot is updated");
				 }
				 else
				 {
					 if (this->display_debug_msg_ == true) ROS_INFO("pose of ego robot is not received");
					 ready = false;
					 break;
				 }
			 }
			 else
			 {
				 			
				 float x_cell, y_cell;
				
				 x_cell = agent_info_array_[index].x_m/this->map_resolution_;
				
				 if(this->y_axis_swap_ == true) y_cell = (float)this->map_height_ - agent_info_array_[index].y_m/this->map_resolution_;
				 else y_cell = agent_info_array_[index].y_m/this->map_resolution_;
				
				 if(x_cell >= (float)this->map_width_)  x_cell = (float)this->map_width_ - 1.0;
				 if(y_cell >= (float)this->map_height_) y_cell = (float)this->map_height_ - 1.0;
			 
				 if(agent_info_array_[index].is_renew == true)
				 {
					 bool success = DecentVoroHandle_->AgentPoseUpdate(x_cell,  y_cell, agent_info_array_[index].is_dropout, agent_info_array_[index].robot_cov_radius, cur_robot_num);
					 agent_info_array_prev_[index].x_m = agent_info_array_[index].x_m;
					 agent_info_array_prev_[index].y_m = agent_info_array_[index].y_m;
					 agent_info_array_prev_[index].is_dropout = agent_info_array_[index].is_dropout;
					 agent_info_array_prev_[index].robot_cov_radius = agent_info_array_[index].robot_cov_radius;
					  agent_info_array_prev_[index].coverage_metric = agent_info_array_[index].coverage_metric;
					 if(success == false) 
					 {
						 if (this->display_debug_msg_ == true) ROS_INFO("pose of %d robot is not updated", index);
					 }
					 else 
					 {
						 coverage_metric_vec_.push_back(agent_info_array_[index].coverage_metric);
						 cur_robot_num++;
						 if (this->display_debug_msg_ == true) ROS_INFO("pose of %d robot is updated", index);
					 }
				 }
				 else if((this->InhbtDropout_ == true)&&(this->is_dropout_use_ == true))
				 {
					 x_cell = agent_info_array_prev_[index].x_m/this->map_resolution_;
				
					 if(this->y_axis_swap_ == true) y_cell = (float)this->map_height_ - agent_info_array_prev_[index].y_m/this->map_resolution_;
					 else y_cell = agent_info_array_[index].y_m/this->map_resolution_;
				
					 if(x_cell >= (float)this->map_width_)  x_cell = (float)this->map_width_ - 1.0;
					 if(y_cell >= (float)this->map_height_) y_cell = (float)this->map_height_ - 1.0;
					 bool success = DecentVoroHandle_->AgentPoseUpdate(x_cell,  y_cell, false, agent_info_array_prev_[index].robot_cov_radius, cur_robot_num);
					 if(success == false) 
					 {
						 if (this->display_debug_msg_ == true) ROS_INFO("pose of %d robot is not updated 22", index);
					 }
					 else 
					 {
						coverage_metric_vec_.push_back(agent_info_array_[index].coverage_metric);
						 cur_robot_num++;
						 if (this->display_debug_msg_ == true) ROS_INFO("pose of %d robot is updated 22", index);
					 }
				 }
			 }	
		}
		
		if (ready == true) 
		{
			this->RealTimeVoroProcess(cur_robot_num);
			float goal_x, goal_y, goal_y_m;
		    bool success = DecentVoroHandle_->AgentPoseGet(goal_x,  goal_y, 0);
			
			geometry_msgs::PoseStamped goal_msg;
			
		    if(success == true)
		    {
			     if(this->y_axis_swap_ == true) goal_y_m = ((float)this->map_height_ - goal_y - 1.0)*this->map_resolution_;
		         else goal_y_m = goal_y*this->map_resolution_;
			     goal_msg.pose.position.x = goal_x*this->map_resolution_;
			     goal_msg.pose.position.y =goal_y_m;
			     this->goal_pub_.publish(goal_msg);
		     }
		     else 
			 {
				 if (this->display_debug_msg_ == true) ROS_INFO("goal of robot is not published");
			 }
			 
			 if(this->ego_pose_ok_ == true)
		    {
			    decent_voronoi::robot_info robot_info_msg;
			    robot_info_msg.odom.pose.pose.position.x = this->ego_pos_x_;
			    robot_info_msg.odom.pose.pose.position.y = this->ego_pos_y_;
			    robot_info_msg.is_dropout.data = this->ego_is_dropout_;
			    robot_info_msg.robot_cov_radius.data = this->robot_cov_radius_;
			    robot_info_msg.coverage_metric.data = this->coverage_metric_;
				robot_info_msg.is_in_obs.data = this->is_in_obs_;
				robot_info_msg.coverage_metric_diff.data = this->coverage_metric_diff_;
			    robot_info_pub_.publish(robot_info_msg);
				this->txt_file_ <<ros::Time::now() << " " <<  this->ego_pos_x_   <<  " " <<  this->ego_pos_y_  <<  " "  << this->ego_is_dropout_ <<  " " << this->is_in_obs_ << " "<< this->robot_cov_radius_ << " " << this->coverage_metric_ << " " << this->coverage_metric_diff_ <<  std::endl;
		    }
		}
		

		

		

		
		
		coverage_metric_vec_.clear();
		
		for(int index=0; index < this->max_agent_num_; index++)
		{
			agent_info_array_[index].is_renew = false;
		}
		
	    //ros::spin();
	    ros::spinOnce();
	    this->loop_rate_->sleep();
	}
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_voronoi_ros");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  
  DecentVoronoiRosAgent  decentvoronoi_ros_simple_goal(nh,_nh);
  
  decentvoronoi_ros_simple_goal.run();
 
   return 0;
}
