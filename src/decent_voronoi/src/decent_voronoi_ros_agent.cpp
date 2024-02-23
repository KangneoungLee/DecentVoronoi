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

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui.hpp"


#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // [0]: pose x, [1]: pose y, [2]: is_dropout, [3]:robot_cov_radius, [4]: coverage_metric

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

//#include <decent_voronoi/robot_info.h>

#include "decent_voronoi/decent_voronoi.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

/************Function architecture*******************/
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

class DecentVoronoiRosAgent : public rclcpp::Node {
	
	private:

	     long int LoopRate_Milisec_ = 2000;
	 
		 int map_height_;
		 int map_width_;
		 int max_pixel_length_ = 400;
		 
		 int robot_index_ = 0;
		 float robot_maxspd_;
		 int robot_cov_radius_; 
		 float min_dist_goal_send_;

		 std::string img_save_dir_;
		 std::string img_color_label_dir_;
		 float map_resolution_;
		 
		 bool is_in_obs_ = false;
		 		 
		 bool y_axis_swap_;
		 int max_agent_num_;
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
		 float coverage_metric_ = 100;
		 float coverage_metric_prev_;
		 float coverage_metric_diff_prev_ = 100;
		 float coverage_metric_diff_ = 100;
		 float cm_diff_rate_ = 0.1;
		 
		 bool start_enable_ = false;
		 bool density_read_ok_ = false;
		 bool is_hete_cov_radius_;
		 bool is_propa_connected_area_;
		 bool is_img_save_;
		 bool is_uniform_density_;
		 std::string density_img_dir_;
		 
		 std::vector<float> coverage_metric_vec_; 
		 
		 
		 bool text_log_enable_;
		 std::string text_log_dir_;
		 std::ofstream txt_file_;
		 
		 rclcpp::TimerBase::SharedPtr timer_;
		  
		 std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> RobotInfoSubVec_;
		 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
		 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr robot_info_pub_;
		 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
		 rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr StartSub_;
		 
		 struct agent_info* agent_info_array_;
		 struct agent_info* agent_info_array_prev_;
		 	
	public:
	
	    void run();
		
		void read_density(unsigned char*  density_array, std::string img_dir, int height, int width);
		void ReadDensity();
		void IntializedAgents();
		void RealTimeVoroProcess(int cur_robot_num);
		void RobotInfoSubCallback(const std_msgs::msg::Float64MultiArray& msg , int index);
		void PosSubCallback(const nav_msgs::msg::Odometry & msg, int index);
		void StartCallback(const std_msgs::msg::Int8 & msg);

	 
		/*constructor and destructor*/
	    DecentVoronoiRosAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	    ~DecentVoronoiRosAgent();
	

}; // class DynamicVoronoiRosSimpleGoal end

DecentVoronoiRosAgent::DecentVoronoiRosAgent(const rclcpp::NodeOptions & options):Node("decent_voronoi_ros_agent")
{

	 auto option = this->get_node_options();
	 
	 this->declare_parameter("loop_rate_milisec", rclcpp::ParameterValue(2000));
	 this->declare_parameter("y_axis_swap", rclcpp::ParameterValue(true));
	 // if y_axis_swap is true, map coordinate in dynamic voronoi follows image coordinate (E : x, S :y) and real agent follows ENU coordinate (E : x, N : y)
	 // if y_axis_swap is false, map coordinate in dynamic voronoi and real agents follow image coordinate (E : x, S :y)
	 this->declare_parameter("map_height", rclcpp::ParameterValue(200)); //unit: pixel
	 this->declare_parameter("map_width", rclcpp::ParameterValue(200)); //unit: pixel
	 this->declare_parameter("map_resolution", rclcpp::ParameterValue(0.1)); 
     this->declare_parameter("robot_max_speed", rclcpp::ParameterValue(0.8)); 
     this->declare_parameter("robot_cov_radius", rclcpp::ParameterValue(16));
	 this->declare_parameter("robot_index", rclcpp::ParameterValue(0));
	 this->declare_parameter("min_dist_goal_send", rclcpp::ParameterValue(0.5));	
     this->declare_parameter("is_hete_cov_radius", rclcpp::ParameterValue(false));
     this->declare_parameter("is_dropout_use", rclcpp::ParameterValue(true)); 	 
	 this->declare_parameter("is_img_save", rclcpp::ParameterValue(false)); 
	 this->declare_parameter("img_save_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/opt_animation/"));
	 this->declare_parameter("img_color_label_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt"));
	 this->declare_parameter("is_propa_connected_area", rclcpp::ParameterValue(false));
	 this->declare_parameter("display_debug_msg", rclcpp::ParameterValue(false));
	 this->declare_parameter("max_agent_num", rclcpp::ParameterValue(20));
	 this->declare_parameter("dropout_terminate_criteria", rclcpp::ParameterValue(0.0002));
	 this->declare_parameter("low_pass_filter_coeff", rclcpp::ParameterValue(0.1));
	 this->declare_parameter("density_img_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/density/density_200_200_ver3.png"));
	 this->declare_parameter("text_log_enable", rclcpp::ParameterValue(false));
	 this->declare_parameter("text_log_dir", rclcpp::ParameterValue(""));
	 this->declare_parameter("max_pixel_length", rclcpp::ParameterValue(400));
	 
	 this->get_parameter("loop_rate_milisec", LoopRate_Milisec_);
	 this->get_parameter("y_axis_swap", this->y_axis_swap_);
	 this->get_parameter("map_height", this->map_height_);
	 this->get_parameter("map_width", this->map_width_);
	 this->get_parameter("map_resolution", this->map_resolution_);
	 this->get_parameter("robot_max_speed", this->robot_maxspd_);
	 this->get_parameter("robot_cov_radius", this->robot_cov_radius_);
	 this->get_parameter("robot_index", this->robot_index_);
	 this->get_parameter("min_dist_goal_send", this->min_dist_goal_send_);
	 this->get_parameter("is_hete_cov_radius", this->is_hete_cov_radius_);
	 this->get_parameter("is_dropout_use", this->is_dropout_use_);
	 this->get_parameter("is_img_save", this->is_img_save_);
	 this->get_parameter("img_save_dir", this->img_save_dir_);
	 this->get_parameter("img_color_label_dir", this->img_color_label_dir_);
	 this->get_parameter("is_propa_connected_area", this->is_propa_connected_area_);
	 this->get_parameter("display_debug_msg", this->display_debug_msg_);
	 this->get_parameter("max_agent_num", this->max_agent_num_);
	 this->get_parameter("dropout_terminate_criteria", this->dropout_terminate_criteria_);
	 this->get_parameter("low_pass_filter_coeff", this->cm_diff_rate_);
	 this->get_parameter("density_img_dir", this->density_img_dir_);
	 this->get_parameter("text_log_enable", this->text_log_enable_);
	 this->get_parameter("text_log_dir", this->text_log_dir_);
	 this->get_parameter("max_pixel_length", max_pixel_length_);


	
	 this->is_uniform_density_ = false;

	 
	 if(this->is_dropout_use_ == true)  this->InhbtDropout_ =  false; 
	 else this->InhbtDropout_ = true;

	 
	 float DropOutWeight_init = (float)this->max_agent_num_/(float)(this->max_agent_num_-1);
	 
	 	 

	 this->text_log_dir_ = this->text_log_dir_ + "agent" + std::to_string(this->robot_index_) + ".txt";

	 this->txt_file_.open(this->text_log_dir_);
	 this->txt_file_ << " time " << "pos_x (m) "<< "pos_y (m) " << "is_dropout "   << "is_in_obs " <<   "robot_cov_radius " << "coverage_metric "  << "coverage_metric_diff " <<  std::endl;
	 
	densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	for(int row=0; row<map_height_; row++)  
    {
        for(int col=0; col<map_width_; col++)
	{
            int index =col + row*map_width_;
	    densityPtr_[index] =  0;
	}
    }
	 
	 //this->ReadDensity();
	 //densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	 //this->read_density(densityPtr_, this->density_img_dir_, this->map_height_ , this->map_width_); //read density map from image file
	 DecentVoroHandle_ = new DecentVoronoi(this->map_height_, this->map_width_, DropOutWeight_init, this->is_uniform_density_, this->is_dropout_use_, 
	                                                                       this->is_hete_cov_radius_, this->is_propa_connected_area_, this->is_img_save_, this->display_debug_msg_, densityPtr_);
	 																   
	 agent_info_array_ = new struct agent_info[static_cast<int>(this->max_agent_num_)];
	 agent_info_array_prev_ = new struct agent_info[static_cast<int>(this->max_agent_num_)];
	 
	 this->IntializedAgents();	
	 	
	 /**generate subscriber for pose of robots, create publisher for goal, explore enable, costmap for exploration,  intiailize the cost map for exploration*/
	 std::string robot_str = "Robot";
	 for(int index=0; index< this->max_agent_num_; index++)
	 {
		 if(index != this->robot_index_)
		 {
			 std::string sub_topic = robot_str+std::to_string(index)+"/robot_info";
			 std::function<void(std_msgs::msg::Float64MultiArray)> fnc = std::bind(&DecentVoronoiRosAgent::RobotInfoSubCallback, this, _1, index);
			 rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(sub_topic, 3, fnc);
			 RobotInfoSubVec_.push_back(_sub);
		 }			 
		
		float x_pos = 0;
		float y_pos = 0;
		float v_travel = 0;
		float cov_radius = 1;
		DecentVoroHandle_->PushPoint(x_pos, y_pos, v_travel, cov_radius); 
		
	 }
	 

	 std::string sub_topic = robot_str+std::to_string(this->robot_index_)+"/odom";
	 std::function<void(nav_msgs::msg::Odometry)> fnc = std::bind(&DecentVoronoiRosAgent::PosSubCallback, this, _1, 0);
	 this->pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(sub_topic, 3, fnc);
	 	
	 this->StartSub_ = this->create_subscription<std_msgs::msg::Int8>("decent_voro_start", 3, std::bind(&DecentVoronoiRosAgent::StartCallback, this, _1));
	 	
	 std::string pub_topic = robot_str+std::to_string(this->robot_index_)+"/robot_info";
	 this->robot_info_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_topic, 3);
	 	
	 pub_topic = robot_str+std::to_string(this->robot_index_)+"/goal";
	 this->goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pub_topic, 3);
	 
	 this->coverage_metric_prev_ = this->coverage_metric_;
	 
	 timer_ = create_wall_timer(std::chrono::milliseconds(LoopRate_Milisec_), std::bind(&DecentVoronoiRosAgent::run, this));
}

DecentVoronoiRosAgent::~DecentVoronoiRosAgent()
{
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


void DecentVoronoiRosAgent::RobotInfoSubCallback(const std_msgs::msg::Float64MultiArray& msg , int index)
{
	if(agent_info_array_[index].is_renew == false)
	{
	    agent_info_array_[index].x_m = (float)msg.data[0];
	    agent_info_array_[index].y_m= (float)msg.data[1];
	    agent_info_array_[index].index =index;
		agent_info_array_[index].is_dropout = (bool)msg.data[2];
		agent_info_array_[index].robot_cov_radius = (float)msg.data[3];
		agent_info_array_[index].coverage_metric = (float)msg.data[4];
	    agent_info_array_[index].is_renew =true;
	}
}

void DecentVoronoiRosAgent::StartCallback(const std_msgs::msg::Int8 & msg)
{
	if (msg.data == 0) start_enable_ = false;
	else start_enable_ = true;
}

void DecentVoronoiRosAgent::PosSubCallback(const nav_msgs::msg::Odometry & msg, int index)
{
	 ego_pos_x_ = msg.pose.pose.position.x;
	 ego_pos_y_= msg.pose.pose.position.y;
	 ego_pose_ok_ = true;
}



void DecentVoronoiRosAgent::RealTimeVoroProcess(int cur_robot_num)
{
    bool is_in_obs  = DecentVoroHandle_->CheckCurPosInObs();
	DecentVoroHandle_->ExpandedVoronoi(cur_robot_num);
	DecentVoroHandle_->CentroidCal();
	DecentVoroHandle_->MoveAgents();
	this->coverage_metric_ = DecentVoroHandle_->CoverageMetric();
	
	if (this->display_debug_msg_ == true) std::cout << "sum of coverage metric : " << this->coverage_metric_ << " ,  cur robot num : " << cur_robot_num << std::endl;
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
	
	if (this->display_debug_msg_ == true) std::cout << "sum of coverage metric : " << this->coverage_metric_ << " ,  cur robot num : " << cur_robot_num << std::endl;
	
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


void DecentVoronoiRosAgent::ReadDensity()
{
    cv::Mat img = cv::imread(density_img_dir_, cv::IMREAD_GRAYSCALE); //cv::imread(density_img_dir_, cv::IMREAD_GRAYSCALE);
    int width_org = img.cols;
    int height_org = img.rows;
    
    if ((width_org >= height_org)&&(width_org > max_pixel_length_))
    {
    	float ratio = ((float)height_org)/((float)width_org);
    	map_width_ = (int)max_pixel_length_;
    	map_height_ = (int)(ratio*((float)map_width_));
    }
    else if((height_org >= width_org)&&(height_org > max_pixel_length_))
    {
    	float ratio = ((float)width_org)/((float)height_org);
    	map_height_ = (int)max_pixel_length_;
    	map_width_ = (int)(ratio*((float)map_height_));    
    }
    else
    {
        map_width_ = width_org;
        map_height_ = height_org;
    }
    
    if(!(densityPtr_ == NULL)) delete[] this->densityPtr_;
    densityPtr_ = new unsigned char[map_height_*map_width_];
    cv::resize(img, img, cv::Size(map_width_, map_height_));
    cv::Mat mat_255(cv::Size(map_width_, map_height_), CV_8UC1, cv::Scalar(255));
    
    cv::absdiff(mat_255, img, img);
    //cv::imwrite("/home/artlab/nri_plan_ws/src/trunk/density_resize.png", img);
    
    img.convertTo(img, CV_32FC1);  //refering unsigned char type is not working so convert image to float type

    for(int row=0; row<map_height_; row++)  
    {
        for(int col=0; col<map_width_; col++)
	{
            int index =col + row*map_width_;
	    densityPtr_[index] =  (unsigned char)img.at<float>(row, col);
	}
    }


    if (this->display_debug_msg_ == true) std::cout << "____Read Density Map completed____ " << std::endl;
}


void DecentVoronoiRosAgent::run()
{  

	int cur_robot_num = 1;
	int all_robot_num = 1;
	
	if (this->start_enable_ == false) 
	{
		this->density_read_ok_ = false;
		return;
	}
	
	if(this->density_read_ok_ == false)
	{
		this->ReadDensity();
		DecentVoroHandle_->ResizeMap(map_height_, map_width_, densityPtr_);
		this->density_read_ok_ = true;
	}
	
	bool ready = true;

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
					if (this->display_debug_msg_ == true) std::cout << "pose of ego robot is not updated" << std::endl;
					ready = false;
					break;
				}
				if (this->display_debug_msg_ == true) std::cout << "pose of ego robot is updated" << std::endl;
				
				if (global_cnt < 1)
				{
					auto robot_info_msg = std_msgs::msg::Float64MultiArray();
					robot_info_msg.data.push_back((double)this->ego_pos_x_);
					robot_info_msg.data.push_back((double)this->ego_pos_y_);
					robot_info_msg.data.push_back((double)this->ego_is_dropout_);
					robot_info_msg.data.push_back((double)this->robot_cov_radius_);
					robot_info_msg.data.push_back((double)this->coverage_metric_);
					robot_info_msg.data.push_back((double)this->is_in_obs_);
					robot_info_msg.data.push_back((double)this->coverage_metric_diff_);
					robot_info_pub_->publish(robot_info_msg);
				}
			}
			else
			{
				if (this->display_debug_msg_ == true) std::cout << "pose of ego robot is not received" << std::endl;
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
					if (this->display_debug_msg_ == true) std::cout << "pose of "<< index << " robot is not updated" << std::endl;
				}
				else 
				{
					coverage_metric_vec_.push_back(agent_info_array_[index].coverage_metric);
					cur_robot_num++;
					if (this->display_debug_msg_ == true) std::cout << "pose of "<< index << " robot is updated" << std::endl;
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
					if (this->display_debug_msg_ == true) std::cout << "pose of " << index << " robot is not updated under self-dropout terminated " << std::endl;
				}
				else 
				{
					coverage_metric_vec_.push_back(agent_info_array_[index].coverage_metric);
					cur_robot_num++;
					if (this->display_debug_msg_ == true) std::cout << "pose of "<< index << " robot is updated under self-dropout terminated" << std::endl;
				}
			}
		}	
    }
	
	if (cur_robot_num <= 1) ready = false;
	else ready = true;
	
	if (ready == true) 
	{
		this->RealTimeVoroProcess(cur_robot_num);
		float goal_x, goal_y, goal_y_m;
		bool success = DecentVoroHandle_->AgentPoseGet(goal_x,  goal_y, 0);
		
        auto goal_msg = geometry_msgs::msg::PoseStamped();		
			
		if(success == true)
		{
			if(this->y_axis_swap_ == true) goal_y_m = ((float)this->map_height_ - goal_y - 1.0)*this->map_resolution_;
		    else goal_y_m = goal_y*this->map_resolution_;
			goal_msg.pose.position.x = goal_x*this->map_resolution_;
			goal_msg.pose.position.y =goal_y_m;
			this->goal_pub_->publish(goal_msg);
		}
		else 
		{
			if (this->display_debug_msg_ == true) std::cout << "goal of robot is not published" << std::endl;
		}
			 
		if(this->ego_pose_ok_ == true)
		{
			auto robot_info_msg = std_msgs::msg::Float64MultiArray();

			robot_info_msg.data.push_back((double)this->ego_pos_x_);
			robot_info_msg.data.push_back((double)this->ego_pos_y_);
			robot_info_msg.data.push_back((double)this->ego_is_dropout_);
			robot_info_msg.data.push_back((double)this->robot_cov_radius_);
			robot_info_msg.data.push_back((double)this->coverage_metric_);
		    robot_info_msg.data.push_back((double)this->is_in_obs_);
			robot_info_msg.data.push_back((double)this->coverage_metric_diff_);
			robot_info_pub_->publish(robot_info_msg);
			this->txt_file_ << " " <<  this->ego_pos_x_   <<  " " <<  this->ego_pos_y_  <<  " "  << this->ego_is_dropout_ <<  " " << this->is_in_obs_ << " "<< this->robot_cov_radius_ << " " << this->coverage_metric_ << " " << this->coverage_metric_diff_ <<  std::endl;
		}
	}
		
	coverage_metric_vec_.clear();
		
	for(int index=0; index < this->max_agent_num_; index++)
	{
		agent_info_array_[index].is_renew = false;
    }
		
	
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecentVoronoiRosAgent>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
 
   return 0;
}
