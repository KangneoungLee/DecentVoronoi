/*********************************************************************
MIT License

Copyright (c) 2024 Kangneoung Lee

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
#include <math.h>
#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

/************Function architecture*******************/
std::mutex _lock;
int global_cnt = 0;

struct agent_info
{
	float x;
	float y;
	float max_speed;
	int index;
	bool is_renew;
};

class DynVoroVirtualAgentRos: public rclcpp::Node{
	
	private:		 
		 int virtual_agent_start_index_;
		 
		 bool y_axis_swap_;
		 int agent_num_;
		 
		 std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_sub_vec_;
		 std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pose_pub_vec_;
		 
		 float move_rate_ = 0.1;
		 
		 long int LoopRate_Milisec_ = 2000;
		 
		 float init_pivot_x_m_;
		 float init_pivot_y_m_;
		 float init_random_d_m_;
		 
		 struct agent_info* agent_pose_array_;
		 struct agent_info* agent_goal_array_;
		 
		 rclcpp::TimerBase::SharedPtr timer_;

		 std::vector<std::vector<float> > agent_attribute_vec_;
		 
	public:
	
	    void run();
		void IntializedAgents(std::vector<double> init_pose_vec);		
		void goal_sub_callback(const geometry_msgs::msg::PoseStamped & msg, int index);
	 
		/*constructor and destructor*/
	    DynVoroVirtualAgentRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	    ~DynVoroVirtualAgentRos();
	

}; // class DynamicVoronoiRos end

DynVoroVirtualAgentRos::DynVoroVirtualAgentRos(const rclcpp::NodeOptions & options):Node("decent_voro_virtual_agent_ros")
{

     auto option = this->get_node_options();
	 
	 this->declare_parameter("loop_rate_milisec", rclcpp::ParameterValue(2000));
	 this->declare_parameter("y_axis_swap", rclcpp::ParameterValue(true));
	 this->declare_parameter("virtual_agent_start_index", rclcpp::ParameterValue(0));
	 this->declare_parameter("agent_num", rclcpp::ParameterValue(10));
	 this->declare_parameter("init_pivot_x_m", rclcpp::ParameterValue(0.4));
	 this->declare_parameter("init_pivot_y_m", rclcpp::ParameterValue(0.4));
	 this->declare_parameter("init_random_d_m", rclcpp::ParameterValue(0.4));
	 this->declare_parameter("agent_init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
	 
	 this->get_parameter("loop_rate_milisec", LoopRate_Milisec_);
	 this->get_parameter("y_axis_swap", this->y_axis_swap_);
	 this->get_parameter("virtual_agent_start_index", this->virtual_agent_start_index_);
	 this->get_parameter("agent_num", this->agent_num_);
	 this->get_parameter("init_pivot_x_m", this->init_pivot_x_m_);
	 this->get_parameter("init_pivot_y_m", this->init_pivot_y_m_);
	 this->get_parameter("init_random_d_m", this->init_random_d_m_);
	     
    
	 std::vector<double> agent_init_pose_vec;
	 
	 this->get_parameter("agent_init_pose", agent_init_pose_vec);

	 agent_pose_array_ = new struct agent_info[static_cast<int>(this->agent_num_)];
	 agent_goal_array_ = new struct agent_info[static_cast<int>(this->agent_num_)];

	 this->IntializedAgents(agent_init_pose_vec);	

	 for(int index=this->virtual_agent_start_index_; index< this->agent_num_; index++)
	 {
		std::string robot_str = "Robot"; 
		std::string sub_topic = robot_str+std::to_string(index)+"/goal";
		std::function<void(geometry_msgs::msg::PoseStamped)> fnc = std::bind(&DynVoroVirtualAgentRos::goal_sub_callback, this, _1, index);
	    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_topic, 2, fnc);
	    goal_sub_vec_.push_back(goal_sub);		 
		
		std::string pub_topic = robot_str+std::to_string(index)+"/odom";
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(pub_topic, 1);
		pose_pub_vec_.push_back(pose_pub);
		
		agent_pose_array_[index].index = index;
		agent_pose_array_[index].is_renew = false;
		
		agent_goal_array_[index].index = index;
		agent_goal_array_[index].is_renew = false;
	 }
	 
	 timer_ = create_wall_timer(std::chrono::milliseconds(LoopRate_Milisec_), std::bind(&DynVoroVirtualAgentRos::run, this));
	 
	 std::cout << "virtual agent initialize done" << std::endl;
}

DynVoroVirtualAgentRos::~DynVoroVirtualAgentRos()
{
	delete[] this->agent_pose_array_;
    delete[] this->agent_goal_array_;
}

void DynVoroVirtualAgentRos::IntializedAgents(std::vector<double> init_pose_vec)
{
	 int no_agent_internal_param = 3;
	 for (int i = 0; i < this->agent_num_; i++)
	 {
		 std::vector<float> agent_attribute;
		 for (int j =0; j<  no_agent_internal_param; j++)
		 {
			float value_temp;
            value_temp = (float)init_pose_vec[no_agent_internal_param * i + j];    

			agent_attribute.push_back(value_temp);
		 }
		 agent_attribute_vec_.push_back(agent_attribute);
	 }
	 
	 int idx = 0;
	 std::vector<std::vector<float>>::iterator iter;
	 
	 for (iter = this->agent_attribute_vec_.begin(); iter != this->agent_attribute_vec_.end(); iter++) {
		
		std::vector<float> agent_attribute;
		agent_attribute = *iter; 
		agent_pose_array_[idx].x = agent_attribute.at(0);
		agent_pose_array_[idx].y = agent_attribute.at(1);
		agent_pose_array_[idx].max_speed = agent_attribute.at(2);
		idx++;
		
		std::cout<< " idx : " << idx <<  " x : " << agent_attribute.at(0) << " y : " << agent_attribute.at(1) << " max speed : " << agent_attribute.at(2) << std::endl;
	 }	
	
}


void DynVoroVirtualAgentRos::goal_sub_callback(const geometry_msgs::msg::PoseStamped & msg , int index)
{
	 agent_goal_array_[index].x = msg.pose.position.x;
	 agent_goal_array_[index].y = msg.pose.position.y;
	 agent_goal_array_[index].index =index;
	 agent_goal_array_[index].is_renew = true;
}


void DynVoroVirtualAgentRos::run()
{  

	bool ready = true;
	
	for(int index=this->virtual_agent_start_index_; index < this->agent_num_; index++)
	{
		if (ready == false) break;
			
		if(agent_goal_array_[index].is_renew == true)
		{
			float del_x =  (agent_goal_array_[index].x - agent_pose_array_[index].x);
			float del_y = (agent_goal_array_[index].y - agent_pose_array_[index].y);
			float dist = (float)sqrt((double)(del_x*del_x + del_y*del_y));
			float max_dist = agent_pose_array_[index].max_speed*(this->LoopRate_Milisec_/1000.0);
			//std::cout << " index : " << index << " dist : " << dist << " max_dist : " << max_dist << " del_x : " << del_x << " del_y : " << del_y << std::endl;
			float inc_x = 0;
			float inc_y = 0;

			if(dist > max_dist)
			{
				float cos =  del_x/dist;
				float sin =   del_y/dist;
					
				inc_x = cos*max_dist;
				inc_y = sin*max_dist;
			}
			else
			{
				inc_x = del_x;
				inc_y = del_y;
			}
			
			bool no_update = false;
			
			/*for (int index_2 = index; index_2 < this->agent_num_; index_2++)
			{
				if (index_2 == index) continue;
				
				float del_x_collide = agent_pose_array_[index].x +  inc_x - agent_pose_array_[index_2].x;
				float del_y_collide = agent_pose_array_[index].y +  inc_y - agent_pose_array_[index_2].y;
				
				float dist = (float)sqrt((double)(del_x_collide*del_x_collide + del_y_collide*del_y_collide));
				
				if (dist < 2) 
				{
					no_update = true;
					break;
				}
			}*/
		    
			if (no_update == false)
			{
				agent_pose_array_[index].x =  agent_pose_array_[index].x +  inc_x;
				agent_pose_array_[index].y =  agent_pose_array_[index].y +  inc_y;
			}
		}
			
		auto pose_msg = nav_msgs::msg::Odometry();
		pose_msg.pose.pose.position.x = agent_pose_array_[index].x;
		pose_msg.pose.pose.position.y = agent_pose_array_[index].y;
		pose_pub_vec_[index- this->virtual_agent_start_index_]->publish(pose_msg);
			
		std::cout << index << " agent pose x : " << agent_pose_array_[index].x << " y : " << agent_pose_array_[index].y << std::endl; 
	}

	
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynVoroVirtualAgentRos>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
 
   return 0;
}
