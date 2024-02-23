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

#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ml.hpp"
#include <boost/bind.hpp>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>

#include "decent_voronoi/decent_voronoi.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

/************Function architecture*******************/
std::mutex _lock;

struct agent_info
{
	float x;
	float y;
	float max_speed;
	int index;
	bool is_renew;
};

class OfflineMetricCal : public rclcpp::Node{
	
	private:
	
		 long int LoopRate_Milisec_ = 2000;
		 unsigned short map_height_;
		 unsigned short map_width_;
		 float map_resolution_;
		 
		 unsigned char* densityPtr_ = NULL;
		 
		 DecentVoronoi* DecentVoroHandle_;
		 
		 bool is_propa_connected_area_;
		 bool is_hete_cov_radius_;
		 bool y_axis_swap_;
		 bool completed_ = false;
		 int agent_num_;
		 
		 std:: ifstream txtfile_[100];
		 std::vector<std::string> text_;
		 std::ofstream metric_out_txt_file_;
		 
		 std::string log_file_dir_;
		 std::string metric_out_file_dir_;
		 std::string img_save_dir_;
		 std::string img_color_label_dir_;
		 std::string density_img_dir_;
		 
		 rclcpp::TimerBase::SharedPtr timer_;

	public:

		void MetricCal();
		void read_density(unsigned char*  density_array, std::string img_dir, int height, int width);
	 
		/*constructor and destructor*/
	    OfflineMetricCal(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	    ~OfflineMetricCal();
	

}; // class OfflineMetricCal end

OfflineMetricCal::OfflineMetricCal(const rclcpp::NodeOptions & options):Node("Offline_metric_cal")
{

     auto option = this->get_node_options();

	 this->declare_parameter("loop_rate_milisec", rclcpp::ParameterValue(2000));
	 this->declare_parameter("y_axis_swap", rclcpp::ParameterValue(true));
	 this->declare_parameter("map_height", rclcpp::ParameterValue(200)); //unit: pixel
	 this->declare_parameter("map_width", rclcpp::ParameterValue(200)); //unit: pixel
	 this->declare_parameter("map_resolution", rclcpp::ParameterValue(0.1));
	 this->declare_parameter("agent_num", rclcpp::ParameterValue(10));
     this->declare_parameter("is_propa_connected_area", rclcpp::ParameterValue(false));
	 this->declare_parameter("is_hete_cov_radius", rclcpp::ParameterValue(false));
	 this->declare_parameter("density_img_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/density_new/case1/user_define10.png"));
	 this->declare_parameter("log_file_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/ICRA2024_paper/case1_2_unity_dropout"));
	 this->declare_parameter("metric_out_file_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/ICRA2024_paper/case1_2_unity_dropout"));
	 this->declare_parameter("img_save_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/decent_voronoi/src/decent_voronoi/test/opt_animation/"));
	 this->declare_parameter("img_color_label_dir", rclcpp::ParameterValue("/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt"));
	 	 
	 
	 this->get_parameter("loop_rate_milisec", this->LoopRate_Milisec_);
	 this->get_parameter("y_axis_swap", this->y_axis_swap_);
	 this->get_parameter("map_height", this->map_height_);
	 this->get_parameter("map_width", this->map_width_);
	 this->get_parameter("map_resolution", this->map_resolution_);
	 this->get_parameter("agent_num", this->agent_num_);
	 this->get_parameter("is_propa_connected_area", this->is_propa_connected_area_);
	 this->get_parameter("is_hete_cov_radius", this->is_hete_cov_radius_);
	 this->get_parameter("density_img_dir", this->density_img_dir_);
	 this->get_parameter("log_file_dir", this->log_file_dir_);
	 this->get_parameter("metric_out_file_dir", this->metric_out_file_dir_);
	 this->get_parameter("img_save_dir", this->img_save_dir_);
	 this->get_parameter("img_color_label_dir", this->img_color_label_dir_);
	 
	 densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	 this->read_density(densityPtr_, this->density_img_dir_, this->map_height_ , this->map_width_); //read density map from image file
	 
	 float DropOutWeight_init = 1;// dummy variable
	 bool is_uniform_density = false;
	 bool is_dropout_use = false;
	 bool is_img_save = true;
	 bool display_debug_msg_ = false;
	 
	 DecentVoroHandle_ = new DecentVoronoi(this->map_height_, this->map_width_, DropOutWeight_init, is_uniform_density, is_dropout_use, 
	                                                                       this->is_hete_cov_radius_, this->is_propa_connected_area_, is_img_save,  display_debug_msg_, densityPtr_);

	 
	 this->metric_out_txt_file_.open(this->metric_out_file_dir_);


	 for(int i = 0; i < this->agent_num_; i++)
	 {	 
		 DecentVoroHandle_->PushPoint(0, 0, 0, 1); 
	 }
	 
	 timer_ = create_wall_timer(std::chrono::milliseconds(LoopRate_Milisec_), std::bind(&OfflineMetricCal::MetricCal, this));
	 
	 std::cout << "OfflineMetricCal initialize done" << std::endl;
}

OfflineMetricCal::~OfflineMetricCal()
{
	for(int i =0; i < this->agent_num_; i++)
	{
		this->txtfile_[i].close();
	}
	this->metric_out_txt_file_.close();
}

void OfflineMetricCal::MetricCal()
{
	if (this->completed_ == true) return;

	for (int i = 0; i < this->agent_num_; i ++)
	{
		std::string log_file = this->log_file_dir_ + "/agent" + std::to_string(i) +".txt";
		this->txtfile_[i].open(log_file);
	}

	int read_line_cnt = 0;
	bool end_of_line = false;
	   
	while(1)
	{
		   
		this->text_.clear();
		for(int i = 0; i < this->agent_num_; i ++)
		{
			std::string text_s;
			std::getline(this->txtfile_[i] ,text_s);
			if (text_s.empty())
			{
				end_of_line = true;
				break;
			}			   
			this->text_.push_back(text_s);
		}
		   
		if(end_of_line == true) break;
		   
		if (read_line_cnt == 0)
		{
			read_line_cnt++;
			continue;
		}
	 
		for(int i = 0; i < this->agent_num_; i ++)
	    {
			std::istringstream ss(this->text_[i]);
			std::string word;
			std::vector<float> input;  /*time x_pose y_pose is_dropout is_in_obs robot_cov_radius coverage_metric coverage_metric_diff*/
			while (ss >> word)
			{
			    input.push_back(std::stof(word));
	        }
			
			std::cout << i <<"th agent info... x pose : " <<  input[1] <<" , y_pose : " << input[2] <<" , cov radius : " << (int)input[5] << std::endl;

			bool success = DecentVoroHandle_->AgentPoseUpdate(input[1]/this->map_resolution_,  input[2]/this->map_resolution_, false, (int)input[5], i);
			if(success == false) std::cout << "fail to pose update" << i << "th agent" << std::endl; 
				
		    
		}
		   
		DecentVoroHandle_->ExpandedVoronoi(this->agent_num_);
		DecentVoroHandle_->CentroidCal();
		DecentVoroHandle_->MoveAgents();
		float coverage_metric = DecentVoroHandle_->CoverageMetric();
		   
		std::string img_dir =this->img_save_dir_ + "robot" + std::to_string(read_line_cnt) + ".png";
		DecentVoroHandle_->Colorized(img_dir, this->img_color_label_dir_);
		   
		DecentVoroHandle_->InitializeCell();
		   
		this->metric_out_txt_file_ << coverage_metric << std::endl;
		   
		read_line_cnt++;
	}
	
	this->completed_ = true;


	
}

void OfflineMetricCal::read_density(unsigned char*  density_array, std::string img_dir, int height, int width)
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
	std::cout<<" ************Read density information ***************" <<std::endl;
	
	return;
	
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OfflineMetricCal>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

 
   return 0;
}
