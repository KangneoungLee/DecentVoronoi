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

#include "decent_voronoi/decent_voronoi.h"


std::vector<std::string> split(std::string str, char Delimiter) {
    std::istringstream iss(str);             
    std::string buffer;                      
 
    std::vector<std::string> result;
 
 
    while (std::getline(iss, buffer, Delimiter)) {
        result.push_back(buffer);             
    }
 
    return result;
}


float density_avg = 1;

int propagation_frame_interval = 100;
std::ofstream img_list;

bool is_optimize_animation = false;

/*constructor and destructor*/
DecentVoronoi::DecentVoronoi(unsigned short map_height, unsigned short map_width, float DropOutWeight,
                                                            bool is_uniform_density, bool is_dropout_use, bool is_hete_cov_radius, bool is_propa_connected_area, bool is_img_save, bool display_debug_msg, unsigned char* vorocellDenseMapExtPtr):
map_height_(map_height),map_width_(map_width), DropOutWeight_(DropOutWeight),  is_uniform_density_(is_uniform_density), is_dropout_use_(is_dropout_use), is_hete_cov_radius_(is_hete_cov_radius), 
is_propa_connected_area_(is_propa_connected_area), is_img_save_(is_img_save), display_debug_msg_(display_debug_msg), vorocellDenseMapExtPtr_(vorocellDenseMapExtPtr)
{
	/* datum is for calculating the starting point of coverage*/

   /*c++ style */	
   this->vorocell_ = new VoroCell*[map_height*map_width];
   
   for (int i = 0; i < map_height*map_width; ++i) {
    this->vorocell_[i] = new VoroCell();
   }
   
   this->vorocellDenseMap_ = new unsigned char[map_height*map_width];
   
   this->vorocellObsMap_ = new unsigned char[map_height*map_width];
   /*c style */
   //this->vorocell_ = (VoroCell*)malloc(sizeof(VoroCell)*((size_t)map_height)*((size_t)map_width));
   //this->vorocellDenseMap_ = (unsigned char*)malloc(sizeof(unsigned char)*((size_t)map_height)*((size_t)map_width));
   
   this->is_propa_completed_ = false;
   this->VoroPartNum_ = 0;
   this->CurAgentNum_ = 0;
   this->TotalMass_ = 0;
   this->CovrdMass_ = 0;
   this->TotalArea_ = 0;
   this->CovrdArea_ = 0;   
   
   this->InitializeCell();
   this->InitializeDensityMap();
}

DecentVoronoi::~DecentVoronoi()
{
     for (int i = 0; i < this->map_height_*this->map_width_; ++i) delete this->vorocell_[i];

     for (int i = 0; i < this->VoroPartNum_; ++i) delete this->partition_info_[i];

	 img_list.close();
	 
	 delete[] this->vorocell_;
	 delete[] this->vorocellDenseMap_;
	 delete[] this->partition_info_;
	 delete[] this->vorocellObsMap_;

}

void DecentVoronoi::ResizeMap(int map_height_new, int map_width_new, unsigned char* vorocellDenseMapExtPtr_new)
{
	if(!(this->vorocell_ == NULL))  
	{
		for (int i = 0; i < this->map_height_*this->map_width_; ++i) delete this->vorocell_[i];
		delete[] this->vorocell_;
	}
    
	if(!(this->partition_info_ == NULL))  
	{
		for (int i = 0; i < this->VoroPartNum_; i++) 
		{
			PartitionInfo* partition_info_single = this->partition_info_[i];
			partition_info_single->centroid_momentsum_x_ = 0;
	        partition_info_single->centroid_momentsum_y_ = 0;
	        partition_info_single->part_mass_ = 0;
	        partition_info_single->part_area_ = 0;
	        partition_info_single->part_radius_in_area_ = 0;
			partition_info_single->is_dropout_ = false;
	        partition_info_single->part_centroid_x_ = 0;
	        partition_info_single->part_centroid_y_ = 0;
		}
	}
	
	if(!(this->vorocellDenseMap_ == NULL)) delete[] this->vorocellDenseMap_;
	
	if(!(this->vorocellObsMap_ == NULL)) delete[] this->vorocellObsMap_;	
	
	std::cout << " Map resize start " << std::endl;
	
	this->map_height_ = map_height_new;
	this->map_width_ = map_width_new;
	this->vorocellDenseMapExtPtr_ = vorocellDenseMapExtPtr_new;
	
	this->vorocell_ = new VoroCell*[this->map_height_*this->map_width_];
	
	for (int i = 0; i < this->map_height_*this->map_width_; ++i)  this->vorocell_[i] = new VoroCell();
 
    this->vorocellDenseMap_ = new unsigned char[this->map_height_*this->map_width_]; 

    this->vorocellObsMap_ = new unsigned char[this->map_height_*this->map_width_];
	
   this->InitializeCell();
   this->InitializeDensityMap();	
	
   std::cout << " Map resize end "  << " new map height : " << this->map_height_  << " new map width : " << this->map_width_<< std::endl;
}

VoroCell** DecentVoronoi::GetVoroCellMap()
{
	return this->vorocell_;
}

int DecentVoronoi::GetIndex(int x, int y)
{
	if((x>=this->map_width_)||(y>=this->map_height_))
	{
		std::cout<<"x or y index is out of bound in DynamicVoronoi :" << " x : " << x << " y : " << y <<std::endl;
		exit(0);
	}
	
	int index = x + y*this->map_width_;
	
	return index;
};

int DecentVoronoi::MapPointToIndex(float x, float y)
{
	int x_rounded = (int)std::round(x);
	int y_rounded = (int)std::round(y);
	
	int index = this->GetIndex(x_rounded, y_rounded);
	
	return index;
};

VoroCell* DecentVoronoi::GetSingleCellByIndex(int x, int y)
{
	int index = this->GetIndex(x, y);
	
	return   this->vorocell_[index];
};

VoroCell* DecentVoronoi::GetSingleCellByPoint(float x, float y)
{
	int index = this->MapPointToIndex(x, y);
	
	return  this->vorocell_[index];
};


bool DecentVoronoi::AgentPoseUpdate(float x, float y, bool is_dropout, float robot_cov_radius, int agent_num)
{
	if(x >= this->map_width_)
	{
		return false;
	}
	
	if(y >= this->map_height_)
	{
		return false;
	}	
	
	if(agent_num > this->VoroPartNum_)
	{
		return false;
	}
	
	PartitionInfo* partition_info_single = this->partition_info_[agent_num];
	
	partition_info_single->part_agent_coor_x_ = x;
	partition_info_single->part_agent_coor_y_ = y;
	partition_info_single->is_dropout_ = is_dropout;
	partition_info_single->part_cov_radius_ = robot_cov_radius;
	
	return true;
}

bool DecentVoronoi::AgentPoseGet(float& x, float& y, int agent_num)
{
	
	if(agent_num > this->VoroPartNum_)
	{
		return false;
	}
	
	PartitionInfo* partition_info_single = this->partition_info_[agent_num];
	
	x = partition_info_single->part_agent_coor_x_ ;
	y = partition_info_single->part_agent_coor_y_ ;
	
	return true;
}

bool DecentVoronoi::PushPoint(float x, float y, float v_travel, float cov_radius)
{
	/* add agent's parameter set (for example, agent position, velocity.. etc*/
	
	if(x >= this->map_width_)
	{
		return false;
	}
	
	if(y >= this->map_height_)
	{
		return false;
	}
	
	if(v_travel < 0.01)
	{
		v_travel = 0.01;
	}
	

	if(cov_radius < 1)
	{
		cov_radius = 1; 
		std::cout<<" covarge radius should be greater than or equal to 1"<<std::endl;
	}	
	
	if(this->VoroPartNum_ == 0)
	{
		this->partition_info_ = new PartitionInfo*[this->VoroPartNum_+1];
		this->partition_info_[0] = new PartitionInfo();
	}
	else
	{
		
		PartitionInfo** partition_info_Temp = new PartitionInfo*[this->VoroPartNum_+1];
		
		memcpy(partition_info_Temp,this->partition_info_,sizeof(PartitionInfo*)*this->VoroPartNum_);
		partition_info_Temp[this->VoroPartNum_] = new PartitionInfo();
		
		delete[] this->partition_info_;
		
		this->partition_info_ = new PartitionInfo*[this->VoroPartNum_+1];
		
		
		memcpy(this->partition_info_,partition_info_Temp,sizeof(PartitionInfo*)*(this->VoroPartNum_+1));
		
		delete[] partition_info_Temp;
	}
	
	PartitionInfo* partition_info_single = this->partition_info_[this->VoroPartNum_];
    partition_info_single->part_agentclass_ = this->VoroPartNum_;
	partition_info_single->centroid_momentsum_x_ = 0;
	partition_info_single->centroid_momentsum_y_ = 0;
	partition_info_single->part_mass_ = 0;
	partition_info_single->part_area_ = 0;
	partition_info_single->part_radius_in_area_ = 0;
	partition_info_single->is_dropout_ = false;
	partition_info_single->part_centroid_x_ = 0;
	partition_info_single->part_centroid_y_ = 0;
	partition_info_single->part_agent_coor_x_ = x;
	partition_info_single->part_agent_coor_y_ = y;
	partition_info_single->part_v_travel_ = v_travel;
	partition_info_single->part_cov_radius_ = cov_radius;


	//std::vector<float>  xy_coor;
    
    //xy_coor.push_back(x);
    //xy_coor.push_back(y);	
	
	//this->VoroPartCenters_.insert(std::make_pair(this->VoroPartNum_, xy_coor));

	this->is_propa_completed_ = false;
	
	this->VoroPartNum_ = this->VoroPartNum_ + 1;


	this->inhibit_dropout_ = false;
	this->inhibit_dropout_cnt = 0;

	
	return true;
}

void DecentVoronoi::InitializeCell()
{
	int x,y;
	
	VoroCell* single_cell;
	
	for(x= 0;x<this->map_width_;x++)
	{
		for(y=0;y<this->map_height_;y++)
		{
			
			single_cell = GetSingleCellByIndex(x,y);
			single_cell->is_edge_ = false; 
			single_cell->sq_dist_ = 0xFFFFFFFF;   /* Max value of usigned int value */
			single_cell->agentclass_ = 0xFFFF; /*Max value of unsigned short value */
			single_cell->col_index_ = x;
			single_cell->row_index_ = y; 
			single_cell->state_ = init;
			
		}	
	}
	
	if (this->display_debug_msg_ == true) std::cout<<" ************Cell info initialization ***************" <<std::endl;
}

bool DecentVoronoi::CheckCurPosInObs()
{
	PartitionInfo* partition_info_single = this->partition_info_[0];
	float x = partition_info_single->part_agent_coor_x_;
	float y =  partition_info_single->part_agent_coor_y_;
	float density = this->GetDensity((int)x, (int)y);
	
	if (density > 0) return false;
	else return true;
}

bool DecentVoronoi::saveDensityMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", map_width_, map_height_, 0xff);
  for (unsigned int iy = 0; iy < map_height_; iy++) 
  {
  for (unsigned int ix = 0; ix < map_width_; ix++) 
    {
		int index = this->GetIndex(ix, iy);
        unsigned char density = this->vorocellDenseMap_[index];
        fprintf(fp, "%d ", density);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

bool DecentVoronoi::saveAgentMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", map_width_, map_height_, 0xff);
  for (unsigned int iy = 0; iy < map_height_; iy++)
  {
    for (unsigned int ix = 0; ix < map_width_; ix++)
    {
		int index = this->GetIndex(ix, iy);
       VoroCell* single_vorocell = this->vorocell_[index];
      fprintf(fp, "%d ", single_vorocell->agentclass_);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

bool DecentVoronoi::Colorized(std::string img_save_dir, std::string label_txt_dir)
{
	 if(is_img_save_ == false) return false; 
     label_text_.open(label_txt_dir);

	 std::string label_text_s;
	 while(std::getline(label_text_,label_text_s))
	 {
	 
	    std::istringstream ss(label_text_s);
	 
	    std::string word;
	 
	    std::vector<int> color_temp;  /*red green blue*/
	 
	    while (ss >> word)
	    {
		    color_temp.push_back(std::stoi(word));
	    }
	    this->color_map_label_.push_back(color_temp);
	 }
	
	 cv::Mat labelImg = cv::Mat::zeros(this->map_height_, this->map_width_, CV_8UC3);

	 for (int x = 0; x<this->map_width_;x++)
	 {
		 for (int y = 0; y<this->map_height_;y++)
		 {
			 VoroCell* vorocell_temp = GetSingleCellByIndex(x, y);
			 unsigned short  agent_index = vorocell_temp->agentclass_;			 
			 float density = this->GetDensity(x,y);
			 
			 if(agent_index == 0xFFFF)
			 {
				 labelImg.at<cv::Vec3b>(y,x)[0] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[1] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[2] = 255;

			 }
			 else
			 {
				 std::vector<int> color_load = this->color_map_label_[agent_index]; /*red green blue*/
				 labelImg.at<cv::Vec3b>(y,x)[0] = (unsigned char)color_load.at(0);
				 labelImg.at<cv::Vec3b>(y,x)[1] = (unsigned char)color_load.at(1);
				 labelImg.at<cv::Vec3b>(y,x)[2] = (unsigned char)color_load.at(2);

				 
			 }
			 if(density == 0)  // if the point is obstacle, set wight color
			 {
				 labelImg.at<cv::Vec3b>(y,x)[0] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[1] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[2] = 255;
				 
			 }
		 }
		 
	 }


	for(int i =0; i <this->CurAgentNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		
		unsigned short centroid_x = (unsigned short)std::round(partition_info_single->part_centroid_x_);
		unsigned short centroid_y = (unsigned short)std::round(partition_info_single->part_centroid_y_);
		
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[0] = 0;
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[1] = 0;
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[2] = 255;
		
		unsigned short agent_cen_x = (unsigned short)std::round(partition_info_single->part_agent_coor_x_);
		unsigned short agent_cen_y = (unsigned short)std::round(partition_info_single->part_agent_coor_y_);
		unsigned short radius = (unsigned short) partition_info_single->part_cov_radius_;
		
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[0] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[1] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[2] = 0;
		
		if(this->is_hete_cov_radius_) cv::circle(labelImg, cv::Point(agent_cen_x, agent_cen_y), radius, cv::Scalar(0,0,0));
		
	}


	 cv::imwrite(img_save_dir, labelImg);
	 
	 return true;
}


bool DecentVoronoi::saveSingleVoro(std::string file_name, int x, int y)
{
  FILE *fp = fopen(file_name.c_str(), "w");
  
  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", x, y, 0xff);
 
  int index = this->GetIndex(x, y);
      VoroCell* single_vorocell = this->vorocell_[index];
      fprintf(fp, "is_edge_ : %d \n", single_vorocell->is_edge_);
      fprintf(fp, "sq_dist_ : %u  \n", single_vorocell->sq_dist_);
	  fprintf(fp, "agentclass_ : %d  \n", single_vorocell->agentclass_);
	  fprintf(fp, "col_index_ : %d  \n", single_vorocell->col_index_);
	  fprintf(fp, "row_index_ : %d  \n", single_vorocell->row_index_);
	  fprintf(fp, "state_ : %d  \n", single_vorocell->state_);

   fclose(fp);
  return true;
}


void DecentVoronoi::InitializeDensityMap()
{
	int x,y;
	int area_wo_density = 0;
	float density;
	
	if((this->is_uniform_density_ == true)||(this->vorocellDenseMapExtPtr_ == NULL))
	{
	   density = 1.0/255.0;
	   for(x= 0;x<this->map_width_;x++)
	   {
		   for(y=0;y<this->map_height_;y++)
		   {
			    int index = this->GetIndex(x, y);
			    this->vorocellDenseMap_[index] = density;
			
			    this->TotalMass_ = this->TotalMass_ + density;
				this->TotalArea_ = this->TotalArea_ + 1;
				
				if(density <= 0)   this->vorocellObsMap_[index] = 1;
				else   this->vorocellObsMap_[index] = 0;
		   }	
	   }
	   density_avg = 1;
	}
	else
	{
		memcpy(this->vorocellDenseMap_, this->vorocellDenseMapExtPtr_, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
	
		for(x= 0;x<this->map_width_;x++)
		{
			for(y=0;y<this->map_height_;y++)
			{
				density = this->GetDensity(x,y);
				int index = this->GetIndex(x, y);
				//std::cout<<" row: "<< y << " col: "<< x << " density: "<< (float)density <<std::endl;
				this->TotalMass_ = this->TotalMass_ + density;
				this->TotalArea_ = this->TotalArea_ + 1;
				
				area_wo_density = area_wo_density + 1;
				if(density <= 0)   this->vorocellObsMap_[index] = 1;
				else   this->vorocellObsMap_[index] = 0;
			}
		}
	    density_avg = 	this->TotalMass_/area_wo_density;
	}
	
	this->inhibit_dropout_ = false;
	this->inhibit_dropout_cnt = 0;
	if (this->display_debug_msg_ == true) std::cout<<" ************Initialize density map success ***************" <<std::endl;
}


void DecentVoronoi::UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr)
{
	memcpy(this->vorocellDenseMap_, vorocellDenseMapExtPtr, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
    //memcopy
	this->TotalMass_ = 0;
	
	for(int x= 0;x<this->map_width_;x++)
	{
		for(int y=0;y<this->map_height_;y++)
		{
			float  density = this->GetDensity(x,y);
			int index = this->GetIndex(x, y);
			this->TotalMass_ = this->TotalMass_ + density;
				
			if(density <= 0)   this->vorocellObsMap_[index] = 1;
			else  this->vorocellObsMap_[index] = 0;
		}
	}
}

float DecentVoronoi::GetDensity(int x, int y)
{
	int index = this->GetIndex(x, y);
	return ((float)this->vorocellDenseMap_[index])/255.0;
}

float DecentVoronoi::CoverageMetric()
{
	if(this->is_hete_cov_radius_==true)
	{
		float var = 0;
		for(int i =0; i <this->CurAgentNum_;i++)
	   {
		   PartitionInfo* partition_info_single = this->partition_info_[i];
		   float in_area = partition_info_single->part_radius_in_area_;
		   float radius = partition_info_single->part_cov_radius_;
		   float max_area = PI*radius*radius;
		   float temp_var = in_area/max_area - 1.0;
		   var = std::abs(temp_var) + var;
	   }
	   var = var/this->CurAgentNum_;
	   return var;

	}
	else
	{
	   float ave_mass = this->CovrdMass_/this->CurAgentNum_;
	   float var = 0;
	   for(int i =0; i <this->CurAgentNum_;i++)
	   {
		   PartitionInfo* partition_info_single = this->partition_info_[i];
		   float mass = partition_info_single->part_mass_;
		   float temp_var = mass/ave_mass - 1.0;
		   var = std::abs(temp_var) + var;
	   }
	   float NotCovrdMass = this->TotalMass_ - this->CovrdMass_;
	   //float NotCovrdMass = 0;
	   
	   if( NotCovrdMass == 0)   var = var/this->CurAgentNum_;
	   else var = var/this->CurAgentNum_ + NotCovrdMass/(ave_mass*this->CurAgentNum_);
	
	   return var;		
		
	}
}


bool DecentVoronoi::ExpandedVoronoi(int cur_robot_num, bool is_propagation_animation, std::string img_dir, std::string label_dir)
{
	unsigned short agent_index;
	VoroCell* vorocell_temp;
	
	this->dropout_agent_num_ = 0;

	if (this->display_debug_msg_ == true) std::cout<<" ******************** DynamicVoronoi::ExpandedVoronoi start ******************* " << std::endl;
	this->CurAgentNum_ = cur_robot_num;

	for(int i =0; i <this->CurAgentNum_;i++)
	{
		
		PartitionInfo* partition_info_single = this->partition_info_[i];
		agent_index = partition_info_single->part_agentclass_;
		float radius = partition_info_single->part_cov_radius_;
		
		if (partition_info_single->is_dropout_ == true) 
		{
			this->dropout_agent_num_ = this->dropout_agent_num_ + 1; 
			continue;
		}


		 
		unsigned short agent_cen_x = (unsigned short)std::round(partition_info_single->part_agent_coor_x_);
		unsigned short agent_cen_y = (unsigned short)std::round(partition_info_single->part_agent_coor_y_);
		
		vorocell_temp = GetSingleCellByIndex(agent_cen_x,agent_cen_y);
			
		vorocell_temp->sq_dist_ = 0;
		vorocell_temp->agentclass_ = agent_index;
		vorocell_temp->agent_cen_x_ = agent_cen_x;
		vorocell_temp->agent_cen_y_ = agent_cen_y;
		vorocell_temp->state_ = completed;
		
		//int sq_dist_prev = vorocell_temp->sq_dist_;
		
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
				
		// Propagate the cells from the initial cell for each agent
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row, radius);		
	}


	
	int i = 0;
	int f = 0;
    
	if(is_propagation_animation == true)
	{		
       //std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	   std::string list_dir = img_dir + "img_file_list.txt";
	   img_list.open(list_dir);   
	}
	
   // Propagate the cells from the queued cell 	
	std::vector<int>  xy_point_int;
 
	while (this->ProccessingQueue_.size()>0)
	{
		xy_point_int = this->ProccessingQueue_.front();
		this->ProccessingQueue_.pop();
		
	    //std::cout<<"xy_point_int "<<xy_point_int[0]<<"  "<<xy_point_int[1]<<" queue size : "<<this->ProccessingQueue_.size()<<" empty :"<<this->ProccessingQueue_.empty()<<std::endl;
		
		vorocell_temp = GetSingleCellByIndex(xy_point_int[0], xy_point_int[1]);
		vorocell_temp->state_ = completed;
		
		unsigned short agent_cen_x = vorocell_temp->agent_cen_x_;
		unsigned short agent_cen_y = vorocell_temp->agent_cen_y_;
	
		agent_index = vorocell_temp->agentclass_;
		
		PartitionInfo* partition_info_single = this->partition_info_[agent_index];
		float radius = partition_info_single->part_cov_radius_;
			
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
		
		// Propagate the cells from the initial cell for each agent
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row, radius);
		
		if((is_propagation_animation==true)&&((i%propagation_frame_interval) == 0))
		{	
	
	        this->CentroidCal();
	
		    //std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	        //std::string label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";			
			std::string img_save_dir = img_dir + "map" + std::to_string(f) +".png";
			
			this->Colorized(img_save_dir,label_dir);
			
			img_list<<"map" + std::to_string(f) +".png"<<std::endl;
			
			f = f +1;			
		}
        i = i +1;
	}
	
	
	if (this->display_debug_msg_ == true) std::cout<<" ******************** DynamicVoronoi::ExpandedVoronoi end ******************* " << std::endl;

    this->is_propa_completed_ = true;

	return true;

}

void DecentVoronoi::Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row, float radius)
{
	VoroCell* vorocell_temp;
	std::vector<int> queue_xy;
      
	 int col_move[4] = {1, 0, -1, 0};
	 int row_move[4] = {0, 1, 0, -1};

	 
     for (int k = 0; k <4; k ++)
	 {
		 int new_col = col+col_move[k];
		 int new_row = row+row_move[k];
		 
		 if((new_col < 0)||(new_row < 0)||(new_row == this->map_height_)||( new_col == this->map_width_)) continue;
		 
		 vorocell_temp = GetSingleCellByIndex(new_col, new_row);
		 
		float ObsPathPenalty = CulObsCell(agent_cen_x, agent_cen_y, new_col, new_row);
		 
		if((is_propa_connected_area_ == true)&&(ObsPathPenalty >= 1))
		{
			/* No action */
		}
		else{
			float sq_dist_raw;
			
			if(this->is_hete_cov_radius_ == false)
			{
		        sq_dist_raw = (agent_cen_x - (new_col))*(agent_cen_x - (new_col)) + (agent_cen_y - new_row)*(agent_cen_y - new_row); 
			}
			else
			{
				sq_dist_raw = ((agent_cen_x - (new_col))*(agent_cen_x - (new_col)) + (agent_cen_y - new_row)*(agent_cen_y - new_row))/(radius*radius); 
			}
			
			float sq_dist_new = sq_dist_raw;
			
		    if(vorocell_temp->state_ == init)
		    {
			    vorocell_temp->agentclass_ = agent_index;
		        vorocell_temp->agent_cen_x_ = agent_cen_x;
		        vorocell_temp->agent_cen_y_ = agent_cen_y;	
			    vorocell_temp->sq_dist_ = sq_dist_new;
			    vorocell_temp->state_ = queued;
			    queue_xy.push_back(new_col);
			    queue_xy.push_back(new_row);
			    this->ProccessingQueue_.push(queue_xy);
			    queue_xy.clear();		
		    }
		    else
		    {
			    if(vorocell_temp->sq_dist_ >sq_dist_new)
			    {
			       vorocell_temp->is_edge_ = false;
			       vorocell_temp->agentclass_ = agent_index;
	               vorocell_temp->agent_cen_x_ = agent_cen_x;
	               vorocell_temp->agent_cen_y_ = agent_cen_y;
			       vorocell_temp->sq_dist_ = sq_dist_new;
			       vorocell_temp->state_ = queued;
			       queue_xy.push_back(new_col);
			       queue_xy.push_back(new_row);
			       this->ProccessingQueue_.push(queue_xy);
			       queue_xy.clear();
			    }
                else if(vorocell_temp->sq_dist_  == sq_dist_new)
			    {
				    if(vorocell_temp->agentclass_ < agent_index)
				    {
					    vorocell_temp->agentclass_ = agent_index;
		                vorocell_temp->agent_cen_x_ = agent_cen_x;
		                vorocell_temp->agent_cen_y_ = agent_cen_y;
			            vorocell_temp->sq_dist_ = sq_dist_new;	
				    }
				    //vorocell_temp->is_edge_ = true;
				    //std::cout<<"line 569"<<" col :"<<col-1<<" row :"<<row<<std::endl;
			    }
		    }
		}
	 }
}

void DecentVoronoi::CentroidCal()
{
	VoroCell* vorocell_temp;
	PartitionInfo* partition_info_single;
	
	if (this->display_debug_msg_ == true) std::cout<<" ******************** DynamicVoronoi::CentroidCal start ******************* " << std::endl;
	
	/* initialize variables for centroid calculation */
	for(int i =0; i <this->CurAgentNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		partition_info_single->part_mass_ = 0;
		partition_info_single->part_area_ = 0;
		partition_info_single->part_radius_in_area_ = 0;
		partition_info_single->centroid_momentsum_x_ = 0;
		partition_info_single->centroid_momentsum_y_ = 0;			
	}

    /* calcuate variables for centroid  */
	for(int col= 0;col<this->map_width_;col++)
	{
		for(int row=0;row<this->map_height_;row++)
		{
			vorocell_temp = GetSingleCellByIndex(col, row);
			unsigned short agentclass = vorocell_temp->agentclass_;
			
			if(agentclass == 0xFFFF) continue;
			
			partition_info_single = this->partition_info_[agentclass];
			float density_temp = GetDensity(col, row);
			partition_info_single->part_mass_ = partition_info_single->part_mass_ + density_temp;
            partition_info_single->part_area_ = partition_info_single->part_area_ + 1;			
			partition_info_single->centroid_momentsum_x_ = partition_info_single->centroid_momentsum_x_ + ((float)col)*density_temp;
			partition_info_single->centroid_momentsum_y_ = partition_info_single->centroid_momentsum_y_ + ((float)row)*density_temp;	
		}
		
	}

	/* calculate centroid */
	for(int i = 0; i < this->CurAgentNum_; i++)
	{
		partition_info_single = this->partition_info_[i];
		
		if(partition_info_single->part_mass_ > 0)
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->centroid_momentsum_x_/partition_info_single->part_mass_;
		   partition_info_single->part_centroid_y_ = partition_info_single->centroid_momentsum_y_/partition_info_single->part_mass_;			
		}
		else
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->part_agent_coor_x_;
		   partition_info_single->part_centroid_y_ = partition_info_single->part_agent_coor_y_;	
		}			
		if (this->display_debug_msg_ == true) std::cout<< "class num : "<<i<<" centroid x : "<<partition_info_single->part_centroid_x_<<" centroid y : "<<partition_info_single->part_centroid_y_<<std::endl;
	}
    
	if(this->is_hete_cov_radius_ == true)
	{
	   for(int col= 0;col<this->map_width_;col++)
	   {
		   for(int row=0;row<this->map_height_;row++)
		   {
			   vorocell_temp = GetSingleCellByIndex(col, row);
			   unsigned short agentclass = vorocell_temp->agentclass_;
			
			   if(agentclass == 0xFFFF) continue;
			
			   partition_info_single = this->partition_info_[agentclass];
			   float centroid_x = partition_info_single->part_centroid_x_;
			   float centroid_y = partition_info_single->part_centroid_y_;
			   float radius = partition_info_single->part_cov_radius_;
			   float dis_sq =  (centroid_x - (float)col)*(centroid_x - (float)col) + (centroid_y - (float)row)*(centroid_y - (float)row); 
			   
			   if(dis_sq<=(radius*radius))
			   { partition_info_single->part_radius_in_area_ = partition_info_single->part_radius_in_area_ +1; }
		   }
	   }
	}


	this->CovrdMass_ = 0;
	this->CovrdArea_ = 0;
	this->CvdAreaDivMaxR_ = 0;
	
	for(int i = 0; i < this->CurAgentNum_; i++)
	{
		partition_info_single = this->partition_info_[i];
				
		float mass = partition_info_single->part_mass_;
		this->CovrdMass_ = this->CovrdMass_ + mass;
		
		float area = partition_info_single->part_area_;
		this->CovrdArea_ = this->CovrdArea_ + area;
		
		float in_area = partition_info_single->part_radius_in_area_;
		float radius = partition_info_single->part_cov_radius_;
		float CvdArea = 0;
		if(area == 0) CvdArea = 1;
		else CvdArea = area/(PI*radius*radius);
		this->CvdAreaDivMaxR_ = this->CvdAreaDivMaxR_ + CvdArea;
	}
	//std::cout<<" ******************** DynamicVoronoi::CentroidCal end ******************* " << std::endl;

}

void DecentVoronoi::MoveAgents()
{
	PartitionInfo* partition_info_single;
	int cnt_project_req = 0;
	if (this->display_debug_msg_ == true) std::cout<<" ************DynamicVoronoi::MoveAgents start ***************" <<std::endl;
	

	bool is_Partition_inObs = false;
	partition_info_single = this->partition_info_[0];
		
	if( (partition_info_single->part_mass_ > 0))
	{
		is_Partition_inObs = false;
	}
	else
	{
		is_Partition_inObs = true;
		cnt_project_req++;
	}
		
	if (this->display_debug_msg_ == true) std::cout << " partition_info_single->part_mass : "<< partition_info_single->part_mass_ <<" is_Partition_inObs : "<< is_Partition_inObs << std::endl;
		
	if(is_Partition_inObs == true)
	{
		bool is_proj_success  = false;
            //float ref_dist_sq = this->map_width_*this->map_width_ + this->map_height_*this->map_height_; 
		
		    //float init_agent_coor_x = partition_info_single->part_agent_coor_x_;
		    //float init_agent_coor_y = partition_info_single->part_agent_coor_y_;

		    //bool is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 1, 0, &ref_dist_sq);  // go to east to find non obstacle region
		    //is_proj_success = is_proj_success_temp | is_proj_success;		
		    //is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 0, -1, &ref_dist_sq);  // go to north to find non obstacle region
		    //is_proj_success = is_proj_success_temp | is_proj_success;
		    //is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, -1, 0, &ref_dist_sq);  // go to west to find non obstacle region
		    //is_proj_success = is_proj_success_temp | is_proj_success;
		    //is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 0, 1, &ref_dist_sq);  // go to south to find non obstacle region
		    //is_proj_success = is_proj_success_temp | is_proj_success;


		bool is_proj_success_temp = ProjNearPoint(partition_info_single); 
		is_proj_success = is_proj_success_temp | is_proj_success;

		if(is_proj_success == false)
		{
				//partition_info_single->part_agent_coor_x_ = 0;
				//partition_info_single->part_agent_coor_y_ = 0;
			if (this->display_debug_msg_ == true) std::cout<<" Projection failed, check the density map"<<std::endl;
		}
		else
		{
			   /* after running the FindNearPtNonObs function, the agent position may be relocated */
		    float new_agent_coor_x = partition_info_single->part_agent_coor_x_;
		    float new_agent_coor_y = partition_info_single->part_agent_coor_y_;		
		}
	}
	else
	{
		float mass = partition_info_single->part_mass_;
		float agent_coor_x = partition_info_single->part_agent_coor_x_;
		float agent_coor_y = partition_info_single->part_agent_coor_y_;
		float centroid_x = partition_info_single->part_centroid_x_;
	    float centroid_y = partition_info_single->part_centroid_y_;
			/*agent coordinate update using gradient descent */ /*instead using the partition mass directly, normalized mass was used (mass/Total mass) or inverse normalized mass ((Total mass - mass)/Total mass)*/
			
			/*float coefficient = (this->TotalMass_- mass)/this->TotalMass_;
			/float min_coeff = 0.5;
			
			if(coefficient < min_coeff)
			{
				coefficient = min_coeff;
			}
			coefficient = min_coeff;*/
		    //agent_coor_x = agent_coor_x - 2*coefficient*(agent_coor_x - centroid_x);     
		   // agent_coor_y = agent_coor_y - 2*coefficient*(agent_coor_y - centroid_y);
		agent_coor_x = centroid_x;
		agent_coor_y = centroid_y;
					/*boundary check*/
		if(agent_coor_x<1)
		{
			agent_coor_x = 1;
		}
		else if(agent_coor_x>(this->map_width_-1))
		{ 
			agent_coor_x = this->map_width_-1;
		}
		
		if(agent_coor_y<1)
		{
			agent_coor_y = 1;
		}
		else if(agent_coor_y>(this->map_height_-1))
		{ 
			agent_coor_y = this->map_height_-1;
		}
		
		    /* allocate new agent coordinate */
		partition_info_single->part_agent_coor_x_ = agent_coor_x;
		partition_info_single->part_agent_coor_y_ = agent_coor_y;
			
		    /* insert the index of agent which is on non obstacle region */
		float density = this->GetDensity((int)std::round(partition_info_single->part_agent_coor_x_), (int)std::round(partition_info_single->part_agent_coor_y_));
		if(density>0)
		{
							
				/*std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_x_));
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_y_));
								
			    if(this->AgentCoorOpenSp_.empty()) this->AgentCoorOpenSp_.push_back(AgentCoor_temp);*/
			    	
		}
		else
		{

            this->ProjNearPoint(partition_info_single);				
				
				/* std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_x_));
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_y_));
				
				float density_temp = this->GetDensity((int)std::round(partition_info_single->part_agent_coor_x_), (int)std::round(partition_info_single->part_agent_coor_y_));  // somtimes, the post pos check doesn't work. density should be checked again
				
			    if((this->AgentCoorOpenSp_.empty())&&(density_temp > 0)) this->AgentCoorOpenSp_.push_back(AgentCoor_temp);*/
		}
	}
		
     if (this->display_debug_msg_ == true) std::cout<<" new agent coordinate x : "<<partition_info_single->part_agent_coor_x_<<" new agent coordinate y : "<<partition_info_single->part_agent_coor_y_<<std::endl;
		
		

	
	
	if(cnt_project_req==0)
	{
		this->AgentCoorOpenSp_.clear();
		
	}
}


float DecentVoronoi::CulObsCell(unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row)
{

	float increment_x;
	float increment_y;	
	int sign_conv;
	float CulObs = 0;
	
    int index_init = this->GetIndex(col, row);		
	CulObs = CulObs + (float)this->vorocellObsMap_[index_init];
	
	float x_diff = (float)col - agent_cen_x;
	float y_diff = (float)row - agent_cen_y;
	
	 if((x_diff<1)&&(x_diff>-1)&&(y_diff<1)&&(y_diff>-1)) //prevent singularity
	 {
		   return CulObs;
	  }
	
     if(std::abs(y_diff) > std::abs(x_diff))
     {
		 increment_x = x_diff/y_diff;
		 
		 if(y_diff >= 0)
	     { 
		     increment_y = 1;
			 sign_conv = 1;
	     }
	     else
	     {
		     increment_y = -1;
			 sign_conv = -1;
	     }
		 
		 float pos_x = agent_cen_x;
		 
	     for(float pos_y = agent_cen_y; (row - pos_y)*sign_conv > 0;)
	     {
		     pos_x = pos_x + increment_x*sign_conv;
		     pos_y = pos_y + increment_y;
		
		     if((pos_x < 0)||(pos_x >= this->map_width_)||(pos_y < 0)||(pos_y >= this->map_height_))
		     {
			     continue;
		     }
		
		     int index = this->GetIndex((int)std::round(pos_x), (int)std::round(pos_y));		
		     CulObs = CulObs + (float)this->vorocellObsMap_[index]; 		 
	     }
     }
     else
     {
		 increment_y = y_diff/x_diff;
		 
		 if(x_diff>= 0)
	     { 
		     increment_x = 1;
			 sign_conv = 1;
	     }
	     else
	     {
		     increment_x = -1;
			 sign_conv = -1;
	     }
		 
		 float pos_y = agent_cen_y;
		 
	     for(float pos_x = agent_cen_x; (col - pos_x)*sign_conv > 0;)
	     {
		     pos_x = pos_x + increment_x;
		     pos_y = pos_y + increment_y*sign_conv;
		
		     if((pos_x < 0)||(pos_x >= this->map_width_)||(pos_y < 0)||(pos_y >= this->map_height_))
		     {
			     continue;
		     }
		
		     int index = this->GetIndex((int)std::round(pos_x), (int)std::round(pos_y));		
		     CulObs = CulObs + (float)this->vorocellObsMap_[index]; 		 
	     }
     }		 

		
	return CulObs;
}

bool DecentVoronoi::ProjNearPoint(PartitionInfo* partition_info_single)
{
    int init_x = std::round(partition_info_single->part_agent_coor_x_);
	int init_y = std::round(partition_info_single->part_agent_coor_y_);
    bool success = true;
	
	std::queue<std::vector<int>> PosQueue;  
    
	unsigned char* VisitCheck;
	
	VisitCheck = new unsigned char[this->map_height_*this->map_width_];
	memset(VisitCheck,0,this->map_height_*this->map_width_*sizeof (unsigned char));
	AgentPosPropagation(PosQueue, VisitCheck, init_x, init_y);
	
	int cnt =0;
	
	while (PosQueue.size()>0)
	{
		cnt++;
		std::vector<int> pos_front = PosQueue.front(); 
		PosQueue.pop(); 
		 
        float density = this->GetDensity(pos_front[0], pos_front[1]);
	    if(density > 0) 
		{
			partition_info_single->part_agent_coor_x_ = pos_front[0];
			partition_info_single->part_agent_coor_y_ = pos_front[1];
			break;
		}
        else 	AgentPosPropagation(PosQueue, VisitCheck, pos_front[0], pos_front[1]);
		
		if(cnt > (this->map_width_*this->map_height_))
		{
			success = false;
			break;
		}
	}


	delete VisitCheck;
	
	return success;
	
}

void DecentVoronoi::AgentPosPropagation(std::queue<std::vector<int>>& PosQueue, unsigned char* VisitCheck, int init_x, int init_y)
{	
	int x_move[4] ={1, 0, -1, 0};
	int y_move[4] ={0, 1, 0, -1};
	std::vector<int> pos_xy;
	
	for (int k =0; k<4; k++)
	{
		int x = init_x + x_move[k];
		int y = init_y + y_move[k];
		
		if((x == this->map_width_)||(y == this->map_height_)||(x < 0)||(y < 0))
		{
			continue;
		}
		
		unsigned char  visited = VisitCheck[x+y*this->map_width_];
		if (visited == 1) continue;
		VisitCheck[x+y*this->map_width_] = 1;
		pos_xy.push_back(x);
		pos_xy.push_back(y);
		PosQueue.push(pos_xy);
        pos_xy.clear();	
	}
}

bool DecentVoronoi::FindNearPtNonOvL(PartitionInfo* partition_info)
{
	bool end_while_loop = false;
	bool is_in_range_ = true;
	float init_agent_coor_x = partition_info->part_agent_coor_x_;
	float init_agent_coor_y = partition_info->part_agent_coor_y_;
	bool is_success_ = true;

     int mul = std::rand()%5;

	int x_move[4] ={1, 0, -1, 0};
	int y_move[4] ={0, 1, 0, -1};
	
	for (int k =0; k<4; k++)
	{
	    float new_agent_coor_x = init_agent_coor_x + x_move[k]*mul;
	    float new_agent_coor_y = init_agent_coor_y + y_move[k]*mul;
	
	    if((new_agent_coor_x>=0)&&(new_agent_coor_x<=this->map_width_-1)&&(new_agent_coor_y>=0)&&(new_agent_coor_y<=this->map_height_-1))
	    {
			/*intentially empty*/
	    }
	    else
	    {
			is_in_range_ = false;
	    }
    
	    float density = 0;
	    if(is_in_range_ == true) density = this->GetDensity((int)std::round(new_agent_coor_x), (int)std::round(new_agent_coor_y));
    
	    if(density >0)
	    {
		    partition_info->part_agent_coor_x_ = (int)std::round(new_agent_coor_x);
		    partition_info->part_agent_coor_y_ = (int)std::round(new_agent_coor_y);
			break;
	    }
	    else
	    {
		    is_success_ = false;
	    }
			
	}
	
	return is_success_;
	
}

bool DecentVoronoi::FindNearPtNonObs(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float increment_x, float increment_y, float* ref_dist_sq)
{
	bool end_while_loop = false;
	bool is_success_ = true;
	float new_agent_coor_x = init_agent_coor_x_local;
	float new_agent_coor_y = init_agent_coor_y_local;
	
	while(!end_while_loop)
	{
		new_agent_coor_x = new_agent_coor_x + increment_x;
		new_agent_coor_y = new_agent_coor_y + increment_y;
		
		if((new_agent_coor_x>=0)&&(new_agent_coor_x<=this->map_width_-1)&&(new_agent_coor_y>=0)&&(new_agent_coor_y<=this->map_height_-1))
		{
			/*intentially empty*/
		}
		else
		{
			 is_success_ = false;
             end_while_loop = true;
			 break;
			
		}
		
		
		float density = this->GetDensity((int)std::round(new_agent_coor_x), (int)std::round(new_agent_coor_y));
		
		if(density >0)
		{
			float dist_sq_temp = (new_agent_coor_x - init_agent_coor_x_local)*(new_agent_coor_x - init_agent_coor_x_local)+ (new_agent_coor_y - init_agent_coor_y_local)*(new_agent_coor_y - init_agent_coor_y_local);
			
			if(*ref_dist_sq > dist_sq_temp)
			{
				*ref_dist_sq = dist_sq_temp;
				partition_info_single->part_agent_coor_x_ = (int)std::round(new_agent_coor_x);
				partition_info_single->part_agent_coor_y_ = (int)std::round(new_agent_coor_y);
			}
			else
			{
				/*intentionally empty*/
			}
			
			end_while_loop = true;
			break;
		}
	}
	

	return is_success_;
	
}


bool DecentVoronoi::FindNearPtNonObs_R2(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float* ref_dist_sq)
{
	bool is_success_ = true;	

    float gradient;
	float x_increment;
	
	if(!this->AgentCoorOpenSp_.empty())
	{

		std::vector<std::vector<float>>::iterator it;
			
		for(it=this->AgentCoorOpenSp_.begin(); it!=this->AgentCoorOpenSp_.end(); it++)
		{
			float agent_coor_x_notobs = it->at(0);
			float agent_coor_y_notobs = it->at(1);
			
			bool end_while_loop = false;
			float new_agent_coor_x = init_agent_coor_x_local;
	        float new_agent_coor_y = init_agent_coor_y_local;
			
			gradient = (agent_coor_y_notobs - init_agent_coor_y_local)/(agent_coor_x_notobs - init_agent_coor_x_local);
			
			if((agent_coor_x_notobs - init_agent_coor_x_local) >= 0)
			{
				x_increment = 1;
			}
			else
			{
				x_increment = -1;
			}
			
			//std::cout<<" debug  *** "<<"init_agent_coor_x_local : "<<init_agent_coor_x_local<<"init_agent_coor_y_local : "<<init_agent_coor_y_local<<"gradient : "<< gradient <<" agent_coor_x_notobs : "<< agent_coor_x_notobs << " agent_coor_y_notobs : "<< agent_coor_y_notobs <<std::endl;
			
			while(!end_while_loop)
			{
			    new_agent_coor_x = new_agent_coor_x + x_increment;
		        new_agent_coor_y = new_agent_coor_y + gradient;
				
				
				if((new_agent_coor_x>=0)&&(new_agent_coor_x<=this->map_width_-1)&&(new_agent_coor_y>=0)&&(new_agent_coor_y<=this->map_height_-1))
		        {
			         /*intentially empty*/
		        }
		        else
		        {
					std::cout<<" debug  *** "<<"break  "<< std::endl;
			         is_success_ = false;
                     end_while_loop = true;
			         break;
		        }
				
				
				float density = this->GetDensity((int)std::round(new_agent_coor_x), (int)std::round(new_agent_coor_y));
						
		        if(density >0)
		        {
			        float dist_sq_temp = (new_agent_coor_x - init_agent_coor_x_local)*(new_agent_coor_x - init_agent_coor_x_local)+ (new_agent_coor_y - init_agent_coor_y_local)*(new_agent_coor_y - init_agent_coor_y_local);
			        
	
					
			        if(*ref_dist_sq > dist_sq_temp)
			        {
				       *ref_dist_sq = dist_sq_temp;
				       partition_info_single->part_agent_coor_x_ = (int)std::round(new_agent_coor_x);
				       partition_info_single->part_agent_coor_y_ = (int)std::round(new_agent_coor_y);
			        }
			        else
			        {
				        /*intentionally empty*/
			        }
			
			        end_while_loop = true;
			        break;
		        }	
			}
		}	
	}
	else
	{
		is_success_ = false;
	}

	return is_success_;		
	
}

bool DecentVoronoi::AgentDropOut()
{
	PartitionInfo* partition_info_single;
	float DropOutTolerance = 100000.0;
	
	bool active = false;
	
	if (this->CurAgentNum_ == 1) return active;
	else if(this->dropout_agent_num_+1 == this->CurAgentNum_) return active;
	
	this->DropOutWeight_ = (float)this->CurAgentNum_/(float)(this->CurAgentNum_-1);
	
	if(this->is_hete_cov_radius_ == false)  DropOutTolerance = this->CovrdMass_*this->DropOutWeight_/(float)this->CurAgentNum_;
	else DropOutTolerance = this->CvdAreaDivMaxR_*this->DropOutWeight_/(float)this->CurAgentNum_;

	float DropOutVar;
	partition_info_single = this->partition_info_[0];
		
	if(this->is_hete_cov_radius_ == false) DropOutVar = partition_info_single->part_mass_;
	else 
	{
		float area = partition_info_single->part_area_; 
		float radius = partition_info_single->part_cov_radius_; 
		DropOutVar = area/(PI*radius*radius);
	}
		
	if(DropOutVar > DropOutTolerance)
	{
		     active = true; 
	}

	return active;
}
