#include "control_to_stair/stair_container.h"
#include <iostream>
#include <fstream>
#include<sstream>

/*constructor and destructor*/
STAIR_CONTAINER::STAIR_CONTAINER(std::string probability_type):_probability_type(probability_type)
{
   this->_probability = 0; 
   this->_count = 0; 
   this->_stair_container_x_coor = 0;    
   this->_stair_container_y_coor = 0;  
   this->_stair_container_z_coor = 0; 
   this->_stair_container_x_coor_old = 0;    
   this->_stair_container_y_coor_old = 0;  
   this->_stair_container_z_coor_old = 0; 
   this->_container_empty_flag = true;
	
}
STAIR_CONTAINER::~STAIR_CONTAINER(){}

void STAIR_CONTAINER::stair_container_clear()
{
   this->_probability = 0; 
   this->_count = 0; 
   this->_stair_container_x_coor = 0;    
   this->_stair_container_y_coor = 0;  
   this->_stair_container_z_coor = 0; 
   this->_stair_container_x_coor_old = 0;    
   this->_stair_container_y_coor_old = 0;  
   this->_stair_container_z_coor_old = 0; 
   this->_container_empty_flag = true;
	
   //this->_prob_factor_geo_series = 0;
   //this->_forget_factor_geo_series = 0;
   //this->_prob_factor_linear = 0;
   //this->_ forget_factor_linear = 0;
}

bool STAIR_CONTAINER::stair_container_probforget_factor_update_geo_series(float prob_factor, float forget_factor)
{
	
	if(prob_factor<1)
	{
	   this->_prob_factor_geo_series = 1;	
	}
	else
	{
		this->_prob_factor_geo_series = prob_factor;
	}
   
   if(forget_factor>1)
   {
	   this->_forget_factor_geo_series = 1;
   }
   else
   {
	   this->_forget_factor_geo_series = forget_factor;   
   }
    
 
   std::string  temp ="geo_series";
   
   bool method_check;
   
   if(_probability_type.compare(temp) == 0)
   {
	   method_check =true;
   }
   else
   {
	    method_check = false; 
	}
   
   return method_check;
}


bool STAIR_CONTAINER::stair_container_probforget_factor_update_linear(int prob_factor, int forget_factor)
{
	
	if(prob_factor<0)
	{
		this->_prob_factor_linear = 0;
	}
	else
	{
	    this->_prob_factor_linear = prob_factor;	
	}
   
   if(forget_factor>0)
   {
	   this->_forget_factor_linear = 0;
   }
   else
   {
	   this->_forget_factor_linear = forget_factor;   
   }
   
    
 
   std::string  temp ="linear";
   
   bool method_check;
   
   if(_probability_type.compare(temp) == 0)
   {
	   method_check =true;
   }
   else
   {
	    method_check = false; 
	}
   
   return method_check;
}

void  STAIR_CONTAINER::stair_container_pos_return(float *x, float *y, float *z)
{
	*x = this->_stair_container_x_coor;
	*y = this->_stair_container_y_coor;
	*z = this->_stair_container_z_coor;
	
		//std::cout<<"this_coor_x return :"<<this->_stair_container_x_coor<<std::endl;
	
} 

void STAIR_CONTAINER::stair_container_prob_pos_update(float x, float y, float z, bool prob_count_update)
{ 
    this->_pose_update_weight = this->_probability/PROBABILITY_MAX*(this->_pose_update_weight_max - this->_pose_update_weight_min) + this->_pose_update_weight_min;
	
	
	if(this->_container_empty_flag==true)
	{
	      this->_stair_container_x_coor = x;
	      this->_stair_container_y_coor = y;
	      this->_stair_container_z_coor = z;
	}
	else
	{
		this->_stair_container_x_coor = this->_stair_container_x_coor_old*this->_pose_update_weight + (1-this->_pose_update_weight)*x;
		this->_stair_container_y_coor = this->_stair_container_y_coor_old*this->_pose_update_weight + (1-this->_pose_update_weight)*y;
		this->_stair_container_z_coor = this->_stair_container_z_coor_old*this->_pose_update_weight + (1-this->_pose_update_weight)*z;
	}
	
	 this->_stair_container_x_coor_old = this->_stair_container_x_coor;
	 this->_stair_container_y_coor_old = this->_stair_container_y_coor;
	 this->_stair_container_z_coor_old = this->_stair_container_z_coor;
	
	//std::cout<<"this_coor_x : "<<this->_stair_container_x_coor<<std::endl;
	
	this->_count ++;
	
	if(prob_count_update == true)
	{   
		this->stair_container_prob_update();
	}
}

void STAIR_CONTAINER::stair_container_prob_update()
{
	if(_probability_type.compare("linear")==0)
	{
		this->_probability = this->_probability + this->_prob_factor_linear;		
	}
    else if(_probability_type.compare("geo_series")==0)
	{
		if(this->_probability < 1)
		{
			this->_probability = 1;
		}
		
		this->_probability = this->_probability*this->_prob_factor_geo_series;
	}
	
	if(this->_probability > 100)
	{
		this->_probability = 100;
	}
	
	this->_container_empty_flag = false;
}


void STAIR_CONTAINER::stair_container_prob_forget()
{
	if(_probability_type.compare("linear")==0)
	{
		this->_probability = this->_probability - this->_forget_factor_linear;		
	}
    else if(_probability_type.compare("geo_series")==0)
	{
		if(this->_probability < 0.2)
		{
			this->_probability = 0;
		}
		
		this->_probability = this->_probability*this->_forget_factor_geo_series;
	}
	
	if(this->_probability <= 0)
	{
		this->stair_container_clear();
		this->_probability = 0;
	}
	
	if(this->_probability == 0)
	{
		this->_container_empty_flag = true;
	}
	
}

float STAIR_CONTAINER::stair_container_pob_return()
{
	return this->_probability;
}

bool STAIR_CONTAINER::stair_container_empty_flag_return()
{
	return this->_container_empty_flag;
}



	