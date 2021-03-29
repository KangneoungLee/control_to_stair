#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Int8.h>
#include <mutex>
#include <boost/bind.hpp>
#include <stair_detection/pixel_stairlocation.h>
#include "control_to_stair/stair_container.h"
//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/server/simple_action_server.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib_msgs/GoalID.h>

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

std::mutex _lock;

class PIXEL_CONTROL_TO_STAIR_ROS{
	
	private:
		 
		 ros::NodeHandle main_nh;
         ros::NodeHandle param_nh;
	     ros::Rate* _loop_rate;

         ros::Subscriber _stair_center_pixel_sub;
		 ros::Subscriber  _detec_trigger_sub;
		 
		  int _detec_trigger_flag;
		 
		  ros::Subscriber _sc_flag;
		 ros::Publisher _cmd_vel_pub;
		 
		  ros::Publisher _flag_pub;
		 
		 int _TP_rotation_start_threshold;
		 int _TP_rotation_end_threshold;
		 float _TP_angular_vel_p_gain;
		 float _TP_weight_of_lpf;
		 float _TP_linear_vel;
		 float _TP_duration_for_reset_target;
		 float _TP_stair_case_close_tolerance;
		 int _TP_detection_count_th;
		 int _TP_target_row_center;
		
		 
		 int _detection_count = 0;
		 unsigned char _control_state_num=0; /* 0 : initial 1: working 2: completed*/
		 unsigned char _control_state_num_old=0;
		 
		 float _filtered_row_center = 0;
		 float  _filtered_row_center_old;
		 
		 float _filtered_depth_data = 0;
		 float _filtered_depth_data_old;
		 
		 bool _rotation_command_working_flag = false;
		 bool _vel_reset_flag = false;
		 bool _cmd_vel_ok_flag = false;
		 bool _control_num_2_delay_flag = false;
		 
		 
		 bool _TP_presearch_enable_flag;
		 float _TP_presearch_duration_for_interval;
		 int _TP_presearch_interval_num;
		 float _TP_presearch_ang_vel;
		 
         int _total_stairdetec_num_array[100];
		 int _presearch_interval_count = 0;
		 int _presearch_interval_stairdetec_count_start = 0;
		 int _presearch_interval_stairdetec_count_end = 0;
		 
		 int _stairdetec_max_index = 0;
		 int _stairdetec_max_count=0;
		 
		 bool _presearch_rotat_flag = false;
		 bool _presearch_pause_flag = false;

		 bool _presearch_init_comp_flag = false;
		 bool _presearch_final_comp_flag = false; 
		 bool _presearch_ok_flag = false;
		 
		  ros::Time _presearch_pattern_elapse_time ;
		 
		 ros::Time _presearch_init_time;
		 
		 //actionlib_msgs::GoalID cancelMoveBaseAction;
         //actionlib::SimpleClientGoalState moveBaseGoalState(actionlib::SimpleClientGoalState::PENDING);
		 
		 int _update_rate;
	
	public:
	
	    void run();
		void loop();
		void presearching();
		void presearching_reset();
		void control_reset();
		void executeCB(const stair_detection::pixel_stairlocationConstPtr &stair_pixel_pose_vec, int value);
		
		void executeCB_detect_trigger_flag(const std_msgs::Int8::ConstPtr& msg,int value);
	 
		/*constructor and destructor*/
	    PIXEL_CONTROL_TO_STAIR_ROS(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~PIXEL_CONTROL_TO_STAIR_ROS();
		
		
		ros::Time _update_time;
		

};

PIXEL_CONTROL_TO_STAIR_ROS::PIXEL_CONTROL_TO_STAIR_ROS(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh(m_nh),param_nh(p_nh)
{
	int update_rate = 10;
	
	 std::string  stair_center_pixel_sub_topic = "front_cam/camera/center_pixel";
	 std::string  control_to_stair_cmd_vel_topic = "stair_nav/cmd_vel_stair";
	 std::string  control_to_stair_flag_topic = "stair_nav/cmd_vel_stair_flag";
	 std::string detect_trigger_flag_sub_topic ="detec_logic_trigger";
	 
	 int TP_rotation_start_threshold = 30; /*unit : pixel*/
	 int TP_rotation_end_threshold = 15;  /*unit : pixel*/
	 float TP_angular_vel_p_gain = 0.001;
	 float TP_duration_for_reset_target =2; /*unit : sec*/
	 float TP_weight_of_lpf = 0.2;
	 float TP_linear_vel = 1;    /*unit : m/s*/
	 int TP_target_row_center =424;  /*unit : pixel */
	 float TP_stair_case_close_tolerance = 2; /*unit : m*/

	 
	 
	 int TP_detection_count_th =3;
	 
	
	 param_nh.getParam("stair_center_pixel_sub_topic",stair_center_pixel_sub_topic);
	 param_nh.getParam("control_to_stair_cmd_vel_topic",control_to_stair_cmd_vel_topic);
	 param_nh.getParam("control_to_stair_flag_topic",control_to_stair_flag_topic);
	 param_nh.getParam("detect_trigger_flag_sub_topic",detect_trigger_flag_sub_topic);
	  
	  
	 param_nh.getParam("TP_rotation_start_threshold",TP_rotation_start_threshold);
	 param_nh.getParam("TP_rotation_end_threshold",TP_rotation_end_threshold);
	 param_nh.getParam("TP_angular_vel_p_gain",TP_angular_vel_p_gain);
	 param_nh.getParam("TP_duration_for_reset_target",TP_duration_for_reset_target);
	 param_nh.getParam("TP_weight_of_lpf",TP_weight_of_lpf);
	 param_nh.getParam("TP_linear_vel",TP_linear_vel);
	 param_nh.getParam("TP_target_row_center",TP_target_row_center);
	 param_nh.getParam("TP_stair_case_close_tolerance",TP_stair_case_close_tolerance);
	 param_nh.getParam("TP_detection_count_th",TP_detection_count_th);
	
	 
	 
	 param_nh.getParam("stair_center_pixel_sub_topic",stair_center_pixel_sub_topic);
	 
	 param_nh.getParam("update_rate",update_rate);
	  
	 this->_TP_rotation_start_threshold = TP_rotation_start_threshold;
	 this->_TP_rotation_end_threshold = TP_rotation_end_threshold;
	 this->_TP_angular_vel_p_gain = TP_angular_vel_p_gain;
	 this->_TP_duration_for_reset_target = TP_duration_for_reset_target;
	 this->_TP_weight_of_lpf = TP_weight_of_lpf;
	 this->_TP_linear_vel = TP_linear_vel;
	 this->_TP_target_row_center = TP_target_row_center;
	 this->_TP_stair_case_close_tolerance = TP_stair_case_close_tolerance;
	 this->_TP_detection_count_th= TP_detection_count_th;
	 
	 
	 
	 this->_update_rate = update_rate;
	 
	 
	 /* pre searching parameter */
     bool TP_presearch_enable_flag = false;
	 float TP_presearch_duration_for_interval =1; /*unit : sec*/
	 int TP_presearch_interval_num=6;
	 float TP_presearch_ang_vel = 0.05;
	 
	 param_nh.getParam("TP_presearch_enable_flag",TP_presearch_enable_flag);
	 param_nh.getParam("TP_presearch_duration_for_interval",TP_presearch_duration_for_interval);
	 param_nh.getParam("TP_presearch_interval_num",TP_presearch_interval_num);
	 param_nh.getParam("TP_presearch_ang_vel",TP_presearch_ang_vel);
	 
	 this->_TP_presearch_enable_flag=TP_presearch_enable_flag;
	 this->_TP_presearch_duration_for_interval = TP_presearch_duration_for_interval;
	 this->_TP_presearch_interval_num = TP_presearch_interval_num;
	 this->_TP_presearch_ang_vel = TP_presearch_ang_vel;
	 
	 
	 this->_stair_center_pixel_sub = main_nh.subscribe<stair_detection::pixel_stairlocation>(stair_center_pixel_sub_topic, 1, boost::bind(&PIXEL_CONTROL_TO_STAIR_ROS::executeCB, this,_1,0));
	 this->_detec_trigger_sub = main_nh.subscribe<std_msgs::Int8>(detect_trigger_flag_sub_topic, 1, boost::bind(&PIXEL_CONTROL_TO_STAIR_ROS::executeCB_detect_trigger_flag, this,_1,0));  
	 this->_cmd_vel_pub = main_nh.advertise<geometry_msgs::Twist>(control_to_stair_cmd_vel_topic, 2);
	 this->_flag_pub = main_nh.advertise<std_msgs::Int8>(control_to_stair_flag_topic,2);
	 
	 this->_loop_rate = new ros::Rate(this->_update_rate);
	 
	 this->_update_time =  ros::Time::now();
}

PIXEL_CONTROL_TO_STAIR_ROS::~PIXEL_CONTROL_TO_STAIR_ROS()
{
	delete this->_loop_rate;

}

void PIXEL_CONTROL_TO_STAIR_ROS::executeCB_detect_trigger_flag(const std_msgs::Int8::ConstPtr& msg,int value)
{
	if(msg->data == 1)
	{
		this->_detec_trigger_flag = 1;
	}
	else if(msg->data == 2)
	{
		this->_detec_trigger_flag = 2;
	}
	else
	{
		this->_detec_trigger_flag = 0;
		
		this->presearching_reset();
		this->control_reset();
	}
	
}



void PIXEL_CONTROL_TO_STAIR_ROS::executeCB(const stair_detection::pixel_stairlocationConstPtr &stair_pixel_pose_vec, int value)
{
	 //ROS_INFO("debug_line2");
	
	bool data_fail = false;
	
	if((stair_pixel_pose_vec->row_center.empty())||(stair_pixel_pose_vec->column_center.empty())||(stair_pixel_pose_vec->depth_vec.empty()))
	{
		data_fail = true;
	}
	else if((stair_pixel_pose_vec->row_center.size() != stair_pixel_pose_vec->column_center.size())||(stair_pixel_pose_vec->row_center.size()!=stair_pixel_pose_vec->depth_vec.size()))
	{
		data_fail = true;
	}
	
	//ROS_INFO("data_fail : %d", data_fail);
	
	int i;
	int end_i = stair_pixel_pose_vec->row_center.size();
	
	//ROS_INFO("end_i : %d", end_i);
	
	for(i = 0; i<end_i; i++)
	{  
       
	   if( this->_filtered_row_center == 0)
	   {
	        this->_filtered_row_center =  stair_pixel_pose_vec->row_center.at(i);
	    }
		else
		{
			this->_filtered_row_center =  stair_pixel_pose_vec->row_center.at(i)*this->_TP_weight_of_lpf + this->_filtered_row_center_old*(1-this->_TP_weight_of_lpf );
		}
		this->_filtered_row_center_old = this->_filtered_row_center;
		
		float invalid_depth_rejected;
		
		if(stair_pixel_pose_vec->depth_vec.at(i) <0.1)
		{
			invalid_depth_rejected = this->_TP_stair_case_close_tolerance +0.5;
		}
		else if(stair_pixel_pose_vec->depth_vec.at(i) > 10)
		{
			invalid_depth_rejected = 10;
		}
		else
		{
			invalid_depth_rejected = stair_pixel_pose_vec->depth_vec.at(i); 
		}
		
	    if( this->_filtered_depth_data == 0)
	   {
	        this->_filtered_depth_data =  invalid_depth_rejected;
	    }
		else
		{
			this->_filtered_depth_data =  invalid_depth_rejected*this->_TP_weight_of_lpf + this->_filtered_depth_data_old*(1-this->_TP_weight_of_lpf );
		}
		this->_filtered_depth_data_old = this->_filtered_depth_data;
		
		
		if(this->_control_state_num!=2)
		{
		   this->_detection_count = this->_detection_count + 1;
		   
		   if(this->_detection_count>(2 + this->_TP_detection_count_th))
		   {
			   this->_detection_count = 2 + this->_TP_detection_count_th;
		   }
		}		   
		
	}
	
	 ROS_INFO("end_i : %d", end_i);
	 ROS_INFO("_detection_count : %d",  this->_detection_count);
	 //ROS_INFO("_filtered_row_center : %f", this->_filtered_row_center);
	 //ROS_INFO("_filtered_depth_data : %f", this->_filtered_depth_data);
	
	 this->_update_time = ros::Time::now();
	 this->_vel_reset_flag = false;
}

void PIXEL_CONTROL_TO_STAIR_ROS::presearching()
{
	
	/*get feedback from move base_ stair_case detetion bridge */ 
	std_msgs::Int8 stair_to_control_flag_temp;
	geometry_msgs::Twist cmd_vel_temp;
	
     cmd_vel_temp.linear.x =0.0;
     cmd_vel_temp.linear.y = 0.0;
	 
	
	if(this->_control_state_num == 0)
	{
		this->_presearch_init_time = ros::Time::now();
	}
	
	ros::Duration elapse_time = ros::Time::now() - this->_presearch_init_time;
	
	float initalized_total_rot_time = (this->_TP_presearch_duration_for_interval*this->_TP_presearch_interval_num)/2;
	
	if((elapse_time<ros::Duration(initalized_total_rot_time))&&(this->_presearch_init_comp_flag == false))
	{  
         this->_control_state_num = 1;
		 stair_to_control_flag_temp.data = this->_control_state_num;
		 cmd_vel_temp.angular.z = -this->_TP_presearch_ang_vel;   /* minus sign is important*/
		 
		 this->_flag_pub.publish(stair_to_control_flag_temp);
	     this->_cmd_vel_pub.publish(cmd_vel_temp);
	}
	else
	{
		this->_presearch_init_comp_flag = true;
	}
	
	ros::Duration _presearch_pattern_elapse_time ;
	
	if(this->_presearch_init_comp_flag == true)
	{
		if((this->_presearch_pause_flag == false)&&(this->_presearch_final_comp_flag == false))
		{
		   if(this->_presearch_rotat_flag == false)
		   {
			   this->_presearch_pattern_elapse_time = ros::Time::now();
			   this->_presearch_rotat_flag = true;
			   this->_presearch_interval_stairdetec_count_start = this->_detection_count;
			
			    this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
			
			    cmd_vel_temp.angular.z = this->_TP_presearch_ang_vel;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);
		   }
		   else if((ros::Time::now()-this->_presearch_pattern_elapse_time)<ros::Duration(this->_TP_presearch_duration_for_interval))
		  {
			   this->_control_state_num = 1;
		       stair_to_control_flag_temp.data = this->_control_state_num;
			
			   cmd_vel_temp.angular.z = this->_TP_presearch_ang_vel;
			
			   this->_flag_pub.publish(stair_to_control_flag_temp);
	           this->_cmd_vel_pub.publish(cmd_vel_temp);
		   }
		   else
		   {
			   //this->_presearch_pattern_elapse_time = ros::Time::now();
			   this->_presearch_rotat_flag = false;
			   //this->_presearch_pause_flag = true;
		   }
	    }
		
		if((this->_presearch_rotat_flag == false)&&(this->_presearch_final_comp_flag == false))
		{
			if(this->_presearch_pause_flag == false)
			{
				this->_presearch_pattern_elapse_time = ros::Time::now();
				this->_presearch_pause_flag = true;
				
				this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
			
			    cmd_vel_temp.angular.z = 0;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);
			}
			else if((ros::Time::now()-this->_presearch_pattern_elapse_time)<ros::Duration(2*this->_TP_presearch_duration_for_interval))
			{
				this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
			
			    cmd_vel_temp.angular.z = 0;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);
			}
			else
			{
				this->_presearch_pause_flag = false;
				this->_presearch_interval_stairdetec_count_end = this->_detection_count;
			}
		}
		
		if((this->_presearch_pause_flag == false)&&(this->_presearch_rotat_flag == false)&&(this->_presearch_final_comp_flag == false))
		{
			int total_detec_num = this->_presearch_interval_stairdetec_count_end - this->_presearch_interval_stairdetec_count_start;
			
			_total_stairdetec_num_array[this->_presearch_interval_count] = total_detec_num;
		
			if(_total_stairdetec_num_array[this->_presearch_interval_count]>this->_stairdetec_max_count)
			{
				this->_stairdetec_max_count = _total_stairdetec_num_array[this->_presearch_interval_count];
				this->_stairdetec_max_index = this->_presearch_interval_count;
			}
			this->_presearch_interval_stairdetec_count_end = 0;
			this->_presearch_interval_stairdetec_count_start  = 0;
			
			this->_presearch_interval_count = this->_presearch_interval_count +1;
		}
		
		if(this->_presearch_interval_count==this->_TP_presearch_interval_num)
		{   
	        
			float duration_multiplier = this->_TP_presearch_interval_num-this->_stairdetec_max_index;
			
			if(_presearch_final_comp_flag == false)
			{
				this->_presearch_pattern_elapse_time = ros::Time::now();
				this->_presearch_final_comp_flag = true;
				this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
			
			    cmd_vel_temp.angular.z = -this->_TP_presearch_ang_vel;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);

			}
			else if((ros::Time::now()-this->_presearch_pattern_elapse_time)<ros::Duration(duration_multiplier*this->_TP_presearch_duration_for_interval))
			{
				this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
			
			    cmd_vel_temp.angular.z = -this->_TP_presearch_ang_vel;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);
			}
			else
			{ 
		        this->_control_state_num = 1;
		        stair_to_control_flag_temp.data = this->_control_state_num;
				cmd_vel_temp.angular.z = 0;
			
			    this->_flag_pub.publish(stair_to_control_flag_temp);
	            this->_cmd_vel_pub.publish(cmd_vel_temp);
				
				this->presearching_reset();
				
				this->_presearch_ok_flag = true;
				
			}
		}
		
		
	}
	
	ROS_INFO("_presearch_init_comp_flag : %d", this->_presearch_init_comp_flag);
	ROS_INFO("_presearch_final_comp_flag : %d", this->_presearch_final_comp_flag);
	ROS_INFO("_presearch_interval_count : %d", this->_presearch_interval_count);
	ROS_INFO("_stairdetec_max_count : %d", this->_stairdetec_max_count);
	ROS_INFO("_stairdetec_max_index : %d", this->_stairdetec_max_index);
	
}

void PIXEL_CONTROL_TO_STAIR_ROS::presearching_reset()
{
	this->_presearch_rotat_flag = false;
	this->_presearch_pause_flag = false;
	
	this->_presearch_interval_stairdetec_count_end = 0;
	this->_presearch_interval_stairdetec_count_start  = 0;
	
	this->_presearch_final_comp_flag = false;
	this->_presearch_init_comp_flag = false;
	this->_presearch_interval_count = 0;
	this->_stairdetec_max_count = 0;
	this->_stairdetec_max_index = 0;
}

void PIXEL_CONTROL_TO_STAIR_ROS::control_reset()
{
	
	std_msgs::Int8 stair_to_control_flag_temp;
	
	this->presearching_reset();
	
	this->_control_state_num = 0;
	stair_to_control_flag_temp.data = this->_control_state_num;
	this->_flag_pub.publish(stair_to_control_flag_temp);
	
	
	this->_presearch_ok_flag = false;
	this->_vel_reset_flag = false;
	this->_cmd_vel_ok_flag = false;
	this->_control_num_2_delay_flag  = false;
	
	this->_detection_count = 0;
	
	this->_filtered_depth_data = 0;
	this->_filtered_depth_data_old = 0;
	this->_filtered_row_center = 0;
	this->_filtered_row_center_old = 0;
	this->_rotation_command_working_flag = false;
}


void PIXEL_CONTROL_TO_STAIR_ROS::loop()
{   
   	ros::Duration elapse_time = ros::Time::now() - this->_update_time;
	geometry_msgs::Twist cmd_vel_temp;
	
	std_msgs::Int8 stair_to_control_flag_temp;
	
	float angular_vel_temp = 0;
	
	
	if((elapse_time>ros::Duration(this->_TP_duration_for_reset_target))&&(this->_control_state_num!=2))
	{
		this->_filtered_depth_data = 0;
		this->_filtered_depth_data_old = 0;
		this->_filtered_row_center = 0;
		this->_filtered_row_center_old = 0;
		this->_rotation_command_working_flag = false;
		
		
		if(this->_vel_reset_flag == false)
		 {
			 cmd_vel_temp.linear.x = 0;
			 cmd_vel_temp.linear.y = 0;
			 cmd_vel_temp.angular.z = 0;
			
			 this->_cmd_vel_pub.publish(cmd_vel_temp);
			
			 this->_control_state_num = 1;
			
			 this->_vel_reset_flag = true;
		 }
		 else
		 {
			 this->_control_state_num = 0;
		 }

		
		this->_detection_count = this->_detection_count - 1;

		if(this->_detection_count<=0)
		{  
		    this->_detection_count = 0;
			this->_cmd_vel_ok_flag = false;
		}		
	}
	else
	{
		
	}
	
	if(this->_detection_count>this->_TP_detection_count_th)
	{ 
		this->_cmd_vel_ok_flag = true;
	}


	if((this->_filtered_row_center>20)&&(this->_control_state_num!=2))
	{
		
		
		cmd_vel_temp.linear.x = this->_TP_linear_vel;
        cmd_vel_temp.linear.y = 0.0;
		
		float row_center_error = this->_TP_target_row_center-this->_filtered_row_center;
	     
		 if(elapse_time<ros::Duration(1))   /* when loosing target while rotating, stop the rotating*/
		 {
		   if((std::abs(row_center_error)>this->_TP_rotation_start_threshold)&&(this->_rotation_command_working_flag == false))
		   {
		         cmd_vel_temp.angular.z = row_center_error*this->_TP_angular_vel_p_gain;	
			     this->_rotation_command_working_flag = true;
		   }
           else if((this->_rotation_command_working_flag == true)&&(std::abs(row_center_error)>this->_TP_rotation_end_threshold))
		   {
			     cmd_vel_temp.angular.z = row_center_error*this->_TP_angular_vel_p_gain;	
			     this->_rotation_command_working_flag = true;
		   }
		   else
		   {
			     cmd_vel_temp.angular.z = 0;	
			     this->_rotation_command_working_flag = false;
		   }
	     }
		 else
		 {
			 cmd_vel_temp.angular.z = 0;
		     this->_rotation_command_working_flag = false;
		 }
		 
		 
		if(this->_filtered_depth_data>this->_TP_stair_case_close_tolerance)
		{  
	        if(this->_cmd_vel_ok_flag == true)
			{
			   this->_cmd_vel_pub.publish(cmd_vel_temp);
			   this->_control_state_num = 1;
			}
		}
		else
		{
			if(this->_control_num_2_delay_flag == false)
			{
				
				//ROS_INFO("this->_control_num_2_delay_flag %d",this->_control_num_2_delay_flag);
				
				this->_control_num_2_delay_flag = true;
				
				//ROS_INFO("this->_control_num_2_delay_flag %d",this->_control_num_2_delay_flag);
			}
			else
			{
			    this->_control_state_num = 2;	
				
				ROS_INFO("debug_control_state_num");
			}
			
			this->_detection_count = 0;
			
			cmd_vel_temp.linear.x = 0;
			cmd_vel_temp.angular.z = 0;
			this->_cmd_vel_pub.publish(cmd_vel_temp);
			this->_cmd_vel_ok_flag = false;
			/* need to publish stair climbing trigger*/
		}
		
	}
	
	 ROS_INFO("linear_x %f",cmd_vel_temp.linear.x);
	 ROS_INFO("_vel_reset_flag %d",this->_vel_reset_flag);
	 ROS_INFO("_cmd_vel_ok_flag %d",this->_cmd_vel_ok_flag);
	
	 stair_to_control_flag_temp.data = this->_control_state_num;
	
	 this->_flag_pub.publish(stair_to_control_flag_temp);

}


void PIXEL_CONTROL_TO_STAIR_ROS::run()
{  
    while(ros::ok())
	{ 
       if(this->_detec_trigger_flag == 1)
	   {
	      if(this->_TP_presearch_enable_flag == true)
	      {
	   	      if(this->_presearch_ok_flag==false) this->presearching();
	      }
	      else
	      {
		      this->_presearch_ok_flag = true;
	      }
	   
	      if(this->_presearch_ok_flag == true)
	      { 
             this -> loop();
	      }
	   }
	   else if(this->_detec_trigger_flag == 2)
	   {
		     std_msgs::Int8 stair_to_control_flag_temp;
			 
			 this->_control_state_num = 2;	
			 
			 this->_flag_pub.publish(stair_to_control_flag_temp);
			 
	   }
	   
	   ros::spinOnce();
	   this->_loop_rate->sleep();
	}
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_to_stair");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  
  PIXEL_CONTROL_TO_STAIR_ROS  pixel_control_to_stair_ros(nh,_nh);
  
  pixel_control_to_stair_ros.run();
 
  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("front_cam/camera/depth/image_rect_raw", 1, imageCallback);
  //ros::spin();
   return 0;
}
