#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Int8.h>

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <mutex>
#include <boost/bind.hpp>

#include <stair_detection/stairlocation.h>
#include "control_to_stair/stair_container.h"


#include <cmath>   /*sqrt function */



#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

std::mutex _lock;


float Dist_sq(float x1,float y1,float x2, float y2)
{
	float square_result = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
	
	return square_result;
}


float Dist(float x1,float y1,float x2, float y2)
{
	float square_temp = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
	          
    float dist = sqrt(square_temp);
			  
	return dist;
}



class CONTROL_TO_STAIR_ROS{
	
	private:
		 
		 ros::NodeHandle main_nh;
         ros::NodeHandle param_nh;
	     ros::Rate* _loop_rate;

         ros::Subscriber _stair_pos_sub;
		 ros::Subscriber _odom_sub;
		 ros::Subscriber _detec_trigger_sub;
		 
		 ros::Publisher _cmd_vel_pub;
         ros::Publisher _robot_cancel_move_base_pub;
		 ros::Publisher _flag_pub;
		 
		 tf2_ros::Buffer _tfBuffer;
		 tf2_ros::TransformListener _tfListener;
		 //tf2_ros::MessageFilter<geometry_msgs::PointStamped> _tf2_filter;
		 
		 actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _moveBaseActionClient;
		 
		 std::string _world_frame;
		 int _update_rate;
		 
		 float _TP_probability_update_radius_th;
		 float _TP_stair_case_close_tolerance;
		 float _TP_probability_control_th;
		 float _TP_duration_for_forget_target;
		 float _TP_probability_clip_th;
		 float _TP_directctrl_dist_th;
		 
		 float _target_stair_pos_x = 0;
		 float _target_stair_pos_y  = 0;
		 float _target_stair_pos_theta = 0;
		 
		 float _robot_cur_pos_x = 0;
		 float _robot_cur_pos_y = 0;
		 float _robot_cur_pos_theta = 0;
		 
		 int _control_target_index = 0;
		 
		 int _detec_trigger_flag = 0;
		 
		 unsigned char _control_state_num=0; /* 0 : initial 1: working 2: completed*/
		 unsigned char _control_state_num_old=0;
		 
		 bool _actionServerSuccess = true;
		 bool _moveBaseCommandedOnce = false;
	
	public:
	
	    void run();
		void loop();
		void control();
		void presearching_reset();
		void control_reset();
		void executeCB(const stair_detection::stairlocationConstPtr &stair_pose_vec, int value);
		void odomCB(const nav_msgs::OdometryConstPtr &odom_msg, int value);
		
		void executeCB_detect_trigger_flag(const std_msgs::Int8::ConstPtr& msg,int value);
	 
		/*constructor and destructor*/
	    CONTROL_TO_STAIR_ROS(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~CONTROL_TO_STAIR_ROS();
		
		STAIR_CONTAINER* stair_container_first;
		STAIR_CONTAINER* stair_container_second;
		
		ros::Time _update_time_first;
		ros::Time _update_time_second;
		

};

CONTROL_TO_STAIR_ROS::CONTROL_TO_STAIR_ROS(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh(m_nh),param_nh(p_nh),_tfListener(_tfBuffer),_moveBaseActionClient ("move_base", true)
{
	 int update_rate = 10;
	
	 std::string  stair_pos_sub_topic = "front_cam/camera/stair_pose";
	 std::string  control_to_stair_cmd_vel_topic = "stair_nav/cmd_vel_stair";
	 std::string  control_to_stair_flag_topic = "stair_nav/cmd_vel_stair_flag";
	 std::string  world_frame = "map";
	 std::string  odom_sub_topic ="odom";
	 std::string detect_trigger_flag_sub_topic ="detec_logic_trigger";
	 
	 std::string probability_type ="linear";  /*linear and geo_series*/
	 
	 float TP_probability_update_radius_th = 2; /*unit : m*/
	 float TP_stair_case_close_tolerance = 1.5; /*unit : m*/
	 float TP_probability_clip_th = 70; /* unit : %*/
	 float TP_probability_control_th = 50; /* unit : %*/
	 float TP_duration_for_forget_target = 2; /*unit : sec */
	 float TP_directctrl_dist_th = 4.5; /*unit : m */
	 
	
	 param_nh.getParam("stair_pos_sub_topic",stair_pos_sub_topic);
	 param_nh.getParam("control_to_stair_cmd_vel_topic",control_to_stair_cmd_vel_topic);
	 param_nh.getParam("control_to_stair_flag_topic",control_to_stair_flag_topic);
     
	 param_nh.getParam("odom_sub_topic",odom_sub_topic);
	 param_nh.getParam("detect_trigger_flag_sub_topic",detect_trigger_flag_sub_topic);
	 
	 param_nh.getParam("world_frame",world_frame);
	 
	 param_nh.getParam("TP_probability_update_radius_th",TP_probability_update_radius_th);
	 param_nh.getParam("TP_stair_case_close_tolerance",TP_stair_case_close_tolerance);
	 param_nh.getParam("TP_probability_control_th",TP_probability_control_th);
	 param_nh.getParam("TP_probability_clip_th",TP_probability_clip_th);
	 param_nh.getParam("TP_duration_for_forget_target",TP_duration_for_forget_target);
	 param_nh.getParam("TP_directctrl_dist_th",TP_directctrl_dist_th);
	 
	 param_nh.getParam("probability_type",probability_type);
	 
	 param_nh.getParam("update_rate",update_rate);
	   
	 this->_update_rate = update_rate;
	 this->_world_frame = world_frame;
	 
	 this->_TP_probability_update_radius_th = TP_probability_update_radius_th;
	 this->_TP_stair_case_close_tolerance = TP_stair_case_close_tolerance;
	 this->_TP_probability_control_th = TP_probability_control_th;
	 this->_TP_probability_clip_th = TP_probability_clip_th;
	 this->_TP_duration_for_forget_target = TP_duration_for_forget_target;
	 this->_TP_directctrl_dist_th = TP_directctrl_dist_th;
	 
	 this->_stair_pos_sub = main_nh.subscribe<stair_detection::stairlocation>(stair_pos_sub_topic, 1, boost::bind(&CONTROL_TO_STAIR_ROS::executeCB, this,_1,0));
	 this->_odom_sub = main_nh.subscribe<nav_msgs::Odometry>(odom_sub_topic, 1, boost::bind(&CONTROL_TO_STAIR_ROS::odomCB, this,_1,0));
	 this->_detec_trigger_sub = main_nh.subscribe<std_msgs::Int8>(detect_trigger_flag_sub_topic, 1, boost::bind(&CONTROL_TO_STAIR_ROS::executeCB_detect_trigger_flag, this,_1,0));  
	 this->_flag_pub = main_nh.advertise<std_msgs::Int8>(control_to_stair_flag_topic,2);
	 
	 this->_cmd_vel_pub = main_nh.advertise<geometry_msgs::Twist>(control_to_stair_cmd_vel_topic, 2);
	 	 
	 this->_robot_cancel_move_base_pub = main_nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);    
	 
	 
	 this->_loop_rate = new ros::Rate(this->_update_rate);
	 
     this->stair_container_first = new STAIR_CONTAINER(probability_type);   /*linear, geo_series*/
	 this->stair_container_second = new STAIR_CONTAINER(probability_type);   /*linear, geo_series*/
	 
	 
	 int prob_factor_linear_temp = 5;
	 int forget_factor_linear_temp = 2;
	 
	 this->stair_container_first->stair_container_probforget_factor_update_linear(prob_factor_linear_temp,forget_factor_linear_temp);
	 this->stair_container_second->stair_container_probforget_factor_update_linear(prob_factor_linear_temp,forget_factor_linear_temp);
	 

}

CONTROL_TO_STAIR_ROS::~CONTROL_TO_STAIR_ROS()
{
	delete this->_loop_rate;
	delete  this->stair_container_first;
	delete  this->stair_container_second;
}

void CONTROL_TO_STAIR_ROS::presearching_reset()
{
	
}

void CONTROL_TO_STAIR_ROS::control_reset()
{
	
}



void CONTROL_TO_STAIR_ROS::odomCB(const nav_msgs::OdometryConstPtr &odom_msg, int value)
{
	
	/*reference : https://answers.ros.org/question/58742/quaternion-to-roll-pitch-yaw/*/
	//btQuaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
	
	//float roll;
    //float pictch;
    //float yaw;
	
	//tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	tf::Pose pose;
    tf::poseMsgToTF(odom_msg->pose.pose, pose);
    
	double yaw_angle = tf::getYaw(pose.getRotation());
	
	
	
	this->_robot_cur_pos_theta = yaw_angle*57.3/*rad to deg*/;
	this ->_robot_cur_pos_x = odom_msg->pose.pose.position.x;
	this ->_robot_cur_pos_y = odom_msg->pose.pose.position.y;
	
	//ROS_INFO("_robot_cur_pos_theta:%f \n", this->_robot_cur_pos_theta);
	//ROS_INFO("_robot_cur_pos_x:%f \n", this ->_robot_cur_pos_x);
	//ROS_INFO("_robot_cur_pos_y:%f \n", this ->_robot_cur_pos_y);
	
}


void CONTROL_TO_STAIR_ROS::executeCB_detect_trigger_flag(const std_msgs::Int8::ConstPtr& msg,int value)
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


void CONTROL_TO_STAIR_ROS::executeCB(const stair_detection::stairlocationConstPtr &stair_pose_vec, int value)
{
	 //ROS_INFO("debug_line2");
	 
	 float distance_sq;
	 
	std::vector<float>::iterator it; 
	
	geometry_msgs::PointStamped point_out_single;
	geometry_msgs::PointStamped point_in_single;
	
	bool data_fail = false;
	bool data_tf_fail = false;
	
	if((stair_pose_vec->x.empty())||(stair_pose_vec->y.empty())||(stair_pose_vec->z.empty()))
	{
		data_fail = true;
	}
	else if((stair_pose_vec->x.size() != stair_pose_vec->y.size())||(stair_pose_vec->x.size()!=stair_pose_vec->z.size()))
	{
		data_fail = true;
	}
	
	//ROS_INFO("data_fail : %d", data_fail);
	
	int i;
	int end_i = stair_pose_vec->x.size();
	
	//ROS_INFO("end_i : %d", end_i);
	
	float invalid_depth_rejected;
	
	for(i = 0; i<end_i; i++)
	{  
	    point_in_single.header.stamp =  stair_pose_vec->header.stamp;
        point_in_single.header.frame_id = stair_pose_vec->header.frame_id;
		point_in_single.point.x = stair_pose_vec->x.at(i);
		point_in_single.point.y = stair_pose_vec->y.at(i);
		
		if(stair_pose_vec->z.at(i) < 0.1)
		{
			invalid_depth_rejected = this->_TP_stair_case_close_tolerance +0.5;
		}
		else if(stair_pose_vec->z.at(i) > 10)
		{
			invalid_depth_rejected = 10;
		}
		else
		{
			invalid_depth_rejected = stair_pose_vec->z.at(i);
		}
		
		point_in_single.point.z = invalid_depth_rejected;
	    
		try
	   {
		    _tfBuffer.transform(point_in_single, point_out_single, this->_world_frame);  /* to use transform  #include "tf2_geometry_msgs/tf2_geometry_msgs.h" is needed*/
			
			// ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",
			//                       point_out_single.point.x,
			//						point_out_single.point.y,
			//						point_out_single.point.z);
									
			 if(this->stair_container_first->stair_container_empty_flag_return()==true)
	        {
		         this->stair_container_first->stair_container_prob_pos_update( point_out_single.point.x,point_out_single.point.y,point_out_single.point.z,true);
				 this->_update_time_first = ros::Time::now();
				 continue;
	         }
	         else
			 {
				 float x_temp;
				 float y_temp;
				 float z_temp;
				 
				 this->stair_container_first->stair_container_pos_return(&x_temp,&y_temp,&z_temp);
				 
				 distance_sq = Dist_sq(x_temp,y_temp,point_out_single.point.x,point_out_single.point.y);
				 
				 if(distance_sq<this->_TP_probability_update_radius_th^2)
				 {
					 this->stair_container_first->stair_container_prob_pos_update( point_out_single.point.x,point_out_single.point.y,point_out_single.point.z,true);
					 this->_update_time_first = ros::Time::now();
					 continue;
				 }
				 
			 }
			 
			 if(this->stair_container_second->stair_container_empty_flag_return()==true)
	        {
		         this->stair_container_second->stair_container_prob_pos_update( point_out_single.point.x,point_out_single.point.y,point_out_single.point.z,true);
				 this->_update_time_second = ros::Time::now();
				 continue;
	        }
             else
			 {   
		         float x_temp;
				 float y_temp;
				 float z_temp;
				 
				 this->stair_container_second->stair_container_pos_return(&x_temp,&y_temp,&z_temp);
				 
				 distance_sq = Dist_sq(x_temp,y_temp,point_out_single.point.x,point_out_single.point.y);
				 
				 if(distance_sq<this->_TP_probability_update_radius_th^2)
				 {
					 this->stair_container_second->stair_container_prob_pos_update( point_out_single.point.x,point_out_single.point.y,point_out_single.point.z,true);
					 this->_update_time_second = ros::Time::now();
					 continue;
				 }

			 }				 
			
	   }
	   catch (tf2::TransformException &ex) 
	   {
		   ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
		   data_tf_fail = true;
	   }
	   

	}
	
	 //bool cont_first_empty_flag_temp = this->stair_container_first->stair_container_empty_flag_return();
	 //bool cont_second_empty_flag_temp = this->stair_container_second->stair_container_empty_flag_return();
	 
	 //ROS_INFO("first container empty_flag:%d \n", cont_first_empty_flag_temp);
	 //ROS_INFO("second container empty_flag:%d \n", cont_second_empty_flag_temp);
	 
	 //float cont_first_prob_temp = this->stair_container_first->stair_container_pob_return();
	 //float cont_second_prob_temp = this->stair_container_second->stair_container_pob_return();
	 
	 //ROS_INFO("first container Prob:%f \n", cont_first_prob_temp);
	 //ROS_INFO("second container Prob:%f \n", cont_second_prob_temp);
	 
	 //float first_x_temp;
	 //float first_y_temp;
     //float  first_z_temp;
	 //float  second_x_temp;
     //float  second_y_temp;
     //float  second_z_temp;
	 
	 //this->stair_container_first->stair_container_pos_return(&first_x_temp,&first_y_temp,&first_z_temp);
	 //this->stair_container_second->stair_container_pos_return(&second_x_temp,&second_y_temp,&second_z_temp);
	 
	 //ROS_INFO("first container Position(x:%f y:%f z:%f)\n", first_x_temp,first_y_temp,first_z_temp);
	 //ROS_INFO("second container Position(x:%f y:%f z:%f)\n", second_x_temp,second_y_temp,second_z_temp);
	 
}

void CONTROL_TO_STAIR_ROS::control()
{
	 float TP_arrival_dist_th = 2;
	 float TP_directctrl_dist_th = 4;
	 float target_x = 0, target_y = 0, target_orientation = 0;
	 
	 geometry_msgs::Twist cmd_vel_temp;
	 std_msgs::Int8 stair_to_control_flag_temp;
	 
	 
	 // Create a simple action client to move_base
     // spin a thread by default
	
	// if ( !moveBaseActionClient.waitForServer(ros::Duration(5, 0)) )
    //{
     // ROS_ERROR("The move_base action server did not start.");
     // ROS_ERROR("Aborting.");
      //this->_actionServerSuccess = false;
    //}
	
	this->_actionServerSuccess = true;  /*test code*/
	
	actionlib::SimpleClientGoalState moveBaseGoalState(actionlib::SimpleClientGoalState::PENDING);
    move_base_msgs::MoveBaseGoal moveBaseGoal;
	
	
	moveBaseGoalState = this->_moveBaseActionClient.getState();
	
	
	if(this->_moveBaseCommandedOnce && moveBaseGoalState.isDone() && moveBaseGoalState != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		this->_moveBaseCommandedOnce = false;
	}
	
	if(this->_moveBaseCommandedOnce  && moveBaseGoalState == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		this->_moveBaseCommandedOnce = false;
	}
	
	
	float distance = Dist(this ->_robot_cur_pos_x,this ->_robot_cur_pos_y,this->_target_stair_pos_x,this->_target_stair_pos_y);
	
	ROS_INFO("distacne from robot to stair %f)\n", distance);
	
	if(distance < this->_TP_stair_case_close_tolerance)
	{
		/*end control */
		
		this->_control_state_num = 2;
		
		if((this->_control_state_num_old != 2)&&(this->_control_state_num == 2))
		{
			/*publish zero velocity*/
			cmd_vel_temp.linear.x = 0;
			cmd_vel_temp.linear.y = 0;
			cmd_vel_temp.angular.z = 0;
			
			this->_cmd_vel_pub.publish(cmd_vel_temp);
            /* send flag  1 */			
			
			stair_to_control_flag_temp.data = 1;
			this->_flag_pub.publish(stair_to_control_flag_temp);
			
		}
		else if((this->_control_state_num_old == 2)&&(this->_control_state_num == 2))
		{
			/* send flag 2*/
			
			stair_to_control_flag_temp.data = 2;
			this->_flag_pub.publish(stair_to_control_flag_temp);
			
		}
	}
	else if(((distance < TP_directctrl_dist_th/*temporal value*/)&&(this->_moveBaseCommandedOnce == false))||( this->_actionServerSuccess == false))
	{
		/*direct control */
		this->_control_state_num = 1;
		
		
		stair_to_control_flag_temp.data = 1;
		this->_flag_pub.publish(stair_to_control_flag_temp);
		
	}
    else if(this->_moveBaseCommandedOnce == false)
	{
		/*control via move base*/
		
		if(this->_actionServerSuccess == true)
		{
			
		   float weight =  (1 - TP_directctrl_dist_th/distance) + 0.1;
			
		   target_x = (this->_target_stair_pos_x - this->_robot_cur_pos_x)*weight +  this->_robot_cur_pos_x;
			
		   target_y = (this->_target_stair_pos_y - this->_robot_cur_pos_y)*weight +  this->_robot_cur_pos_y;
			
		   target_orientation =  this->_robot_cur_pos_theta;
			
			
			/* convert euler  to quaternion */
		   tf::Vector3 axis(0, 0, 1);
		   
		   float target_orientationRad = target_orientation/57.3;
		   tf::Quaternion quatTF(axis, target_orientationRad);
		   /* convert euler  to quaternion  end*/
		   
		   /* packing quaternion to msg */
		    geometry_msgs::Quaternion quatOrienationRobot;
		    tf::quaternionTFToMsg(quatTF, quatOrienationRobot);
			/* packing quaternion to msg  end*/
	
			
		   moveBaseGoal.target_pose.header.frame_id = "map";
           moveBaseGoal.target_pose.header.stamp = ros::Time::now();
           moveBaseGoal.target_pose.pose.position.x = target_x;
           moveBaseGoal.target_pose.pose.position.y = target_y;
           moveBaseGoal.target_pose.pose.position.z = 0;
           moveBaseGoal.target_pose.pose.orientation = quatOrienationRobot;
           this->_moveBaseActionClient.sendGoal(moveBaseGoal);
		
		   this->_control_state_num = 0;
		   
		   
		   stair_to_control_flag_temp.data = 0;
		   this->_flag_pub.publish(stair_to_control_flag_temp);
		   
		   this->_moveBaseCommandedOnce = true;
		   
		   
		   
		   ROS_INFO("target_x : %f \n", target_x);
		   ROS_INFO("target_y : %f \n", target_y);
		   ROS_INFO("target_orientation : %f \n", target_orientation);
		   
		}
		
		
	}
	
	ROS_INFO("_moveBaseCommandedOnce : %d \n", this->_moveBaseCommandedOnce);
	
}




void CONTROL_TO_STAIR_ROS::loop()
{
	ros::Duration elapse_time_first= ros::Time::now() - this->_update_time_first;
	ros::Duration elapse_time_second= ros::Time::now() - this->_update_time_second;
	
	if(this->stair_container_first->stair_container_empty_flag_return() == true)
	{
		if((elapse_time_first>ros::Duration(this->_TP_duration_for_forget_target))&&(this->stair_container_first->stair_container_pob_return()<this->_TP_probability_clip_th))
		{
			this->stair_container_first->stair_container_prob_forget();
		}
	}
		
	if(this->stair_container_second->stair_container_empty_flag_return() == true)
	{
		if((elapse_time_second>ros::Duration(this->_TP_duration_for_forget_target))&&(this->stair_container_second->stair_container_pob_return()<this->_TP_probability_clip_th))
		{
			this->stair_container_second->stair_container_prob_forget();
		}
	}
	
	if(this->_control_target_index != 0)
	{
		if(this->_control_target_index == 1)
		{
			if(this->stair_container_first->stair_container_pob_return()<this->_TP_probability_control_th)
			{
				this->_control_target_index  = 0;
				this->_target_stair_pos_x = 0;
				this->_target_stair_pos_y = 0;
				this->_target_stair_pos_theta = 0;
			}
		}
	    else if(this->_control_target_index == 2)
		{ 
	        if(this->stair_container_second->stair_container_pob_return()<this->_TP_probability_control_th)
			{
				this->_control_target_index  = 0;
				this->_target_stair_pos_x = 0;
				this->_target_stair_pos_y = 0;
				this->_target_stair_pos_theta = 0;
			}
		}
	}
	else
	{
		 if(this->stair_container_first->stair_container_pob_return()>=this->_TP_probability_control_th)
		 {
			 float x_temp;
			 float y_temp;
			 float z_temp;
			 
			 this->_control_target_index = 1;
			 
			 this->stair_container_first->stair_container_pos_return(&x_temp,&y_temp,&z_temp);
			  
			 this->_target_stair_pos_x = x_temp;
			 this->_target_stair_pos_y = y_temp;			 
		 }
		 else if(this->stair_container_second->stair_container_pob_return()>=this->_TP_probability_control_th)
		 {
			 float x_temp;
			 float y_temp;
			 float z_temp;
			 
			 this->_control_target_index = 2;
			 
			 this->stair_container_first->stair_container_pos_return(&x_temp,&y_temp,&z_temp);
			  
			 this->_target_stair_pos_x = x_temp;
			 this->_target_stair_pos_y = y_temp;
		 }
	}
	
	if(this->_control_target_index != 0)
	{
		this->control();
	}
	
	// ROS_INFO("_control_target_index:%d \n", this->_control_target_index);
	// ROS_INFO("_target_stair_pos_x:%f \n", this->_target_stair_pos_x);
	// ROS_INFO("_target_stair_pos_y:%f \n",  this->_target_stair_pos_y);
	// ROS_INFO("first_stair_probabilty:%f \n", this->stair_container_first->stair_container_pob_return());
	// ROS_INFO("second_stair_probabilty:%f \n", this->stair_container_second->stair_container_pob_return());
	
	this->_control_state_num_old = this->_control_state_num;
	
}


void CONTROL_TO_STAIR_ROS::run()
{  
    while(ros::ok())
	{ 

      if(this->_detec_trigger_flag == 1)
	  {		  
          this -> loop();
	  }
	  else if(this->_detec_trigger_flag == 2)
	  {
		   std_msgs::Int8 stair_to_control_flag_temp;
		   stair_to_control_flag_temp.data = 2;
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
  
  CONTROL_TO_STAIR_ROS  control_to_stair_ros(nh,_nh);
  
  control_to_stair_ros.run();
 
  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("front_cam/camera/depth/image_rect_raw", 1, imageCallback);
  //ros::spin();
   return 0;
}
