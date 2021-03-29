#ifndef STAIR_CONTAINER_H_
#define STAIR_CONTAINER_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include <map>
#include <utility>  //pair 
#include <cmath>        // std::abs

#define PROBABILITY_MAX 100


class STAIR_CONTAINER{
	
	private:
	
	float _probability;
	int _count;
	float _stair_container_x_coor;
	float _stair_container_y_coor;
	float _stair_container_z_coor;
    float _stair_container_x_coor_old;
	float _stair_container_y_coor_old;
	float _stair_container_z_coor_old;
	//bool _forget_start_flag;
    bool _container_empty_flag;
    std::string _probability_type;	
	
	float _prob_factor_geo_series;
	float _forget_factor_geo_series;
	int  _prob_factor_linear;
	int _forget_factor_linear;
	
	float _pose_update_weight;
	float _pose_update_weight_min = 0.7;
	float _pose_update_weight_max = 0.95;
	
	
	public: 
	
	void stair_container_clear();
	bool stair_container_probforget_factor_update_geo_series(float prob_factor, float forget_factor);  /*return type is about checking the probability type*/
	bool stair_container_probforget_factor_update_linear(int prob_factor, int forget_factor);  /*return type is about checking the probability type*/
	void stair_container_pos_return(float *x, float *y, float *z);
	float stair_container_pob_return();
	bool stair_container_empty_flag_return();
	void stair_container_prob_pos_update(float x, float y, float z, bool prob_count_update = true);
	void stair_container_prob_update();
	void stair_container_prob_forget();
	
	
	/*constructor and destructor*/
	STAIR_CONTAINER(std::string probability_type);
	~STAIR_CONTAINER();
	

};


#endif
