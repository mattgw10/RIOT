/* Vehicle header file
	contains dynamics and state/edge information
	for various vehicles*/

#include"grid_decomposition.h"
#include<ANN/ANN.h>
#include<random>
#pragma once

using namespace space;
using namespace abstraction;

namespace robot{
	class vehicles {
	public:
		virtual double state_distance(int state_ind){}
		virtual double path_est(int state_ind){}
		virtual double forward_simulate(int state_ind, std::vector<bool> blocked){}
		virtual bool check_solution(int sol_ind, std::vector<bool> blocked){}
		virtual void random_control(){}
		virtual int region_of_state(int state_ind){}
		virtual void state_in_region(int region){}
		virtual int nearest_neighbor(){}
		virtual void push_state(int expanded){}
		virtual void save_best(){}
		virtual double g_val(int state_ind){}
		virtual double sample_f_val(){}
		virtual double sample_g_val(){}
		virtual int sample_region(){}
		virtual int parent(int state_ind){}
		virtual double goal_eval(int state_ind){}
		virtual double state_radius(int a, int b){}
		virtual void print_state(int state_ind, ofstream& file){}
		virtual void print_solution(int sol_ind, ofstream& file){}
		virtual void push_point(ANNpointArray points, int expanded){}
		virtual void assign_query(ANNpoint queryPt, int state_ind){}
	};
}