/* Defines vehicle- dynamic car*/

/** \brief A class to facilitate planning for a generic second-order
    car model

    The dynamics of the second-order car are described by the following
    equations:
    \f{eqnarray*}{
    \dot x &=& v\cos\theta,\\
    \dot y &=& v\sin\theta,\\
    \dot\theta &=& \frac{vm}{L}\tan \phi,\\
    \dot v &=& u_0,\\
    \dot\phi &=& u_1,\f}
    where \f$v\f$ is the speed, \f$\phi\f$ the steering angle, the
    controls \f$(u_0,u_1)\f$ control their rate of change, \f$m\f$ is
    the mass of the car, and \f$L\f$ is the distance between the front
    and rear axle of the car. Both \f$m\f$ and \f$L\f$ are set to 1 by
    default.
*/

#include"vehicles.h"
#pragma once

using namespace robot;


class dynamic_car : public vehicles {
public:
	dynamic_car(unsigned int seed, 
		int L_in, int W_in,
		int abs_L_in, int abs_W_in,
		double start_x, double start_y, 
		double goal_x, double goal_y) : gen(seed), disphi(-3.14, 3.14),
												disv(-v_max, v_max),
												disu_0(-u_0_max, u_0_max),
												disu_1(-u_1_max, u_1_max),
												distime(T_min, T_max)
	{
		L = L_in;
		W = W_in;
		abs_L = abs_L_in;
		abs_W = abs_W_in;
		res_x = abs_W/(double)W;
		res_y = abs_L/(double)L;
		start_state.x = start_x;
		start_state.y = start_y;
		goal_state.x = goal_x;
		goal_state.y = goal_y;
		Tree.push_back(start_state);
	}

    double state_distance(int state_ind){
		// normalizes distance to time between states
		double t_cost = 0;
		double theta = Tree[state_ind].v*car_m/car_L*tan(Tree[state_ind].phi);
		if(Tree[state_ind].v*cos(theta) != 0){
			t_cost += (Tree[state_ind].x-state_sample.x)/(Tree[state_ind].v*cos(theta));
		}
		if(Tree[state_ind].v*sin(theta) != 0){
			t_cost += (Tree[state_ind].y-state_sample.y)/(Tree[state_ind].v*sin(theta));
		}
		if(Tree[state_ind].e.u_0 != 0){
			t_cost += (Tree[state_ind].v-state_sample.v)/(Tree[state_ind].e.u_0);
		}
		if(Tree[state_ind].e.u_1 != 0){
			t_cost += (Tree[state_ind].phi-state_sample.phi)/(Tree[state_ind].e.u_1);
		}
		
		return t_cost;
	}

	double path_est(int state_ind){
		return distance(Tree[state_ind].x, Tree[state_ind].y, state_sample.x, state_sample.y)
			+ distance(state_sample.x, state_sample.y, goal_state.x, goal_state.y);
	}

	double forward_simulate(int state_ind, std::vector<bool> blocked)
	{
		double theta, x_dot, y_dot, x_last, y_last, v_dot, phi_dot;
		double cost=0, t=0;

		state_sample = Tree[state_ind];
    	state_sample.e = control_sample;
    	state_sample.p = state_ind;
		while (t < state_sample.e.T) 
		{
			v_dot = control_sample.u_0;
			state_sample.v = clip(state_sample.v+v_dot*dt,-v_max, v_max);

			phi_dot = control_sample.u_1;
			state_sample.phi = state_sample.phi+phi_dot*dt;

			if(state_sample.phi > 3.14){
				state_sample.phi = state_sample.phi-3.14;
			} else if(state_sample.phi < -3.14) {
				state_sample.phi = state_sample.phi+3.14;
			}

			theta = state_sample.v*car_m/car_L*tan(state_sample.phi);

			x_dot = state_sample.v*cos(theta);
			y_dot = state_sample.v*sin(theta);

			x_last = state_sample.x;
			y_last = state_sample.y;

			state_sample.x = state_sample.x+x_dot*dt;
			state_sample.y = state_sample.y+y_dot*dt;

			t+=dt;
			if (state_sample.x<0 || state_sample.x>W || state_sample.y<0 || state_sample.y>L ){
				return DBL_MAX;
			} else{
				if (blocked[coord2grid(state_sample.x, state_sample.y, W)]) {
					return DBL_MAX;
				} else {
					cost += distance(state_sample.x, state_sample.y, x_last, y_last);
				}
			}
		}
		state_sample.g = Tree[state_ind].g+cost;
		return cost;
	}

	bool check_solution(int sol_ind, std::vector<bool> blocked){
		int ind = sol_ind;
		double c;

		while (Tree[ind].p > 0){
			control_sample = Tree[ind].e;
			forward_simulate(Tree[ind].p, blocked);
			if (state_sample.x!=Tree[ind].x || state_sample.y!=Tree[ind].y){

				cout<<"**************SOLUTION ERROR!*****************\n";
				return false;
			}
			ind = Tree[ind].p;
		}
		return true;
	}

	void random_control() {
    	control_sample.u_0 = disu_0(gen);
    	control_sample.u_1 = disu_1(gen);
    	control_sample.T = distime(gen);
    }

    int region_of_state(int state_ind){
    	return coord2grid(Tree[state_ind].x*res_x, Tree[state_ind].y*res_y, abs_W);
    }

    void state_in_region(int region){
    	double x,y;

		grid2coord(region, &x, &y, W);

    	uniform_real_distribution<> disx(x, x+1);
    	uniform_real_distribution<> disy(y, y+1);
    	
    	state_sample.x = disx(gen);
    	state_sample.y = disy(gen);
    	state_sample.phi = disphi(gen);
    	
    	random_control();
    	state_sample.e = control_sample;
    }

    int nearest_neighbor() {
    	double dist, min_dist = DBL_MAX, min_index;

    	for (int n=0; n<Tree.size(); n++){
    		dist = distance(state_sample.x, state_sample.y, Tree[n].x, Tree[n].y);
    		if (dist < min_dist) {
    			min_dist = dist;
    			min_index = n;
    		}
    	}

    	return min_index;
    }

    void push_state(int expanded){
		Tree.push_back(best_state);
	}

	void save_best(){
		best_state = state_sample;
	}

	double g_val(int state_ind){
		return Tree[state_ind].g;
	}

	double sample_f_val(){
		return state_sample.g+distance(state_sample.x, state_sample.y, goal_state.x, goal_state.y);
    }

    double sample_g_val(){
		return state_sample.g;
    }

    int sample_region(){
    	//cout<<"("<<state_sample.x<<","<<state_sample.y<<") res ("<<res_x<<","<<res_y<<")\n";
    	return coord2grid(state_sample.x*res_x, state_sample.y*res_y, abs_W);
	}

	int parent(int state_ind){
		return Tree[state_ind].p;
	}

	double goal_eval(int state_ind){
		return distance(Tree[state_ind].x, Tree[state_ind].y, goal_state.x, goal_state.y);
	}

	double state_radius(int a, int b){
		return distance(Tree[a].x, Tree[a].y,Tree[b].x, Tree[b].y);
	}

    void print_state(int state_ind, ofstream& file){
    	state s = Tree[state_ind];
    	state p = Tree[s.p];

		file<<p.x<<","<<
			p.y<<","<<
			p.phi<<","<<
			p.v<<","<<
			s.e.u_0<<","<<
			s.e.u_1<<","<<
			s.e.T<<","<<
			s.x<<","<<
			s.y<<"\n";
	}

	void print_solution(int sol_ind, ofstream& file){
		int ind = sol_ind;

		while (Tree[ind].p > 0){
			cout<<ind<<"<-"<<Tree[ind].p<<"\n";
			print_state(ind, file);
			ind = Tree[ind].p;
		}
		print_state(ind, file);
	}

	void push_point(ANNpointArray points, int expanded){
		points[expanded-1][0] = best_state.x;
		points[expanded-1][1] = best_state.y;
	}

	void assign_query(ANNpoint queryPt, int state_ind){
		if (state_ind == -1){
			queryPt[0] = state_sample.x;
			queryPt[1] = state_sample.y;
		} else {
			queryPt[0] = Tree[state_ind].x;
			queryPt[1] = Tree[state_ind].y;
		}
	}

	private:
		const double dt = 0.01, T_min = .1, T_max = 1;
		const double u_0_max = 1, u_1_max = 0.5, v_max = 5;
		const double car_m=1, car_L=1;

		struct control{
		double u_0=0, u_1=0, T=0;
		};

		struct state{
			double g=0, x=0, y=0, phi=0, v=0;
		    int p=0;
		   	control e;
		};

		int L,W,abs_L,abs_W;
		double res_x, res_y;
		state start_state, goal_state, state_sample, best_state;
		control control_sample;
		vector<state> Tree;

    	// State Distributions
	    std::mt19937 gen{std::random_device{} () };
	    std::uniform_real_distribution<> disphi;
	    std::uniform_real_distribution<> disv;

	    // Control Distributions
	    std::uniform_real_distribution<> disu_0;
	    std::uniform_real_distribution<> disu_1;
	    std::uniform_real_distribution<> distime;
};