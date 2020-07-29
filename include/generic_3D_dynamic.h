/* Defines vehicle- generic 3D dynamic point robot*/

#include"vehicles.h"
#pragma once

using namespace robot;

class generic_3D: public vehicles {
public:
	generic_3D(unsigned int seed, 
		int L_in, int W_in, int H_in,
		int abs_L_in, int abs_W_in, int abs_H_in,
		double start_x, double start_y, double start_z, 
		double goal_x, double goal_y, double goal_z) : gen(seed),
												disv(-v_max, v_max),
												disu(-u_max, u_max),
												distime(T_min, T_max)
	{
		L = L_in;
		W = W_in;
		H = H_in;
		abs_L = abs_L_in;
		abs_W = abs_W_in;
		abs_H = abs_H_in;
		res_x = abs_W/(double)W;
		res_y = abs_L/(double)L;
		res_z = abs_H/(double)H;
		start_state.x = start_x;
		start_state.y = start_y;
		start_state.z = start_z;
		goal_state.x = goal_x;
		goal_state.y = goal_y;
		goal_state.z = goal_z;
		Tree.push_back(start_state);
	}

    double state_distance(int state_ind){
		// normalizes distance to time between states
		double t_cost = 0;

		if(Tree[state_ind].vx != 0){
			t_cost += (Tree[state_ind].x-state_sample.x)/(Tree[state_ind].vx);
		}
		if(Tree[state_ind].vy != 0){
			t_cost += (Tree[state_ind].y-state_sample.y)/(Tree[state_ind].vy);
		}
		if(Tree[state_ind].vz != 0){
			t_cost += (Tree[state_ind].z-state_sample.z)/(Tree[state_ind].vz);
		}
		if(Tree[state_ind].e.ux != 0){
			t_cost += (Tree[state_ind].vx-state_sample.vx)/(Tree[state_ind].e.ux);
		}
		if(Tree[state_ind].e.uy != 0){
			t_cost += (Tree[state_ind].vy-state_sample.vy)/(Tree[state_ind].e.uy);
		}
		if(Tree[state_ind].e.uz != 0){
			t_cost += (Tree[state_ind].vz-state_sample.vz)/(Tree[state_ind].e.uz);
		}
		
		return t_cost;
	}

	double path_est(int state_ind){
		return distance(Tree[state_ind].x, Tree[state_ind].y, Tree[state_ind].z, state_sample.x, state_sample.y, state_sample.z)
			+ distance(state_sample.x, state_sample.y, state_sample.z, goal_state.x, goal_state.y, goal_state.z);
	}

	double forward_simulate(int state_ind, std::vector<bool> blocked)
	{
		double x_last, y_last, z_last;
		double t=0, cost=0;

		state_sample = Tree[state_ind];
    	state_sample.e = control_sample;
    	state_sample.p = state_ind;
		while (t < state_sample.e.T) 
		{
			state_sample.vx = clip(state_sample.vx+control_sample.ux*dt,-v_max, v_max);
			state_sample.vy = clip(state_sample.vy+control_sample.uy*dt,-v_max, v_max);
			state_sample.vz = clip(state_sample.vz+control_sample.uz*dt,-v_max, v_max);

			x_last = state_sample.x;
			y_last = state_sample.y;
			z_last = state_sample.z;

			state_sample.x = state_sample.x+state_sample.vx*dt;
			state_sample.y = state_sample.y+state_sample.vy*dt;
			state_sample.z = state_sample.z+state_sample.vz*dt;

			t+=dt;
			if (state_sample.x<0 || state_sample.x>W || 
				state_sample.y<0 || state_sample.y>L || 
				state_sample.z<0 || state_sample.z>H){
				return DBL_MAX;
			} else{
				if (blocked[coord2grid(state_sample.x, state_sample.y, state_sample.z, W, L)]) {
					return DBL_MAX;
				} else {
					cost += distance(state_sample.x, state_sample.y, state_sample.z, x_last, y_last, z_last);
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
			if (state_sample.x!=Tree[ind].x || state_sample.y!=Tree[ind].y || state_sample.z!=Tree[ind].z){

				cout<<"**************SOLUTION ERROR!*****************\n";
				return false;
			}
			ind = Tree[ind].p;
		}
		return true;
	}

	void random_control() {
		control_sample.ux = disu(gen);
		control_sample.uy = disu(gen);
		control_sample.uz = disu(gen);
    	control_sample.T = distime(gen);
    }

    int region_of_state(int state_ind){
    	return coord2grid(Tree[state_ind].x*res_x, Tree[state_ind].y*res_y, Tree[state_ind].z*res_z, abs_W, abs_L);
    }

    void state_in_region(int region){
    	double x,y,z;

		grid2coord(region, &x, &y, &z, W, L);

    	uniform_real_distribution<> disx(x, x+1);
    	uniform_real_distribution<> disy(y, y+1);
    	uniform_real_distribution<> disz(z, z+1);
    	
    	state_sample.x = disx(gen);
    	state_sample.y = disy(gen);
    	state_sample.z = disz(gen);
    	state_sample.vx = disv(gen);
    	state_sample.vy = disv(gen);
    	state_sample.vz = disv(gen);
    	
    	random_control();
    	state_sample.e = control_sample;
    }

    int nearest_neighbor() {
    	double dist, min_dist = DBL_MAX, min_index;

    	for (int n=0; n<Tree.size(); n++){
    		dist = distance(state_sample.x, state_sample.y, state_sample.z, Tree[n].x, Tree[n].y, Tree[n].z);
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
		return state_sample.g+distance(state_sample.x, state_sample.y, state_sample.z, goal_state.x, goal_state.y, goal_state.z);
    }

    double sample_g_val(){
		return state_sample.g;
    }
    
    int sample_region(){
    	return coord2grid(state_sample.x*res_x, state_sample.y*res_y, state_sample.z*res_z, abs_W, abs_L);
	}

	int parent(int state_ind){
		return Tree[state_ind].p;
	}

	double goal_eval(int state_ind){
		return distance(Tree[state_ind].x, Tree[state_ind].y, Tree[state_ind].z, goal_state.x, goal_state.y, goal_state.z);
	}

	double state_radius(int a, int b){
		return distance(Tree[a].x, Tree[a].y, Tree[a].z, Tree[b].x, Tree[b].y, Tree[b].z);
	}

    void print_state(int state_ind, ofstream& file){
    	state s = Tree[state_ind];
    	state p = Tree[s.p];

		file<<p.x<<","<<
			p.y<<","<<
			p.z<<","<<
			p.vx<<","<<
			p.vy<<","<<
			p.vz<<","<<
			s.e.ux<<","<<
			s.e.uy<<","<<
			s.e.uz<<","<<
			s.e.T<<"\n";
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
		points[expanded-1][2] = best_state.z;
	}

	void assign_query(ANNpoint queryPt, int state_ind){
		if (state_ind == -1){
			queryPt[0] = state_sample.x;
			queryPt[1] = state_sample.y;
			queryPt[2] = state_sample.z;
		} else {
			queryPt[0] = Tree[state_ind].x;
			queryPt[1] = Tree[state_ind].y;
			queryPt[2] = Tree[state_ind].z;
		}
	}

	private:
		const double dt = 0.01, T_min = 1, T_max = 5;
		const double v_max = 1, u_max = 3;

		struct control{
		double ux=0,uy=0,uz=0,T=0;
		};

		struct state{
			double g=0;
			double x=0, y=0, z=0;
		    double vx=0,vy=0,vz=0;
		    int p=0;
		   	control e;
		};

		int L,W,H,abs_L,abs_W,abs_H;
		double res_x,res_y,res_z;
		state start_state, goal_state, state_sample, best_state;
		control control_sample;
		vector<state> Tree;

    	// State Distributions
	    std::mt19937 gen{std::random_device{} () };
	    std::uniform_real_distribution<> disv;

	    // Control Distributions
	    std::uniform_real_distribution<> disu;
	    std::uniform_real_distribution<> distime;
};