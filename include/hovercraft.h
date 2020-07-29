/* Defines vehicle- hovercraft*/

#include"vehicles.h"
#pragma once

using namespace robot;

class hovercraft : public vehicles {
public:
	hovercraft(unsigned int seed, 
		int L_in, int W_in,
		int abs_L_in, int abs_W_in, 
		double start_x, double start_y, 
		double goal_x, double goal_y) : gen(seed), disphi(-3.14, 3.14),
												disu(0, u_max),
												disv(-v_max, v_max),
												disr(-r_max, r_max),
												disfu(0, Fu_max),
												disdelta(-delta_max, delta_max),
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
		
		if(Tree[state_ind].u*cos(Tree[state_ind].phi)-Tree[state_ind].v*sin(Tree[state_ind].phi) != 0){
			t_cost += (Tree[state_ind].x-state_sample.x)/(Tree[state_ind].u*cos(Tree[state_ind].phi)-Tree[state_ind].v*sin(Tree[state_ind].phi));  	// x distance divided by x speed
		}
		if(Tree[state_ind].u*sin(Tree[state_ind].phi)+Tree[state_ind].v*cos(Tree[state_ind].phi) != 0){
			t_cost += (Tree[state_ind].y-state_sample.y)/(Tree[state_ind].u*sin(Tree[state_ind].phi)+Tree[state_ind].v*cos(Tree[state_ind].phi)); 	// normalized y
		}
		if(Tree[state_ind].r != 0){
			t_cost += (Tree[state_ind].phi-state_sample.phi)/Tree[state_ind].r;																		// normalized phi
		}
		if(-Tree[state_ind].u+Tree[state_ind].e.Fu != 0){
			t_cost += (Tree[state_ind].u-state_sample.u)/(-Tree[state_ind].u+Tree[state_ind].e.Fu);													// normalized forward velocity
		}
		if(p_v*Tree[state_ind].e.delta*Tree[state_ind].e.Fu != 0){
			t_cost += (Tree[state_ind].v-state_sample.v)/(p_v*Tree[state_ind].e.delta*Tree[state_ind].e.Fu); 										// normalized sway velocity
		}
		if(p_psi*Tree[state_ind].e.delta*Tree[state_ind].e.Fu != 0){
			t_cost += (Tree[state_ind].r-state_sample.r)/(p_psi*Tree[state_ind].e.delta*Tree[state_ind].e.Fu); 									// normalized moment velocity
		}
		return t_cost;
	}

	double path_est(int state_ind){
		return distance(Tree[state_ind].x, Tree[state_ind].y, state_sample.x, state_sample.y)
			+ distance(state_sample.x, state_sample.y, goal_state.x, goal_state.y);
	}

	double forward_simulate(int state_ind, std::vector<bool> blocked)
	{
		state_sample = Tree[state_ind];
		double Fv = p_v*control_sample.delta*control_sample.Fu;
		double M = p_psi*control_sample.delta*control_sample.Fu;
		double u_dot, v_dot = Fv, r_dot = M;
		double x_dot, y_dot, phi_dot;
		double cost = 0;
		double x_last, y_last;

		double t=0;
    	state_sample.e = control_sample;
    	state_sample.p = state_ind;
		while (t < state_sample.e.T) 
		{
			u_dot = -state_sample.u+state_sample.e.Fu;
			state_sample.u = clip(state_sample.u+u_dot*dt, 0, u_max);
			state_sample.v = clip(state_sample.v+v_dot*dt, -v_max, v_max);
			state_sample.r = clip(state_sample.r+r_dot*dt, -r_max, r_max);

			phi_dot = state_sample.r;
			state_sample.phi = state_sample.phi+phi_dot*dt;

			x_dot = state_sample.u*cos(state_sample.phi)-state_sample.v*sin(state_sample.phi);
			y_dot = state_sample.u*sin(state_sample.phi)+state_sample.v*cos(state_sample.phi);

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
				cout<<"Shoulde go from G["<<Tree[ind].p<<"] @ ("<<Tree[Tree[ind].p].x<<","<<Tree[Tree[ind].p].y<<") (u,v,r,phi) = ("<<Tree[Tree[ind].p].u<<","<<Tree[Tree[ind].p].v<<","<<Tree[Tree[ind].p].r<<","<<Tree[Tree[ind].p].phi<<")\n";
				cout<<"\tand end @ G["<<ind<<"] @ ("<<Tree[ind].x<<","<<Tree[ind].y<<") (u,v,r,phi) = ("<<Tree[ind].u<<","<<Tree[ind].v<<","<<Tree[ind].r<<","<<Tree[ind].phi<<")\n";
				cout<<"\t with control (Fu,delta,T) = ("<<state_sample.e.Fu<<","<<state_sample.e.delta<<","<<state_sample.e.T<<")\n";
				cout<<"\t\t but ends @ \tG["<<ind<<"] @ ("<<state_sample.x<<","<<state_sample.y<<") (u,v,r,phi) = ("<<state_sample.u<<","<<state_sample.v<<","<<state_sample.r<<","<<state_sample.phi<<")\n";
				return false;
			}
			ind = Tree[ind].p;
		}
		return true;
	}

	void random_control() {
    	control_sample.Fu = disfu(gen);
    	control_sample.delta = disdelta(gen);
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
    	state_sample.u = disu(gen);
    	state_sample.v = disv(gen);
    	state_sample.r = disr(gen);
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
			p.u<<","<<
			p.v<<","<<
			p.r<<","<<
			s.e.Fu<<","<<
			s.e.delta<<","<<
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
		const double dt = 0.01, T_min = 1, T_max = 5;
		const double Fu_max = 2, p_v = 0.2, p_psi = -.2, delta_max = .9*3.14/2;
		const double u_max = 1, v_max = .2, r_max = .5;

		struct control{
		double Fu=0;
		double delta=0;
		double T=0;
		};

		struct state{
			double g=0;
		    double u=0;
		    double v=0;
		    double r=0;
		    double x=0;
		    double y=0;
		    double phi=0;
		    int p=0;
		   	control e;
		};

		int L,W,abs_L,abs_W;
		double res_x,res_y;
		state start_state, goal_state, state_sample, best_state;
		control control_sample;
		vector<state> Tree;

    	// State Distributions
	    std::mt19937 gen{std::random_device{} () };
	    std::uniform_real_distribution<> disphi;
	    std::uniform_real_distribution<> disu;
	    std::uniform_real_distribution<> disv;
	    std::uniform_real_distribution<> disr;

	    // Control Distributions
	    std::uniform_real_distribution<> disfu;
	    std::uniform_real_distribution<> disdelta;
	    std::uniform_real_distribution<> distime;
};