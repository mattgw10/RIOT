/* Region Informed Optimal Trees (RIOT) */

#include<iostream>
#include<algorithm>
#include<string>
#include<stdlib.h>
#include<chrono>
#include"include/hovercraft.h"
#include"include/dynamic_car.h"
#include"include/generic_3D_dynamic.h"

/* Constants:
	dim = dimensions of workspace
	bn = number of controls to blossum
	eps = ANN error bound
	GOAL_RADIUS = distance vertex can be from goal and count as reaching it*/
const int bn = 20;
const double GOAL_RADIUS = 1;

using namespace robot;

int main(int argc, char* argv[])
{
	// Start Time
	chrono::steady_clock::time_point begin = chrono::steady_clock::now();
	
	/* Arguments
		-m [map] -v [vehicle] -t [time]  -s [start grid cell] -g [goal grid cell]
	*/
	ifstream infile;
	string vehicle_name;
	vehicles* vehicle = nullptr;
	space::map* env = nullptr;
	grid_decomposition* decomp = nullptr;
	double time;
	int start, goal;
	for(int i=0; i < argc; i++){
		if (string(argv[i])=="-m"){
			infile.open (argv[i+1]);
		} else if (string(argv[i])=="-v"){
			vehicle_name = string(argv[i+1]);
		} else if (string(argv[i])=="-t"){
			time = atof(argv[i+1]);
		} else if (string(argv[i])=="-s"){
			start = atoi(argv[i+1]);
		} else if (string(argv[i])=="-g"){
			goal = atoi(argv[i+1]);
		}
	}
	
	// Random generator
	random_device rd;  //Will be used to obtain a seed for the random number engine
    mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    discrete_distribution<int> region_selection, state_selection;

	if(vehicle_name == "hovercraft" || vehicle_name == "dynamic_car"){
		int L, W, abs_L, abs_W;
		double start_x, start_y, goal_x, goal_y;
		infile >> L;
		infile >> W;
		infile >> abs_L;
		infile >> abs_W;

		env = new map_2D(L,W);
		decomp = new grid_2D(L,W,abs_L,abs_W);

		grid2coord(start, &start_x, &start_y, W);
		grid2coord(goal, &goal_x, &goal_y, W);

		start = coord2grid(start_x*abs_W/W, start_y*abs_L/L, abs_W);
		goal = coord2grid(goal_x*abs_W/W, goal_y*abs_L/L, abs_W);

		if(vehicle_name == "hovercraft"){
			vehicle = new hovercraft(rd(), L,W,abs_L,abs_W, start_x, start_y, goal_x, goal_y);
		} else if(vehicle_name == "dynamic_car"){
			vehicle = new dynamic_car(rd(), L,W,abs_L,abs_W, start_x, start_y, goal_x, goal_y);
		}
	} else if(vehicle_name == "generic_3D"){
		int L, W, H, abs_L, abs_W, abs_H;
		double start_x, start_y, start_z, goal_x, goal_y, goal_z;
		infile >> L;
		infile >> W;
		infile >> H;
		infile >> abs_L;
		infile >> abs_W;
		infile >> abs_H;

		env = new map_3D(L,W,H);
		decomp = new grid_3D(L,W,H,abs_L,abs_W,abs_H);

		grid2coord(start, &start_x, &start_y, &start_z, W, L);
		grid2coord(goal, &goal_x, &goal_y, &goal_z, W, L);

		start = coord2grid(start_x*abs_W/W, start_y*abs_L/L, start_z*abs_H/H, abs_W, abs_L);
		goal = coord2grid(goal_x*abs_W/W, goal_y*abs_L/L, goal_z*abs_H/H, abs_W, abs_L);

		if(vehicle_name == "generic_3D"){
			vehicle = new generic_3D(rd(), L,W,H,abs_L,abs_W,abs_H, start_x, start_y, start_z, goal_x, goal_y, goal_z);
		}
	} else {
		cout<<"Invalid Vehicle: "<<vehicle_name<<"\n Exiting...\n";
		return 0;
	}

	env->read_map(infile);
	infile.close();
	
	decomp->initialize_abstraction(env->blocked);
	uniform_int_distribution<int> distr(0, env->size);

	// Solution
	double sol = DBL_MAX;
	int generated = 1, expanded = 1, sol_ind, v, vr;

	// Regions to select from
	int selected, r=0, r_last=0;
	decomp->adjust_interior(start);
	decomp->states_in_region[start].push_back(expanded-1);
	decomp->states_g[start].push_back(0);
	decomp->state_count[start] = 1;

	// Control
	double c,f,abs_f,abs_h,f_best = DBL_MAX, abs_f_best,abs_h_best;
	bool save, greedy = false;
	vector<int> blossum_num;

	blossum_num.push_back(bn);

	while ((chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - begin).count()) /1000000.0 < time)
	{
		/* Greedy selection of newest node if in better heuristic value abstraction or
			equal absraction heuristic and better euclidean heuristic.
			Otherwise select region with prob 1/f, and randomly select state in that region*/
		if (greedy) {
			r_last = r;
			selected = expanded-1;
		} else {
			if (sol == DBL_MAX) {
				// Extend nearest neighbor
				decomp->abstraction_search(start, goal)
				selected = distr(gen);
				vehicle->state_in_region(selected);
			    selected = vehicle->nearest_neighbor();
			} else {
				// Sample from interior region
				region_selection = discrete_distribution<int>(decomp->interior.begin(), decomp->interior.end());
				r_last = region_selection(gen);

				state_selection = discrete_distribution<int>(decomp->states_g[r_last].begin(), decomp->states_g[r_last].end());
				selected = decomp->states_in_region[r_last][state_selection(gen)];
			}
		}
		
		greedy = false;
		f_best = DBL_MAX;
		abs_f_best = DBL_MAX;
		abs_h_best = DBL_MAX;
		// Branch and Bound
		if (vehicle->g_val(selected) + vehicle->goal_eval(selected) < sol+GOAL_RADIUS){
			// Blossom
			for (int i=0;i<blossum_num[selected];i++){
				save = false;
				vehicle->random_control();
				c = vehicle->forward_simulate(selected, env->blocked);
				r = vehicle->sample_region();
				if (c<DBL_MAX){
					decomp->update_prob(r, 1);
					
					abs_f = decomp->f[r];
					abs_h = decomp->h[r];
					f =  vehicle->sample_f_val();
					// Take control with best abstraction f, then abstraction h, then euclidean f
					if (abs_f < abs_f_best){
						save = true;
					} else if (abs_f == abs_f_best) {
						if (abs_h < abs_h_best){
							save = true;
						} else if (abs_h < abs_h_best) {
							if (f < f_best){
								save = true;
							}
						}
					}

					if (save and f < sol + GOAL_RADIUS){
						f_best = f;
						abs_f_best = abs_f;
						abs_h_best = abs_h;
						vehicle->save_best();
					}
				} else {
					decomp->update_prob(r, 0);
				}
			}
		}
		
		// Graph expanded
		if (f_best<DBL_MAX){
			expanded+=1;
			vehicle->push_state(expanded);
			blossum_num.push_back(bn);
			
			r = vehicle->region_of_state(expanded-1);
			decomp->adjust_interior(r);
			decomp->states_g[r].push_back(vehicle->g_val(expanded-1));
			decomp->states_in_region[r].push_back(expanded-1);
			decomp->state_count[r] += 1;
			decomp->update_max_g(r, vehicle->g_val(expanded-1));
			if (vehicle->goal_eval(expanded-1) < GOAL_RADIUS){
				if (vehicle->g_val(expanded-1) < sol){
					sol = vehicle->g_val(expanded-1);
					sol_ind = expanded-1;
					cout<<sol<<" "<<(chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - begin).count()) /1000000.0 <<
					" "<<expanded<<" "<<generated<<"\n";
					v = expanded-1;
					while (v != 0){
						vr = vehicle->region_of_state(v);
						decomp->update_max_h(vr, sol-vehicle->g_val(v));
						v = vehicle->parent(v);
					}
					decomp->update_max_h(start, sol);
				}
			}
			if(decomp->h[r] < decomp->h[r_last] || 
				((decomp->h[r] == decomp->h[r_last]) && (vehicle->goal_eval(expanded-1) < vehicle->goal_eval(vehicle->parent(expanded-1))))){
				greedy = true;
			}
		}
		blossum_num[selected] = 1;
		generated+=1;
	}
	delete(env);
	delete(vehicle);
	delete(decomp);
    return 0;
}
