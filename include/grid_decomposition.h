/* Graph Abstraction Functions*/

#include"environment.h"
#include"dijkstras.h"

#pragma once

using namespace space;

namespace abstraction {
	class grid_decomposition : public Graph{
	public:
		grid_decomposition(int size_in, int dim_in):Graph(size_in){
			abs_size = size_in;

			for(int i=0; i<abs_size; i++){
				g.push_back(DBL_MAX);
				h.push_back(DBL_MAX);
				g_max.push_back(DBL_MAX);
				h_max.push_back(DBL_MAX);
				f.push_back(DBL_MAX);
				P_s.push_back(1.0);
				props.push_back(10);
				interior.push_back(0.0);
				state_count.push_back(0);
				states_g.push_back(vector<double>() );
				states_in_region.push_back(vector<int>() );
			}
		};

		int abs_size;

		vector<double> g;
		vector<double> h;
		vector<double> g_max;
		vector<double> h_max;
		vector<double> f;
		vector<double> P_s;
		vector<int> props;
		vector<double> interior;
		vector<int> state_count;
		vector<vector<double>> states_g;
		vector<vector<int>> states_in_region;

		virtual void initialize_abstraction(vector<bool> const &blocked){}
		virtual void print_abs_dims(ofstream& file){}


		void adjust_interior(int region)
		{
			if (interior[region] == 0){
				interior[region] = 1/f[region];
			}
		}

		void update_prob(const int r, const int success){
			if (r>= 0 && r<abs_size){
				P_s[r] = (P_s[r]*props[r]+success)/(props[r]+1);
				props[r] = props[r]+1;
			}
		}


		void update_max_g(const int r, const double new_g){
			if (g_max[r] > new_g){
				g_max[r] = new_g;
			}
		}

		void update_max_h(const int r, const double new_h){
			h_max[r] = new_h;
		}

		void abstraction_search(int start, int goal){
			shortestPath(g, g_max, P_s, start);
			shortestPath(h, h_max, P_s, goal);

			for (int i=0;i<abs_size;i+=1){
				f[i] = g[i]+h[i];
				if (interior[i] != 0){
					interior[i] = 1/f[i];
				}
			}
		}
	};

	class grid_2D : public grid_decomposition{
	public:
		int abs_L;
		int abs_W;
		int env_L;
		int env_W;
		grid_2D(int gL, int gW, int abs_gL, int abs_gW):grid_decomposition(abs_gL*abs_gW,2){
			abs_L = abs_gL;
			abs_W = abs_gW;
			env_L = gL;
			env_W = gW;
		}

		void initialize_abstraction(vector<bool> const &blocked){
			// Create graph
			double x, y, res;
			
			for(int n=0; n<abs_size; n++){			// Add edges
				y = floor(n/abs_W);
				x = n%abs_W;
				if (y < abs_L-1){					// Add edge up
					addEdge(n, n+abs_W, 1*env_L/double(abs_L));
				}
				if (x > 0){							// Add edge left
					addEdge(n, n-1, 1*env_W/double(abs_W)); 
				}
				if (y < abs_L-1 && x > 0){				// Add edge up and left
					addEdge(n, n+abs_W-1, sqrt(env_L/double(abs_L)+env_W/double(abs_W)));
				}
				if (y < abs_L-1 && x < abs_W-1){			// Add edge up and right
					addEdge(n, n+abs_W+1, sqrt(env_L/double(abs_L)+env_W/double(abs_W))); 
				}
			}

			res = double(1/((env_W/abs_W)*(env_L/abs_L)));
			for (int n=0;n<env_L*env_W;n+=1){
				if (blocked[n]){
					grid2coord(n, &x, &y, env_W);
					x = x*abs_W/env_W;
					y = y*abs_L/env_L;
					P_s[coord2grid(x,y,abs_W)] -= res;
				}
			}
		}

		void print_abs_dims(ofstream& file){
			file<<abs_L<<","<<abs_W<<"\n";
		}
	};

	class grid_3D : public grid_decomposition{
	public:
		int abs_L;
		int abs_W;
		int abs_H;
		int env_L;
		int env_W;
		int env_H;
		grid_3D(int gL, int gW, int gH, int abs_gL, int abs_gW, int abs_gH):grid_decomposition(abs_gL*abs_gW*abs_gH,3){
			abs_L = abs_gL;
			abs_W = abs_gW;
			abs_H = abs_gH;
			env_L = gL;
			env_W = gW;
			env_H = gH;
		}

		void initialize_abstraction(vector<bool> const &blocked){
			// Create graph
			double x,y,z,res;
			
			// Add edges 6 ways (could be 26 ways if all diagonals counted)
			for(int n=0; n<abs_size; n++){
				z = floor(n/(abs_W*abs_L));
				y = floor(n%(abs_W*abs_L)/abs_W);
				x = n%(abs_W*abs_L)%abs_W;
				// Add edges
				if (z < abs_H-1){				// Add edge above
						addEdge(n, n+abs_W*abs_L, 1*env_H/double(abs_H));
				}
				if (y < abs_L-1){				// Add edge forward
						addEdge(n, n+abs_W, 1*env_L/double(abs_L));
				}
				if (x < abs_W-1){				// Add edge right
						addEdge(n, n+1, 1*env_W/double(abs_W));
				}
			}

			res = double(1/((env_W/abs_W)*(env_L/abs_L)*(env_H/abs_H)));
			for (int n=0;n<env_L*env_W*env_H;n+=1){
				if (blocked[n]){
					grid2coord(n, &x, &y, &z, env_W, env_L);
					x = x*abs_W/env_W;
					y = y*abs_L/env_L;
					z = z*abs_H/env_H;
					P_s[coord2grid(x,y,abs_W)] -= res;
					P_s[coord2grid(x,y,z,abs_W, abs_L)] -= res;
				}
			}
		}

		void print_abs_dims(ofstream& file){
			file<<abs_L<<","<<abs_W<<","<<abs_H<<"\n";
		}
	};
}