/* Environment Functions*/

#include<vector>
#include<math.h>
#include<fstream>

#pragma once

using namespace std;

namespace space {
	// Overload functions for 2D and 3D
	int coord2grid(int x, int y, int W)
	{
		return y*W+x;
	}
	int coord2grid(double x, double y, int W)
	{
		return floor(y)*W+floor(x);
	}
	void grid2coord(int n, double* x, double* y, int W)
	{
		*x = n%W+0.5;
		*y = floor(n/W)+0.5;
	}
	double distance(double x1, double y1, double x2, double y2){
		return sqrt(pow(x1 - x2, 2) + pow(y1 -  y2, 2));
	}

	int coord2grid(int x, int y, int z, int W, int L)
	{
		return z*W*L+y*W+x;
	}
	int coord2grid(double x, double y, double z, int W, int L)
	{
		return floor(z)*W*L+floor(y)*W+floor(x);
	}
	void grid2coord(int n, double* x, double* y, double* z, int W, int L)
	{
		*z = floor(n/(W*L))+0.5;
		*y = floor(n%(W*L)/W)+0.5;
		*x = n%(W*L)%W+0.5;
	}
	double distance(double x1, double y1, double z1, double x2, double y2, double z2){
		return sqrt(pow(x1 - x2, 2) + pow(y1 -  y2, 2) + pow(z1 -  z2, 2));
	}

	double distance(double x1, double y1, double z1, double a1, double x2, double y2, double z2, double a2){
		return sqrt(pow(x1 - x2, 2) + pow(y1 -  y2, 2) + pow(z1 -  z2, 2) + pow(a1 -  a2, 2));
	}

	double distance(double x1, double y1, double z1, double a1, double b1, double x2, double y2, double z2, double a2, double b2){
		return sqrt(pow(x1 - x2, 2) + pow(y1 -  y2, 2) + pow(z1 -  z2, 2) + pow(a1 -  a2, 2) + pow(b1 -  b2, 2));
	}

	
	// Apply max and min value to number
	double clip(double n, double lower, double upper) {
	  return max(lower, min(n, upper));
	}
	
	struct neighbor{
		double g;
		int ind;
		neighbor(double gval, int indval){
			g = gval;
			ind = indval;
		}
	};

	struct neighbor_rank {
		bool operator()(neighbor const& a, neighbor const& b) const {
			return a.g > b.g;
		}
	};

	void print_grid(double grid[], int N, ofstream& file){
		for(int i=0;i<N;i++)                       /*will print the vertex with their distance from the source to the console */
		{
			file<<i<<","<<grid[i]<<"\n";
		}
	}

	void print_grid(vector<double> grid, int N, ofstream& file){
		for(int i=0;i<N;i++)                       /*will print the vertex with their distance from the source to the console */
		{
			file<<i<<","<<grid[i]<<"\n";
		}
	}

	void print_grid(vector<bool> grid, int N, ofstream& file){
		for(int i=0;i<N;i++)                       /*will print the vertex with their distance from the source to the console */
		{
			if(grid[i]){
				file<<i<<",NaN\n";
			} else {
				file<<i<<",0\n";
			}
		}
	}

	class map{
	public:
		map(int size_in, int dim_in) {
			size = size_in;
			dim = dim_in;

			for(int i=0; i<size; i++){
				blocked.push_back(false);
			}
		};

		int dim;
		int size;

		std::vector<bool> blocked;
		virtual void read_map(ifstream& file){}
		virtual void print_dims(ofstream& file){}
	};

	class map_2D : public map{
	public:
		int L;
		int W;
		map_2D(int gL, int gW):map(gL*gW,2){
			L = gL;
			W = gW;
		}

		void read_map(ifstream& file){
			// Create graph
			char line[W];
			int n;

			for(int i=0; i<L; i++){
				file >> line;
				for(int j=0; j<W; j++) {
					n = (L-i-1)*W+j;	// Current grid cell
					if (line[j] != '.'){ 			// Blocked
						blocked[n] = true;
					}
				}
			}
		}

		void print_dims(ofstream& file){
			file<<L<<","<<W<<"\n";
		}
	};

	class map_3D : public map{
	public:
		int L;
		int W;
		int H;
		map_3D(int gL, int gW, int gH):map(gL*gW*gH,3){
			L = gL;
			W = gW;
			H = gH;
		}

		void read_map(ifstream& file){
			// Create graph
			char line[W];
			int cell_blocked, x1,x2,y1,y2,z1,z2;

			// Reads rectangular obstacles
			// cell_blocked = 1 is obstacle, 0 is space (put in file after obstacles to define gaps in large ones)
			while(true){
				if( file.eof() ) break;
				file >> cell_blocked >> x1 >> x2 >> y1 >> y2 >> z1 >> z2;
				if (cell_blocked == 1){
					for(int x=x1; x<=x2; x++){
						for(int y=y1; y<=y2; y++){
							for(int z=z1; z<=z2; z++){
								blocked[coord2grid(x, y, z, W, L)] = true;
							}
						}
					}
				} else {
					for(int x=x1; x<=x2; x++){
						for(int y=y1; y<=y2; y++){
							for(int z=z1; z<=z2; z++){
								blocked[coord2grid(x, y, z, W, L)] = false;
							}
						}
					}
				}
			}
		}

		void print_dims(ofstream& file){
			file<<L<<","<<W<<","<<H<<"\n";
		}
	};
}