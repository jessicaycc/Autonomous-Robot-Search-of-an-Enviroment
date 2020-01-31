
#include "frontier_search.h"

vector<vector<int>> occ_grid;
int occ_grid_width = 0; 
int occ_grid_height = 0;

using namespace std;

//enum Marker {MAP_OPEN_LIST=0, MAP_CLOSE_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST};

bool is_a_frontier_point(pair<int,int> p){
	vector<pair<int,int>> neighbours = findNeighbours(p);
	if(occ_grid[p.first][p.second] >= 10){
		for(auto nei: neighbours){
			if(occ_grid[nei.first][nei.second] == -1){
				return true;
			}
		}
	}
	return false;

	
}

vector<pair<int,int>> get_medians(vector<vector<pair<int,int>>> list_of_frontiers){
	//pick out the median frontier point from each set
	vector<pair<int, int>> destinations;
	for (auto cur_front: list_of_frontiers){
		sort(cur_front.begin(), cur_front.end());
		destinations.push_back(cur_front[cur_front.size()/2]);
	}

	return destinations; 
}

bool is_open_neighbour(unordered_map<pair<int,int>, int> marker_list, pair<int,int> p){
	//this point has a neighbour that is map_open_list

	vector<pair<int,int>> neighbours = findNeighbours(p);
	for(auto point: neighbours){
		if(marker_list.find(point)!=marker_list.end() &&
			marker_list[point]==MAP_OPEN_LIST){return true;}

	}
	return false;
}
vector<pair<int,int>> findNeighbours(pair<int,int> p){
	//BFS find adj neighbours
	static vector<pair<int,int>> directions = {(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)}; 
	vector<pair<int,int>> neighbours;
	
	for(auto d: directions){
		pair<int, int> temp = (p.first + d.first, p.second + d.second);
		if(temp.first < 0 || temp.second < 0 || temp.first >= occ_grid_height || temp.second >= occ_grid_width || occ_grid[temp.first][temp.second] >=10){
			continue;
		}
		neighbours.push_back(temp);
	}
	return neighbours;
}

vector< vector<pair<int,int>>> wfd(int argc, char **argv)
{

	//Store all new frontiers here
	vector<vector<pair<int, int>>> list_of_frontiers;

	//For keeping track of visited
	unordered_map<pair<int,int>, Marker> marker_list;

	//BFS queue 1
	queue<pair<int, int>, int> queue_m;
	pair<int, int> pose(1,1);
	//get_current_pose();

	while(!queue_m.empty()){
		pair<int, int> p = queue_m.front();
		queue_m.pop();
		
		if(marker_list.find(p)!=marker_list.end && 
			marker_list[p]==MAP_CLOSE_LIST){
			continue; 	
		} else if(is_a_frontier_point(p)){
			queue<pair<int, int>, int> queue_f;
			vector<pair<int, int>> new_frontier;
			queue_f.push(p);
			marker_list[p] = FRONTIER_OPEN_LIST;

			while(!queue_f.empty()){
				pair<int, int> q = queue_f.front();
				queue_f.pop();
				if(marker_list.find(q)!=marker_list.end() &&
					(marker_list[q] == MAP_CLOSE_LIST || 
					marker_list[q] == FRONTIER_CLOSE_LIST)){
					continue;
				}
				else if(is_a_frontier_point(q)){
					new_frontier.push_back(q);
					for(auto adj : findNeighbours(q)){
						if(marker_list.find(adj)!=marker_list.end() &&
						(marker_list[adj] == FRONTIER_OPEN_LIST ||
							marker_list[adj] == FRONTIER_CLOSE_LIST ||
							marker_list[adj] == MAP_CLOSE_LIST)){
							continue;					
						}
						queue_f.push(adj);
						marker_list[adj] = FRONTIER_OPEN_LIST;
					} 
					marker_list[adj] = FRONTIER_CLOSE_LIST;
				}						
			}
			list_of_frontiers.push_back(new_frontier); //Optimize this
			for(pair<int,int> mem : new_frontier){
				marker_list[mem] = MAP_CLOSE_LIST;		
			}
		}

		for(pair<int,int> v : findNeighbours(p){
			if(marker_list.find(v)!=marker_list.end() &&
			(marker_list[v]==MAP_CLOSE_LIST || marker_list[v]==MAP_OPEN_LIST)
			){
				continue;
			}
			if(has_open_neighbour(v)){
				queue_m.push(v);
				marker_list[v] = MAP_OPEN_LIST;
			}
		}
		marker_list[p] = MAP_CLOSE_LIST;	
	}
}
	

