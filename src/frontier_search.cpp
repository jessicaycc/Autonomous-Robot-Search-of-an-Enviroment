
#include "frontier_search.h"
#include "globals.h"

bool is_a_frontier_point(pair<int,int> p){
	vector<pair<int,int>> neighbours = findNeighbours(p);
	if(occ_grid[p.second][p.first] >= 10){
		for(auto nei: neighbours){
			if(occ_grid[nei.second][nei.first] == -1){
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

bool has_open_neighbour(vector<vector<int>> marker_list, pair<int,int> p){
	//this point has a neighbour that is map_open_list

	vector<pair<int,int>> neighbours = findNeighbours(p);
	for(pair<int,int> point: neighbours){
		if(marker_list[point.second][point.first]==MAP_OPEN_LIST)
		{
			return true;
		}

	}
	return false;
}
vector<pair<int,int>> findNeighbours(pair<int,int> p){
	//BFS find adj neighbours
	static vector<pair<int,int>> directions = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0}, {1,1}}; 
	vector<pair<int,int>> neighbours;
	
	for(auto d: directions){
		pair<int, int> temp = {p.first + d.first, p.second + d.second};
		if(temp.first < 0 || temp.second < 0 || 
			temp.first >= occ_height || temp.second >= occ_width || 
			occ_grid[temp.second][temp.first] >=10)
		{
			continue;
		}
		neighbours.push_back(temp);
	}
	return neighbours;
}

vector< vector<pair<int,int>>> wfd()
{
	//Store all new frontiers here
	vector<vector<pair<int, int>>> list_of_frontiers;
	std::cout<<"INITIALIZING WFD: OCC_GRID/HEIGHT" << occ_width <<"/" << occ_height << std::endl;
	std::cout<<"INITIALIZING POSE: " << pose_pos[0] <<", " << pose_pos[1] << std::endl;
	//For keeping track of visited
	vector<vector<int>> marker_list(
		occ_height,
		vector<int>(occ_width, -1)
	); //y,x form (y rows of x length)

	//BFS queue 1
	queue<pair<int, int>> queue_m;
	pair<int, int> pose(int(round(pose_pos[0])), int(round(pose_pos[1])));
	//get_current_pose();

	while(!queue_m.empty()){
		pair<int, int> p = queue_m.front();
		queue_m.pop();
		
		if(marker_list[p.second][p.first]==MAP_CLOSE_LIST){
			continue; 	
		} else if(is_a_frontier_point(p)){
			queue<pair<int, int>> queue_f;
			vector<pair<int, int>> new_frontier;
			queue_f.push(p);
			marker_list[p.second][p.first] = FRONTIER_OPEN_LIST;

			while(!queue_f.empty()){
				pair<int, int> q = queue_f.front();
				queue_f.pop();
				if(marker_list[q.second][q.first] == MAP_CLOSE_LIST || 
					marker_list[q.second][q.first] == FRONTIER_CLOSE_LIST){
					continue;
				}
				else if(is_a_frontier_point(q)){
					new_frontier.push_back(q);
					for(auto adj : findNeighbours(q)){
						if(	marker_list[adj.second][adj.first] == FRONTIER_OPEN_LIST ||
							marker_list[adj.second][adj.first] == FRONTIER_CLOSE_LIST ||
							marker_list[adj.second][adj.first] == MAP_CLOSE_LIST){
							continue;					
						}
						queue_f.push(adj);
						marker_list[adj.second][adj.first] = FRONTIER_OPEN_LIST;
					} 
					marker_list[q.second][q.first] = FRONTIER_CLOSE_LIST;
				}						
			}
			list_of_frontiers.push_back(new_frontier); //Optimize this
			for(pair<int,int> mem : new_frontier){
				marker_list[mem.second][mem.first] = MAP_CLOSE_LIST;		
			}
		}

		for(pair<int,int> v : findNeighbours(p)){
			if(marker_list[v.second][v.first]==MAP_CLOSE_LIST || marker_list[v.second][v.first]==MAP_OPEN_LIST)
			{
				continue;
			}
			if(has_open_neighbour(marker_list,v)){
				queue_m.push(v);
				marker_list[v.second][v.first] = MAP_OPEN_LIST;
			}
		}
		marker_list[p.second][p.first] = MAP_CLOSE_LIST;	
	}
	
	return list_of_frontiers;
}
	

/*

int main(){
	return 1;
}*/