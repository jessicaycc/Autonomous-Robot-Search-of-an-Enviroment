#include "frontier_search.h"
#include "globals.h"

const int MEDIAN_SIZE_LIMITER = 10;
const bool print_times = false;

bool is_a_frontier_point(const pair<int,int> &p){
	if(occ_grid[p.first][p.second] == -1){
		return false;
	}
	vector<pair<int,int>> neighbours = findNeighbours(p);
	if(occ_grid[p.first][p.second] <= 10){
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
		std::cout << "Size of this median was: " << cur_front.size() << std::endl;
		if(cur_front.size() < MEDIAN_SIZE_LIMITER){
			continue;
		}
		sort(cur_front.begin(), cur_front.end());
		destinations.push_back(cur_front[cur_front.size()/2]);
	}
	

	return destinations; 
}

bool has_open_neighbour(const vector<vector<int>> &marker_list, const pair<int,int> &p){
	//this point has a neighbour that is map_open_list

	vector<pair<int,int>> neighbours = findNeighbours(p);
	for(pair<int,int> point: neighbours){
		if(marker_list[point.first][point.second]==MAP_OPEN_LIST)
		{
			return true;
		}

	}
	return false;
}
vector<pair<int,int>> findNeighbours(const pair<int,int> &p){
	//BFS find adj neighbours
	static vector<pair<int,int>> directions = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0}, {1,1}}; 
	//static vector<pair<int,int>> directions = {{-1,0},{0,-1},{0,1},{1,0}}; 
	vector<pair<int,int>> neighbours;
	
	for(auto d: directions){
		pair<int, int> temp = {p.first + d.first, p.second + d.second};
		if(temp.first < 0 || temp.second < 0 || 
			temp.first >= occ_height || temp.second >= occ_width || 
			occ_grid[temp.first][temp.second] >=10)
		{
			continue;
		}
		neighbours.push_back(temp);
	}
	return neighbours;
}

vector<pair<double,double>> wfd(tf::TransformListener &listener)
{
	//Store all new frontiers here
	
	//Get initial point
	

	ROS_INFO("Start frontier search: Initializing transform");

	//std::cout << "Start timing" << endl;
	//auto start = std::chrono::system_clock::now();
	

	listener.waitForTransform("/map", "/odom", ros::Time::now(), ros::Duration(1.0));
	ros::Time cur_time = ros::Time::now();
	listener.waitForTransform("/map", "/odom", cur_time, ros::Duration(10.0));

	
	//auto end = std::chrono::system_clock::now();    
	//auto elapsed =     std::chrono::duration_cast<std::chrono::seconds>(end - start);
	//std::cout << "Elapsed time" << elapsed.count() << '\n';

	//std::cout << "Start BFS timing" << endl;
	//start = std::chrono::system_clock::now();
	

	geometry_msgs::PointStamped cur_pose_stamped;
	cur_pose_stamped.header.frame_id = "/odom";
	cur_pose_stamped.header.stamp = cur_time;

	cur_pose_stamped.point.x = pose_pos[0];
	cur_pose_stamped.point.y = pose_pos[1];
	cur_pose_stamped.point.z = 0;

	


	//listener.waitForTransform("/map", "/odom", cur_time, ros::Duration(10.0));
	listener.transformPoint("/map", cur_pose_stamped, cur_pose_stamped);

	//ROS_INFO("%s", dest.header.frame_id.c_str());
	//ROS_INFO("CUR_POSE is (%.3f, %.3f, %.3f)", pose_pos[0], pose_pos[1], pose_pos[2]);
	


	pair<int, int> map_grid;
	map_grid.second = int(round((cur_pose_stamped.point.x - pose_origin[0])/res));
	map_grid.first = int(round((cur_pose_stamped.point.y - pose_origin[1])/res));




	//ROS_INFO("MAP_POSE is (%.3f, %.3f, %.3f)", cur_pose_stamped.point.x, cur_pose_stamped.point.y, cur_pose_stamped.point.z);
	//ROS_INFO("Grid_Point is (%.3f, %.3f)", map_grid.first, map_grid.second);

	//ROS_INFO("ORIGIN_POSE is (%.3f, %.3f, %.3f)", pose_origin[0], pose_origin[1], pose_origin[2]);

	




			
	vector<vector<pair<int, int>>> list_of_frontiers;
	std::cout<<"INITIALIZING WFD: OCC_GRID/HEIGHT" << occ_width <<"/" << occ_height << std::endl;

	//For keeping track of visited
	vector<vector<int>> marker_list(
		occ_height,
		vector<int>(occ_width, -1)
	); //y,x form (y rows of x length)

	//BFS queue 1
	queue<pair<int, int>> queue_m;
	int value_at_initial = occ_grid[map_grid.first][map_grid.second];
	//std::cout << "Value at initial" << value_at_initial << std::endl;
	//std::cout << "x/y: " <<map_grid.first << "/" << map_grid.second << endl;
	//pair<int, int> pose(int(round(cur_pose_stamped.point.x)), int(round(cur_pose_stamped.point.y)));
	//get_current_pose();
	queue_m.push(map_grid);
	marker_list[map_grid.first][map_grid.second] = MAP_OPEN_LIST;
	//std::cout << "PUSHING MAP GRID" << std::endl;
	while(!queue_m.empty()){
		pair<int, int> p = queue_m.front();
		queue_m.pop();
		
		//std::cout<<"MARKER LIST" << occ_width <<"/" << occ_height << std::endl;



		if(marker_list[p.first][p.second]==MAP_CLOSE_LIST){
			continue; 	
		} else if(is_a_frontier_point(p)){
			queue<pair<int, int>> queue_f;
			vector<pair<int, int>> new_frontier;
			queue_f.push(p);
			marker_list[p.first][p.second] = FRONTIER_OPEN_LIST;

			while(!queue_f.empty()){
				pair<int, int> q = queue_f.front();
				queue_f.pop();
				if(marker_list[q.first][q.second] == MAP_CLOSE_LIST || 
					marker_list[q.first][q.second] == FRONTIER_CLOSE_LIST){
					continue;
				}
				else if(is_a_frontier_point(q)){
					new_frontier.push_back(q);
					for(auto adj : findNeighbours(q)){
						if(	marker_list[adj.first][adj.second] == FRONTIER_OPEN_LIST ||
							marker_list[adj.first][adj.second] == FRONTIER_CLOSE_LIST ||
							marker_list[adj.first][adj.second] == MAP_CLOSE_LIST){
							continue;					
						}
						queue_f.push(adj);
						marker_list[adj.first][adj.second] = FRONTIER_OPEN_LIST;
					} 
					marker_list[q.first][q.second] = FRONTIER_CLOSE_LIST;
				}						
			}
			list_of_frontiers.push_back(new_frontier); //Optimize this
			
			for(pair<int,int> mem : new_frontier){
				marker_list[mem.first][mem.second] = MAP_CLOSE_LIST;		
			}
		}

		for(pair<int,int> v : findNeighbours(p)){
			if(marker_list[v.first][v.second]==MAP_CLOSE_LIST || marker_list[v.first][v.second]==MAP_OPEN_LIST)
			{
				continue;
			}
			if(has_open_neighbour(marker_list,v)){
				queue_m.push(v);
				marker_list[v.first][v.second] = MAP_OPEN_LIST;
			}
		}
		marker_list[p.first][p.second] = MAP_CLOSE_LIST;	
	}
	

	vector<pair<int,int>> list_of_medians = get_medians(list_of_frontiers);

	vector<pair<double, double>> list_of_medians_odom;
	pair<double, double> median_odom;

	//end = std::chrono::system_clock::now();    
	//elapsed =     std::chrono::duration_cast<std::chrono::seconds>(end - start);
    //std::cout << "Elapsed time for BFS: " << elapsed.count() << '\n';

	//start = std::chrono::system_clock::now();
	//listener.waitForTransform("/odom", "/map", cur_time, ros::Duration(10.0));


	/* Convert back to map frame */
	geometry_msgs::PointStamped cyc_pose_stamped;
	cyc_pose_stamped.header.frame_id = "/map";
	cyc_pose_stamped.header.stamp = cur_time;

	

	for(pair<int, int> median: list_of_medians){
		cyc_pose_stamped.point.x = median.second*res + pose_origin[0];
		cyc_pose_stamped.point.y = median.first*res + pose_origin[1];
		cyc_pose_stamped.point.z = 0;
		listener.transformPoint("/odom", cyc_pose_stamped, cyc_pose_stamped);
		median_odom.first = cyc_pose_stamped.point.x;
		median_odom.second = cyc_pose_stamped.point.y;
		list_of_medians_odom.push_back(median_odom);


	}
	/*
	end = std::chrono::system_clock::now();    
	elapsed =     std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Elapsed time to return medians: " << elapsed.count() << '\n';
	*/
	return list_of_medians_odom;
}
	

/*

int main(){
	return 1;
}*/

