/* This file lists globals for use in other files
   Define your globals in contest1.cpp and include this where needed */

// Occupancy grid globals for storing callback data
extern std::vector<std::vector<int>> occ_grid;
extern int occ_width;
extern int occ_height;
extern float res;
extern float pose_pos[3];
extern float pose_orientation[4];
extern float pose_origin [3];
extern ros::NodeHandle nh;
