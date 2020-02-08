#include <visualization_msgs/Marker.h>

void generateMarkers(ros::Publisher marker_pub, 
    std::vector<std::pair<double, double>> list_of_medians);