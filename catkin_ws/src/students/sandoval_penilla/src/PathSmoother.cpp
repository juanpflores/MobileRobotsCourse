#include "PathSmoother.h"

nav_msgs::OccupancyGrid PathSmoother::InflateObstacles(nav_msgs::OccupancyGrid& map,  float inflation_radius)
{
    nav_msgs::OccupancyGrid inflated = map;
    int inflation_cells = (int)(inflation_radius / map.info.resolution);
    int kernel_size     = (2*inflation_cells + 1) * (2*inflation_cells + 1);
    std::vector<int> neighbors;
    neighbors.resize(kernel_size);

    for(int i=-inflation_cells, counter = 0; i<=inflation_cells; i++)
        for(int j=-inflation_cells; j<=inflation_cells; j++, counter++)
            neighbors[counter] = j*map.info.width + i;

    for(size_t i=0; i < map.data.size(); i++)
	if(map.data[i] > 40)
	    for(size_t j=0; j < neighbors.size(); j++)
		inflated.data[i + neighbors[j]] = 100;
    
    return inflated;
}

nav_msgs::OccupancyGrid PathSmoother::GetNearnessMap(nav_msgs::OccupancyGrid& map, float nearness_radius)
{
    if(nearness_radius <= 0)
	return map;	
    nav_msgs::OccupancyGrid nearnessMap = map;
    
    /* TODO:
     * Implement the algorithm to get the nearness map. For example, if the map is: ('x' indicates an occupied cell)
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence (nearness radius)
    */

    return nearnessMap;
}

nav_msgs::Path PathSmoother::SmoothPath(nav_msgs::Path& path, float alpha, float beta)
{
    nav_msgs::Path newPath = path;    
    if(path.poses.size() < 3)
        return newPath;

    /* TODO:
     * Implement the algorithm to smooth a path by the gradient descend method.
     * Parameters alpha and beta are set using the GUI
     * You must find the appropriate values for the rest of parameters.
     */
    
    return newPath;
}
