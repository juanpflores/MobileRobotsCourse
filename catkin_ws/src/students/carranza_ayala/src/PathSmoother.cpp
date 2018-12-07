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
    //This function calculates the "nearness to obstacles", e.g., for the following grid:
    /*
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence (nearness radius)
    */
    if(nearness_radius <= 0)
	return map;
	
    nav_msgs::OccupancyGrid nearnessMap = map;
    int steps = (int)(nearness_radius / map.info.resolution);
    
    int boxSize = (steps*2 + 1) * (steps*2 + 1);
    int* distances = new int[boxSize];
    int* neighbors = new int[boxSize];

    int counter = 0;
    for(int i=-steps; i<=steps; i++)
        for(int j=-steps; j<=steps; j++)
        {
            neighbors[counter] = i*map.info.width + j;
            distances[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1);
            counter++;
        }
    
    for(int i=0; i < map.data.size(); i++)
        if(map.data[i] > 40)
            for(int j = 0; j < boxSize; j++)
                if(nearnessMap.data[i+neighbors[j]] < distances[j])
                    nearnessMap.data[i+neighbors[j]] = distances[j];

    delete[] distances;
    delete[] neighbors;
    return nearnessMap;
}

nav_msgs::Path PathSmoother::SmoothPath(nav_msgs::Path& path, float alpha, float beta)
{
    nav_msgs::Path newPath = path;
    
    if(path.poses.size() < 3)
        return newPath;

    float tolerance = 0.00001 * path.poses.size();
    int   attempts  = 10000;
    float grad_mag  = tolerance + 1;
    float delta     = 0.5;
    while(grad_mag >= tolerance && --attempts > 0)
    {
        grad_mag = 0;
        for(int i=1; i< path.poses.size() - 1; i++)
        {
	    float xo_i  = path.poses[i].pose.position.x;
	    float yo_i  = path.poses[i].pose.position.y;
	    float xn_i  = newPath.poses[i].pose.position.x;
	    float yn_i  = newPath.poses[i].pose.position.y;
	    float xn_ip = newPath.poses[i-1].pose.position.x;
	    float yn_ip = newPath.poses[i-1].pose.position.y;
	    float xn_in = newPath.poses[i+1].pose.position.x;
	    float yn_in = newPath.poses[i+1].pose.position.y;
	    float grad_x = beta*(xn_i - xo_i) + alpha*(2*xn_i - xn_ip - xn_in);
	    float grad_y = beta*(yn_i - yo_i) + alpha*(2*yn_i - yn_ip - yn_in);
	    
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - delta*grad_x;
	    newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - delta*grad_y;

	    grad_mag += fabs(grad_x) + fabs(grad_y);
        }
    }
    return newPath;
}
