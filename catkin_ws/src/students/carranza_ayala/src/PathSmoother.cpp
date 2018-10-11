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
    int steps = (int)(nearness_radius / map.info.resolution); 
    int boxSize = (steps * 2 + 1) * (steps * 2 + 1);          
    std::vector<int> distances;
    std::vector<int> neighbors;
    distances.resize(boxSize);
    neighbors.resize(boxSize);
    int counter = 0;
    for (int i = -steps; i <= steps; i++)
        for (int j = -steps; j <= steps; j++)
        {
            neighbors[counter] = i * map.info.width + j;
            distances[counter] = (steps - std::max(std::abs(i), std::abs(j)) - 1);
            counter++;
        }

    for (int i = 0; i < map.data.size(); i++)
        if (map.data[i] > 40)
            for (int j = 0; j < boxSize; j++)
                if (nearnessMap.data[i + neighbors[j]] < distances[j])
                    nearnessMap.data[i + neighbors[j]] = distances[j];
    
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

    int attempts    = 10000;                       //Max number of steps the algorithm goes if it doesn't converge
    float tolerance = 0.00001 * path.poses.size(); //Tolerance for convergence
    float grad_mag  = tolerance + 1;               //Initialize the value of the gradient
    float delta     = 0.5;                         //Helps convergence

    while (grad_mag >= tolerance && --attempts > 0)
    {
        grad_mag = 0;
        //Calculate initial points necessary for the first gradient
        float xn_0 = newPath.poses[0].pose.position.x;
        float yn_0 = newPath.poses[0].pose.position.y;
        float xn_1 = newPath.poses[1].pose.position.x;
        float yn_1 = newPath.poses[1].pose.position.y;
        float xo_0 = path.poses[0].pose.position.x;
        float yo_0 = path.poses[0].pose.position.x;
        //Calculate gradient
        float grad_x = alpha * (xn_0 - xn_1) + beta * (xn_0 - xo_0);
        float grad_y = alpha * (yn_0 - yn_1) + beta * (yn_0 - yo_0);
        //Add the first position of the new route
        newPath.poses[0].pose.position.x = xn_0 - delta * grad_x;
        newPath.poses[0].pose.position.y = yn_0 - delta * grad_y;
        //Calculate the gradient magnitude
        grad_mag += fabs(grad_x) + fabs(grad_y);

        //Calculate from the next point up to the second to last point
        for (int i = 1; i < path.poses.size() - 1; i++)
        {
            float xo_i = path.poses[i].pose.position.x;
            float yo_i = path.poses[i].pose.position.y;
            float xn_i = newPath.poses[i].pose.position.x;
            float yn_i = newPath.poses[i].pose.position.y;
            float xn_ip = newPath.poses[i - 1].pose.position.x;
            float yn_ip = newPath.poses[i - 1].pose.position.y;
            float xn_in = newPath.poses[i + 1].pose.position.x;
            float yn_in = newPath.poses[i + 1].pose.position.y;
            //Calculate gradient
            grad_x = alpha * (2 * xn_i - xn_ip - xn_in) + beta * (xn_i - xo_i);
            grad_y = alpha * (2 * yn_i - yn_ip - yn_in) + beta * (yn_i - yo_i);
            //Add position to route
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - delta * grad_x;
            newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - delta * grad_y;
            //Calculate the gradient magnitude
            grad_mag += fabs(grad_x) + fabs(grad_y);
        }
        //Calculate for the last position
        grad_x = alpha * (newPath.poses[path.poses.size() - 1].pose.position.x - newPath.poses[path.poses.size() - 2].pose.position.x) + beta * (newPath.poses[path.poses.size() - 1].pose.position.x - path.poses[path.poses.size() - 1].pose.position.x);
        grad_y = alpha * (newPath.poses[path.poses.size() - 1].pose.position.y - newPath.poses[path.poses.size() - 2].pose.position.y) + beta * (newPath.poses[path.poses.size() - 1].pose.position.y - path.poses[path.poses.size() - 1].pose.position.y);
        //Add last position to route
        newPath.poses[path.poses.size()].pose.position.x = newPath.poses[path.poses.size() - 1].pose.position.x - delta * grad_x;
        newPath.poses[path.poses.size()].pose.position.y = newPath.poses[path.poses.size() - 1].pose.position.y - delta * grad_y;
        //Calculate the gradient magnitude
        grad_mag += fabs(grad_x) + fabs(grad_y);
    }

    return newPath;
}
