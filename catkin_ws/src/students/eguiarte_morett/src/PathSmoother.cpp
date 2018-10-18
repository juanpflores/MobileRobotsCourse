
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
    /* 
     * Implement the algorithm to get the nearness map. For example, if the map is: ('x' indicates an occupied cell)
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence (nearness radius)
    */
    int steps = (int) (nearness_radius/map.info.resolution);//amount of steps given the nearness window
    int boxSize = (steps*2 + 1) * (steps*2 + 1);//size of the nearness window in cells
    std::vector<int> distances;
    std::vector<int> neighbors;
    distances.resize(boxSize); //
    neighbors.resize(boxSize); //Stores the stride correspondant for each cell in meters
    int counter =  0;
   for(int i = -steps; i<=steps; i++)
        for (int j = -steps; j <= steps; j++){
            neighbors[counter] = i*map.info.width + j;
            distances[counter] = (steps - std::max(std::abs(i), std::abs(j)) - 1);
            counter++;
        }
    for(int i = 0; i < map.data.size(); i++){
        if(map.data[i] > 40)
            for(int j = 0; j < boxSize; j++)
                if(nearnessMap.data[i + neighbors[j]] < distances[j])
                    nearnessMap.data[i + neighbors[j]] = distances[j];
    }
    return nearnessMap;
}

nav_msgs::Path PathSmoother::SmoothPath(nav_msgs::Path& path, float alpha, float beta)
{
    /* 
     * Implement the algorithm to smooth a path by the gradient descend method.
     * The gradient is the derivative of the cost function as defined in class
     * we want to get to the global minimum of this function (the fact that it is
     * quadratic assures this happens)
     * the basic algorithm is:
     * while (|grad(U)| > tolerance){
     *      q <-- q - delta*grad(U)
     * }
     * Parameters alpha and beta are set using the GUI
     * You must find the appropriate values for the rest of parameters.
     */
    nav_msgs::Path newPath = path;    
    if(path.poses.size() < 3)
        return newPath;
    float tolerance =  0.00001 * path.poses.size();//tolerance for convergence of the algorithm
    int attempts = 10000;//If the algorithm doesn't converge after this amount of steps, stop it
    float grad_mag = tolerance + 1;//initialize the gradient for the main loop
    float delta  =  0.5;//learning rate, convergence depends on this
    
    while (grad_mag >= tolerance && --attempts > 0){
        grad_mag = 0;
        //Se comentan los cálculos de los indices cero y último puesto que de otra forma se queda oscilando
        float grad_x; //= alpha*(newPath.poses[0].pose.position.x - newPath.poses[1].pose.position.x) + beta*(newPath.poses[0].pose.position.x - path.poses[0].pose.position.x);
        float grad_y; //= alpha*(newPath.poses[0].pose.position.y - newPath.poses[1].pose.position.y) + beta*(newPath.poses[0].pose.position.y - path.poses[0].pose.position.y);
        //newPath.poses[0].pose.position.x = newPath.poses[0].pose.position.x - delta*grad_x;
        //newPath.poses[0].pose.position.y = newPath.poses[0].pose.position.y - delta*grad_y;
        grad_mag += fabs(grad_x) + fabs(grad_y);
        for(int i = 1; i < path.poses.size() - 1; i++){
            grad_x = alpha*(2*newPath.poses[i].pose.position.x - newPath.poses[i-1].pose.position.x - newPath.poses[i+1].pose.position.x) + beta*(newPath.poses[i].pose.position.x - path.poses[i].pose.position.x);
            grad_y = alpha*(2*newPath.poses[i].pose.position.y - newPath.poses[i-1].pose.position.y - newPath.poses[i+1].pose.position.y) + beta*(newPath.poses[i].pose.position.y - path.poses[i].pose.position.y);
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - delta*grad_x;
            newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - delta*grad_y;

            grad_mag += fabs(grad_x) + fabs(grad_y);
        }
        //grad_x = alpha*(newPath.poses[path.poses.size()-1].pose.position.x - newPath.poses[path.poses.size()-2].pose.position.x) + beta*(newPath.poses[path.poses.size()-1].pose.position.x - path.poses[path.poses.size()-1].pose.position.x);
        //grad_y = alpha*(newPath.poses[path.poses.size()-1].pose.position.y - newPath.poses[path.poses.size()-2].pose.position.y) + beta*(newPath.poses[path.poses.size()-1].pose.position.y - path.poses[path.poses.size()-1].pose.position.y);
        //newPath.poses[path.poses.size()-1].pose.position.x = newPath.poses[path.poses.size()-1].pose.position.x - delta*grad_x;
        //newPath.poses[path.poses.size()-1].pose.position.y = newPath.poses[path.poses.size()-1].pose.position.y - delta*grad_y;
        //grad_mag += fabs(grad_x) + fabs(grad_y);
    }
    
    return newPath;
}
