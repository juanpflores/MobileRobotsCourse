#include "PathSmoother.h"

nav_msgs::OccupancyGrid PathSmoother::InflateObstacles(nav_msgs::OccupancyGrid& map,  float inflation_radius)
{
  nav_msgs::OccupancyGrid inflated = map;

  //Obtaining the number of cells inflated from the inflation radius and
  //establishing the kernel size which will contain the offsets
  int inflation_cells = (int)(inflation_radius / map.info.resolution);
  int kernel_size     = (2*inflation_cells + 1) * (2*inflation_cells + 1);
  std::vector<int> neighbors;
  neighbors.resize(kernel_size);
  
  //Assigning two-dimensional indexes to the neighbors of the central element of the kernel 
  for(int i=-inflation_cells, counter = 0; i<=inflation_cells; i++)
    for(int j=-inflation_cells; j<=inflation_cells; j++, counter++)
      neighbors[counter] = j*map.info.width + i;

  //Assigning an occupancy level of 1 to the neighbors when placing the central element in an obstacle
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

  //Obtaining the number of nearness layers from the nearness radius and
  //establishing the matrix size which will contain the nearness values
  int surrounding_layers = (int)(nearness_radius / map.info.resolution);
  int matrix_size        = (2*surrounding_layers + 1) * (2*surrounding_layers + 1);
  std::vector<int> neighbors;
  std::vector<int> distances;
  neighbors.resize(matrix_size);
  distances.resize(matrix_size);

  
  //Assigning two-dimensional indexes to the neighbors of the central element of the matrix, as well as their proximity value 
  int counter = 0;
  for(int i=-surrounding_layers; i<=surrounding_layers; i++)
    for(int j=-surrounding_layers; j<=surrounding_layers; j++)
      {
	neighbors[counter] = i*map.info.width +j;
	distances[counter] = (surrounding_layers - std::max(std::abs(i), std::abs(j)) + 1);
	counter++;
      }
  
  //Assigning the nearness value to the cell when it is greater than the value the cell already has
  for(int i=0; i < map.data.size(); i++)
    if(map.data[i] > 40)
      for(int j=0; j < matrix_size; j++)
	if(nearnessMap.data[i+neighbors[j]] < distances[j])
	  nearnessMap.data[i+neighbors[j]] = distances[j];
  

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

  float tolerance = 0.00001 * path.poses.size();
  float attempts  = 10000;
  float grad_mag  = tolerance + 1;
  float delta     = 0.5;

  while(grad_mag >= tolerance && --attempts > 0)
    {
      grad_mag = 0;
      for(int i=1; i<path.poses.size()-1; i++)
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
