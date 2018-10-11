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
    int steps=(nearness_radius / map.info.resolution);
    int boxSize= (steps*2 +1) * (steps*2 +1);
    int* distances = new int[boxSize];
    int* neighbors = new int[boxSize];
    int counter =0;

    for(int i=-steps; i<=steps; i++)
      for(int j=-steps; j<=steps; j++){
        neighbors[counter] = i*map.info.width +j;
        distances[counter] = (steps -std:: max(std::abs(i), std::abs(j))+1);
        counter++;
      }
    for(int i=0; i< map.data.size(); i++)
      if(map.data[i] >40)
        for(int j=0; j < boxSize; j++)
          if(nearnessMap.data[i+neighbors[j]] < distances[j])
            nearnessMap.data[i+neighbors[j]]=distances[j];
        
    delete[] distances;
    delete[] neighbors;

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

    float tolerance =  0.00001 * path.poses.size();//tolerancia para que el algoritmo converja
    int attempts = 10000;//si no converje se usa esta variable como maximo num. de pasos
    float grad_mag = tolerance + 1;//inicializacion del gradiente
    float delta  =  0.5;//var ayuda a que converja
    

    while (grad_mag >= tolerance && --attempts > 0){
        grad_mag = 0;
        //calculo de los puntos iniciales necesarios para el calulo del gradiente inicial
        float xn_0 = newPath.poses[0].pose.position.x;
        float yn_0 = newPath.poses[0].pose.position.y;
        float xn_1 = newPath.poses[1].pose.position.x;
        float yn_1 = newPath.poses[1].pose.position.y;
        float x0_0 = path.poses[0].pose.position.x;
        float y0_0 = path.poses[0].pose.position.x;
        float grad_x = alpha*(xn_0 - xn_1) + beta*(xn_0 - x0_0);
        float grad_y = alpha*(yn_0 - yn_1) + beta*(yn_0 - y0_0);
        //se agrega posicion inicial de la nueva ruta
        newPath.poses[0].pose.position.x = xn_0 - delta*grad_x;
        newPath.poses[0].pose.position.y = yn_0 - delta*grad_y;
        //calulo de la magnitud del gradiente mediante la magnitud de las componentes en X y Y
        grad_mag += fabs(grad_x) + fabs(grad_y);

        for(int i = 1; i < path.poses.size() - 1; i++){
            //calculo de los puntos de 1 hasta k-1
            float x0_i  = path.poses[i].pose.position.x;
            float y0_i  = path.poses[i].pose.position.y;
            float xn_i  = newPath.poses[i].pose.position.x;
            float yn_i  = newPath.poses[i].pose.position.y;
            float xn_il1 = newPath.poses[i-1].pose.position.x;
            float yn_il1 = newPath.poses[i-1].pose.position.y;
            float xn_ip1 = newPath.poses[i+1].pose.position.x;
            float yn_ip1 = newPath.poses[i+1].pose.position.y;

            grad_x = alpha*(2*xn_i - xn_il1 - xn_ip1) + beta*(xn_i - x0_i);
            grad_y = alpha*(2*yn_i - yn_il1 - yn_ip1) + beta*(yn_i - y0_i);
         
            //se agrega posicion i-esima de la nueva ruta
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - delta*grad_x;
            newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - delta*grad_y;
            //calulo de la magnitud del gradiente mediante la magnitud de las componentes en X y Y
            grad_mag += fabs(grad_x) + fabs(grad_y);
        }
        //calulo de los puntos k (ultimos)
        grad_x = alpha*(newPath.poses[path.poses.size()-1].pose.position.x - newPath.poses[path.poses.size()-2].pose.position.x) 
                 + beta*(newPath.poses[path.poses.size()-1].pose.position.x - path.poses[path.poses.size()-1].pose.position.x);

        grad_y = alpha*(newPath.poses[path.poses.size()-1].pose.position.y - newPath.poses[path.poses.size()-2].pose.position.y)
                 + beta*(newPath.poses[path.poses.size()-1].pose.position.y - path.poses[path.poses.size()-1].pose.position.y);
        //se agrega posicion k de la nueva ruta
        newPath.poses[path.poses.size()].pose.position.x = newPath.poses[path.poses.size()-1].pose.position.x - delta*grad_x;
        newPath.poses[path.poses.size()].pose.position.y = newPath.poses[path.poses.size()-1].pose.position.y - delta*grad_y;
       
        //calulo de la magnitud del gradiente mediante la magnitud de las componentes en X y Y
        grad_mag += fabs(grad_x) + fabs(grad_y);
    }
    
    return newPath;
}
