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
    int step = (int) (nearness_radius/map.info.resolution);
    int boxsize = (step*2 + 1) * (step*2 + 1);
    std::vector<int> distances;
    std::vector<int> neighbors;
    distances.resize(boxsize); 
    neighbors.resize(boxsize); 
    int counter =  0;
   for(int i = -step; i<=step; i++) //Se calcula el valor que se asiganará a la celda dependiendo de su posición
        for (int j = -step; j <= step; j++){
            neighbors[counter] = i*map.info.width + j;
            distances[counter] = (step - std::max(std::abs(i), std::abs(j)) - 1);
            counter++;
        }
    for(int i = 0; i < map.data.size(); i++){
        if(map.data[i] > 40)
            for(int j = 0; j < boxsize; j++)
                if(nearnessMap.data[i + neighbors[j]] < distances[j])
                    nearnessMap.data[i + neighbors[j]] = distances[j];
    } // Si hay celdas con valores menores entre los vecinos y distancias, se intercambian
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
    float tol =  0.00001 * path.poses.size(); // Valor de tolerancia
    int pasos = 10000; // Límite de pasos
    float grad_v = tol + 1; // VAlor inicial del gradiente
    float delta  =  0.45; // Delta estimada
    
	while (grad_v >= tol && --pasos > 0){
        	grad_v = 0;
		// Cálculo de los puntos iniciales
		float xn0 = newPath.poses[0].pose.position.x;
	        float yn0 = newPath.poses[0].pose.position.y;
       		float xn1 = newPath.poses[1].pose.position.x;
        	float yn1 = newPath.poses[1].pose.position.y;
        	float x00 = path.poses[0].pose.position.x;
        	float y00 = path.poses[0].pose.position.x;
        	float grad_x = alpha*(xn0 - xn1) + beta*(xn0 - x00);
		float grad_y = alpha*(yn0 - yn1) + beta*(yn0 - y00);

	    // Posicion inicial de la nueva ruta
        newPath.poses[0].pose.position.x = xn0 - delta*grad_x;
        newPath.poses[0].pose.position.y = yn0 - delta*grad_y;

        // Acomulado del gradiente
        grad_v += fabs(grad_x) + fabs(grad_y);
	
	    for(int i = 1; i < path.poses.size() - 1; i++){
            // Càlculo de los puntos  [1 - k-1]
            float x0i  = path.poses[i].pose.position.x;
            float y0i  = path.poses[i].pose.position.y;
            float xni  = newPath.poses[i].pose.position.x;
            float yni  = newPath.poses[i].pose.position.y;
            float xnil = newPath.poses[i-1].pose.position.x;
            float ynil = newPath.poses[i-1].pose.position.y;
            float xnip = newPath.poses[i+1].pose.position.x;
            float ynip = newPath.poses[i+1].pose.position.y;

            grad_x = alpha*(2*xni - xnil - xnip) + beta*(xni - x0i);
            grad_y = alpha*(2*yni - ynil - ynip) + beta*(yni - y0i);
         
            // Se agrega posicion i-esima de la nueva ruta
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - delta*grad_x;
            newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - delta*grad_y;

            // Acomulado del gradiente 
            grad_v += fabs(grad_x) + fabs(grad_y);
        }

        // Cálculo de los últimos valores
        grad_x = alpha*(newPath.poses[path.poses.size()-1].pose.position.x - newPath.poses[path.poses.size()-2].pose.position.x) 
                 + beta*(newPath.poses[path.poses.size()-1].pose.position.x - path.poses[path.poses.size()-1].pose.position.x);

        grad_y = alpha*(newPath.poses[path.poses.size()-1].pose.position.y - newPath.poses[path.poses.size()-2].pose.position.y)
                 + beta*(newPath.poses[path.poses.size()-1].pose.position.y - path.poses[path.poses.size()-1].pose.position.y);
        
        //Se agrega posicion k de la nueva ruta
        newPath.poses[path.poses.size()].pose.position.x = newPath.poses[path.poses.size()-1].pose.position.x - delta*grad_x;
        newPath.poses[path.poses.size()].pose.position.y = newPath.poses[path.poses.size()-1].pose.position.y - delta*grad_y;
       
        // Acomulado del gradiente mediate X y Y
	    grad_v += fabs(grad_x) + fabs(grad_y);
	}
    
    return newPath;
}
