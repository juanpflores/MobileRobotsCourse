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
     * Implement the alorithm to get the nearness map. For example, if the map is: ('x' indicates an occupied cell)
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence (nearness radius)
    */
    
    int celdas= (nearness_radius/ map.info.resolution);
    int num_casilla=(celdas*2 +1)* (celdas*2 +1);
    int* distances = new int[num_casilla];
    int* neighbors = new int[num_casilla];

    int contando =0;
    for( int i=-celdas; i<=celdas; i++)
      for(int j=-celdas; j<=celdas; j++)
	{
	  neighbors[contando] = i*map.info.width+j;
	  distances[contando] = (celdas - std::max(std::abs(i) , std::abs(j)) +1);
	  contando++;

	}
    for(int i=0; i< map.data.size(); i++)
      if(map.data[i] >40)
	for(int j=0; j<num_casilla; j++)
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

    /* TODO:
     * Implement the algorithm to smooth a path by the gradient descend method.
     * Parameters alpha and beta are set using the GUI
     * You must find the appropriate values for the rest of parameters.
     */
    
    float tolerancia =0.0001 * path.poses.size();
    float intentos  =10000;
    float epsi_i    =tolerancia+1; //La suma del gradiente de todos los puntos desde 1 hasta k-1
    float epsi_0    =tolerancia+1; //El gradiente de 0
    float epsi_k    =tolerancia+1; //El gradiente en k
    float epsi      =tolerancia+1; //La suma de los tres gradientes
    float delta     =0.3;

    //Se coloca esta condición porque queremos que se calcule dentro del rango de puntos, además de que se coloca un limite de interaciones 
    
    while(epsi_i >= tolerancia && --intentos>0) 
      {
	epsi_i=0;
	    //Para calcular los primeros puntos de la función del gradiente        
	    float xo_o = path.poses[0].pose.position.x;
	    float yo_o = path.poses[0].pose.position.y ;	    
	    float xo_n = newPath.poses[0].pose.position.x;
	    float yo_n = newPath.poses[0].pose.position.y;
	    float x1_n = newPath.poses[1].pose.position.x;
	    float y1_n = newPath.poses[1].pose.position.y;
	    float grad_x0=delta*(alpha*(xo_n-x1_n)+beta*(xo_n-xo_o));
	    float grad_y0=delta*(alpha*(yo_n-y1_n)+beta*(yo_n-yo_o));
	    newPath.poses[0].pose.position.x =newPath.poses[0].pose.position.x- grad_x0;
	    newPath.poses[0].pose.position.y =newPath.poses[0].pose.position.y -grad_y0;
				
	    float epsi_0=fabs(grad_x0) + fabs(grad_y0);
	    

	    //Para calcular los puntos que van desde 1 hasta k                 
	    for(int i=1; i< path.poses.size() -1; i++)
	      {	
	    float xn_i = newPath.poses[i].pose.position.x;
	    float yn_i = newPath.poses[i].pose.position.y;
	    float xo_i = path.poses[i].pose.position.x;
	    float yo_i = path.poses[i].pose.position.y;
	    float xn_ip= newPath.poses[i-1].pose.position.x;
	    float yn_ip= newPath.poses[i-1].pose.position.y;
	    float xn_in= newPath.poses[i+1].pose.position.x;
	    float yn_in= newPath.poses[i+1].pose.position.y;
	    float grad_xi = beta*(xn_i - xo_i) + alpha*(2*xn_i - xn_ip - xn_in);
	    float grad_yi = beta*(yn_i - yo_i) + alpha*(2*yn_i - yn_ip - yn_in);
            //Asignando valores para la nueva ruta
	    newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x -delta*grad_xi;
	    newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y -delta*grad_yi;
            //Realizando la suma de todos los gradientes en valor absoluto
	    epsi_i += fabs(grad_xi) + fabs(grad_yi);
     	      }
	    
	    //Para calcular los ultimos  puntos de la ruta de la  función del gradiente
	    float xk_o = path.poses[ path.poses.size() ].pose.position.x;
	    float yk_o = path.poses[ path.poses.size() ].pose.position.y ;
	    float xk_n = newPath.poses[ path.poses.size()].pose.position.x;
	    float yk_n = newPath.poses[ path.poses.size()].pose.position.y;
	    float xk1_n = newPath.poses[ path.poses.size()-1].pose.position.x;
	    float yk1_n = newPath.poses[ path.poses.size()-1].pose.position.y;
	    float grad_xk=delta*(alpha*(xk_n-xk1_n)+beta*(xk_n-xk_o));
	    float grad_yk=delta*(alpha*(yk_n-yk1_n)+beta*(yk_n-yk_o));
	    float epsi_k=fabs(grad_xk) + fabs(grad_yk);

	    newPath.poses[ path.poses.size()].pose.position.x =newPath.poses[ path.poses.size()].pose.position.x- grad_xk;
	    newPath.poses[ path.poses.size()].pose.position.y =newPath.poses[ path.poses.size()].pose.position.y -grad_yk;
	    epsi=epsi_0+epsi_i+epsi_k;

      }
}
