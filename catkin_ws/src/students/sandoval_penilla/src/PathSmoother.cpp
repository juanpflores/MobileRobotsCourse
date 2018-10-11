#include "PathSmoother.h"

nav_msgs::OccupancyGrid PathSmoother::InflateObstacles(nav_msgs::OccupancyGrid& map,  float inflation_radius)
{
    // Creamos una copia del mapa para poder modificarlo y despues regresarlo
    nav_msgs::OccupancyGrid inflated = map;
    // El radio que se introduce se transforma a celdas
    int inflation_cells = (int)(inflation_radius / map.info.resolution);
    // Calculamos el total de celdas de nuestra matriz
    int kernel_size     = (2*inflation_cells + 1) * (2*inflation_cells + 1);
    // Creamos un vector del tamano de nuestra matriz, los cuales son los vecinos
    std::vector<int> neighbors;
    neighbors.resize(kernel_size);
    
    // Generamos un conjunto de offsets
    for(int i=-inflation_cells, counter = 0; i<=inflation_cells; i++)
        // Lo almacenamos de forma lineal
        for(int j=-inflation_cells; j<=inflation_cells; j++, counter++)
            // Transformamos el arreglo bidimensional a lineal
            neighbors[counter] = j*map.info.width + i;
    
     
    for(size_t i=0; i < map.data.size(); i++)
	// Si en mapa se encuentra una celda ocupada, al indice se le suman los offset
	// y tambien van ser celdas ocupadas
	if(map.data[i] > 40)
	    for(size_t j=0; j < neighbors.size(); j++)
		inflated.data[i + neighbors[j]] = 100;
    // Regresamos el mapa ya modificado
    return inflated;
}

nav_msgs::OccupancyGrid PathSmoother::GetNearnessMap(nav_msgs::OccupancyGrid& map, float nearness_radius)
{
    if(nearness_radius <= 0)
	return map;	
    nav_msgs::OccupancyGrid nearnessMap = map;
    
  
     /* Implement the algorithm to get the nearness map. For example, if the map is: ('x' indicates an occupied cell)
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence (nearness radius)
    */

    //return nearnessMap;

    int pasos      = (int)(nearness_radius / map.info.resolution);
    int cuadricula = (pasos*2 + 1)*(pasos*2 + 1);
    int* distancia = new int[cuadricula];
    int* vecinos   = new int[cuadricula];

    int count = 0;
    for(int i=-pasos; i<=pasos; i++){
    // Al igual que cuando inflamos el mapa, generamos los indices
        for(int j=-pasos; j<=pasos; j++){
            vecinos[count] = i * map.info.width + j;
            // Para cada uno de los offsets, asignamos un valor de cercania,
            // El cual se calcula de la sig manera
            distancia[count] = pasos - std::max(1 + std::abs(i), std::abs(j));
            count++;
            
        }
    }
    for(int i=0; i<map.data.size(); i++){
        if(map.data[i] > 40){
        // Si esta ocupada la celda, para todos los vecinos
            for(int j=0; j<cuadricula; j++){
                if(nearnessMap.data[i+vecinos[j]] < distancia[j])
                // Si la cercania que calculo, es mayor a la que ya tenia la celda
                // cambiamos el valor
                nearnessMap.data[i+vecinos[j]] = distancia[j];
            }
        }
    }
    //delete[] distancia;
    //delete[] vecinos;
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
     
     // Definimos una tolerancia y la multicamos por path.poses.size( para que sea
     // porporcional al tamano
     float tol           = 0.00001 * path.poses.size();
     // Definimos un numero maximo de iteraciones para no quedar en un bucle infinito
     int intentos        = 10000;
     // Calculamos la manitud del gradiente
     float magnitud_grad = tol + 1;
     float diff          = 0.5;
     
     while(magnitud_grad >= tol && --intentos > 0){
        magnitud_grad = 0;
        for(int i=1; i<path.poses.size()-1;i++){
            // Calculos de las coord xoI, yoI de la trayectoria Original
            float xoI  = path.poses[i].pose.position.x;
            float yoI  = path.poses[i].pose.position.y;
            // Calculos de las coord xoI, yoI de la trayectoria Nueva
            float xnI  = newPath.poses[i].pose.position.x;
            float ynI  = newPath.poses[i].pose.position.y;
            float xnIp = newPath.poses[i-1].pose.position.x;
            float ynIp = newPath.poses[i-1].pose.position.y;
            float xnIn = newPath.poses[i+1].pose.position.x;
            float ynIn = newPath.poses[i+1].pose.position.y;
            // Calculamos el gradiente
            float gradX= beta*(xnI-xoI) + alpha*(2*xnI - xnIp - xnIn);
            float gradY= beta*(ynI-yoI) + alpha*(2*ynI - ynIp - ynIn);
            // Algoritmo del decenso del gradiente, el valor siguiente se calcula como 
            // actual menos una pequena cantidad en direccion del gradiente
            newPath.poses[i].pose.position.x = newPath.poses[i].pose.position.x - diff*gradX;
            newPath.poses[i].pose.position.y = newPath.poses[i].pose.position.y - diff*gradY;
            // Calculamos la magnitud del gradiente empleando norma uno
            magnitud_grad += fabs(gradX) + fabs(gradY);
        }
     }
    return newPath;
}
