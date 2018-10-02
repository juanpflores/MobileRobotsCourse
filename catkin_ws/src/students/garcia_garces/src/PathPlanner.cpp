#include "PathPlanner.h"

bool PathPlanner::BreadthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
			       nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{
    /*
     * Calculate the corresponding cell indices in the occupancy grid for the start and goal position.
     */
    int   idx_start;
    int   idx_goal;
    idx_start  = (int)((start_y - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_start += (int)((start_x - map.info.origin.position.x)/map.info.resolution);
    idx_goal   = (int)((goal_y  - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_goal  += (int)((goal_x  - map.info.origin.position.x)/map.info.resolution);

    /*
     * Variables:
     * 'nodes':          Array of nodes. One for each cell in the occupancy grid. 
     * 'current_node':   A pointer to the node selected from the open list according to the LEAST RECENT criterion.
     * 'runtime_steps':  An auxiliar counter for benchmarking purposes.
     * 'node_neighbors': Array for storing the indices of the neighbors of the current node.
     */
    std::vector<Node> nodes;
    Node* current_node; 
    int runtime_steps = 0;
    std::vector<int> node_neighbors;
    nodes.resize(map.data.size());
    node_neighbors.resize(4);
    
    /*
     * In the Breadth First Search algorithm, the open list is a QUEUE.
     */
    std::queue<Node*>   open_list;
    
    /*
     * THIS IS WHERE THE ALGORITHM BEGINS
     */
    /*
     * Initialization:
     * All distances are set to infinity (or in this case, to the maximum value).
     * All nodes are marked as not-belonging to the open nor closed list. 
     * After this, start node is added to the open list, marked as in-the-open-list and its distance is set to zero.
     */
    for(size_t i=0;  i < map.data.size(); i++)
    {
	nodes[i].index          = i;
	nodes[i].distance       = INT_MAX;
        nodes[i].in_open_list   = false;
        nodes[i].in_closed_list = false;
        nodes[i].parent         = NULL;
    }
    current_node = &nodes[idx_start];
    current_node->distance     = 0;
    current_node->in_open_list = true;    
    open_list.push(current_node);


    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the LEAST recent node from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    while(!open_list.empty() && current_node->index != idx_goal)
    {
	//Choose the current node with the criterion of the LEAST RECENT and add it to the closed list.
	current_node = open_list.front();  
	open_list.pop();                   
	current_node->in_closed_list = true;
	
	//Get the list of neighbors of the current node (using 4 connectivity).
	node_neighbors[0] = current_node->index + map.info.width;
	node_neighbors[1] = current_node->index + 1;
	node_neighbors[2] = current_node->index - map.info.width;
	node_neighbors[3] = current_node->index - 1;
	
	for(size_t i=0; i < node_neighbors.size(); i++)
	{
	    //If it is an occupied cell or it is in the closed list, ignore it.
	    if(map.data[node_neighbors[i]] > 40 || map.data[node_neighbors[i]] < 0 || nodes[node_neighbors[i]].in_closed_list)
		continue;
	    
	    //If the distance from the current node is less than the previously found distance, then change it,
	    //and set the current node as parent of this neighbor.
	    Node* neighbor = &nodes[node_neighbors[i]];
	    int dist = current_node->distance + 1;
	    if(dist < neighbor->distance)
	    {
		neighbor->distance = dist;
		neighbor->parent   = current_node;
	    }
	    //If it is not in the open list, add it.
	    if(!neighbor->in_open_list)
	    {
		neighbor->in_open_list = true;
		open_list.push(neighbor);
	    }
	    
	    //Increment just to have a measure of the running time.
	    runtime_steps++; 
	}
    }
    //Check if the path was found
    if(current_node->index != idx_goal)
	return false;
    /*
     * THIS IS WHERE THE ALGORITHM ENDS
    */

    std::cout << "Path found using BREADTH FIRST SEARCH after " << runtime_steps << " steps." << std::endl;
    /*
     * Transform all nodes (cells with a corresponding index) to metric coordinates and add them to the resulting path.
     */
    result.header.frame_id = "map";
    result.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while(current_node->parent != NULL)
    {
	p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
	p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
	result.poses.insert(result.poses.begin(), p);
	current_node = current_node->parent;
    }
    return true;
}

bool PathPlanner::DepthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
			     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{

  /*
     * Calculate the corresponding cell indices in the occupancy grid for the start and goal position.
     */
    int   idx_start;
    int   idx_goal;
    idx_start  = (int)((start_y - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_start += (int)((start_x - map.info.origin.position.x)/map.info.resolution);
    idx_goal   = (int)((goal_y  - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_goal  += (int)((goal_x  - map.info.origin.position.x)/map.info.resolution);

    /*
     * Variables:
     * 'nodes':          Array of nodes. One for each cell in the occupancy grid. 
     * 'current_node':   A pointer to the node selected from the open list according to the MOST RECENT criterion.
     * 'runtime_steps':  An auxiliar counter for benchmarking purposes.
     * 'node_neighbors': Array for storing the indices of the neighbors of the current node.
     */
    std::vector<Node> nodes;
    Node* current_node; 
    int runtime_steps = 0;
    std::vector<int> node_neighbors;
    nodes.resize(map.data.size());
    node_neighbors.resize(4);

    
    /*
     * In the Depth First Search algorithm, the open list is a STACK.
     */
    std::stack<Node*>   open_list;

    /*
     * THIS IS WHERE THE ALGORITHM BEGINS
     */
    /*
     * Initialization:
     * All distances are set to infinity (or in this case, to the maximum value).
     * All nodes are marked as not-belonging to the open nor closed list. 
     * After this, start node is added to the open list, marked as in-the-open-list and its distance is set to zero.
     */
    for(size_t i=0;  i < map.data.size(); i++)
    {
	nodes[i].index          = i;
	nodes[i].distance       = INT_MAX;
        nodes[i].in_open_list   = false;
        nodes[i].in_closed_list = false;
        nodes[i].parent         = NULL;
    }
    current_node = &nodes[idx_start];
    current_node->distance     = 0;
    current_node->in_open_list = true;    
    open_list.push(current_node);


    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the MOST recent node from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */

    while(!open_list.empty() && current_node->index != idx_goal)
    {
	//Choose the current node with the criterion of the MOST RECENT and add it to the closed list.
	current_node = open_list.top();  
	open_list.pop();                   
	current_node->in_closed_list = true;
	
	//Get the list of neighbors of the current node (using 4 connectivity).
	node_neighbors[0] = current_node->index + map.info.width;
	node_neighbors[1] = current_node->index + 1;
	node_neighbors[2] = current_node->index - map.info.width;
	node_neighbors[3] = current_node->index - 1;
	
	for(size_t i=0; i < node_neighbors.size(); i++)
	{
	    //If it is an occupied cell or it is in the closed list, ignore it.
	    if(map.data[node_neighbors[i]] > 40 || map.data[node_neighbors[i]] < 0 || nodes[node_neighbors[i]].in_closed_list)
		continue;
	    
	    //If the distance from the current node is less than the previously found distance, then change it,
	    //and set the current node as parent of this neighbor.
	    Node* neighbor = &nodes[node_neighbors[i]];
	    int dist = current_node->distance + 1;
	    if(dist < neighbor->distance)
	    {
		neighbor->distance = dist;
		neighbor->parent   = current_node;
	    }
	    //If it is not in the open list, add it.
	    if(!neighbor->in_open_list)
	    {
		neighbor->in_open_list = true;
		open_list.push(neighbor);
	    }
	    
	    //Increment just to have a measure of the running time.
	    runtime_steps++; 
	}
    }
    //Check if the path was found
    if(current_node->index != idx_goal)
	return false;
    /*
     * THIS IS WHERE THE ALGORITHM ENDS
    */

    std::cout << "Path found using DEPTH FIRST SEARCH after " << runtime_steps << " steps." << std::endl;
    /*
     * Transform all nodes (cells with a corresponding index) to metric coordinates and add them to the resulting path.
     */
    result.header.frame_id = "map";
    result.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while(current_node->parent != NULL)
    {
	p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
	p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
	result.poses.insert(result.poses.begin(), p);
	current_node = current_node->parent;
    }
    return true;
}

bool PathPlanner::Dijkstra(float start_x, float start_y, float goal_x, float goal_y,
		     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{
  /*
     * Calculate the corresponding cell indices in the occupancy grid for the start and goal position.
     */
    int   idx_start;
    int   idx_goal;
    idx_start  = (int)((start_y - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_start += (int)((start_x - map.info.origin.position.x)/map.info.resolution);
    idx_goal   = (int)((goal_y  - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_goal  += (int)((goal_x  - map.info.origin.position.x)/map.info.resolution);

    /*
     * Variables:
     * 'nodes':          Array of nodes. One for each cell in the occupancy grid. 
     * 'current_node':   A pointer to the node selected from the open list according to the MINIMUM DISTANCE criterion.
     * 'runtime_steps':  An auxiliar counter for benchmarking purposes.
     * 'node_neighbors': Array for storing the indices of the neighbors of the current node.
     */
    std::vector<Node> nodes;
    Node* current_node; 
    int runtime_steps = 0;
    std::vector<int> node_neighbors;
    nodes.resize(map.data.size());
    node_neighbors.resize(4);

  
    /*
     * Since the current node is selected according to the distances, a convenient data structure is the PRIORITY QUEUE
     * with the comparison function defined to compare to nodes by their distances.
     */
    std::priority_queue<Node*, std::vector<Node*>, CompareByDistance>   open_list;

    /*
     * THIS IS WHERE THE ALGORITHM BEGINS
     */
    /*
     * Initialization:
     * All distances are set to infinity (or in this case, to the maximum value).
     * All nodes are marked as not-belonging to the open nor closed list. 
     * After this, start node is added to the open list, marked as in-the-open-list and its distance is set to zero.
     */
    for(size_t i=0;  i < map.data.size(); i++)
    {
	nodes[i].index          = i;
	nodes[i].distance       = INT_MAX;
        nodes[i].in_open_list   = false;
        nodes[i].in_closed_list = false;
        nodes[i].parent         = NULL;
    }
    current_node = &nodes[idx_start];
    current_node->distance     = 0;
    current_node->in_open_list = true;    
    open_list.push(current_node);
    
    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the node with the MINIMUM DISTANCE from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */

    while(!open_list.empty() && current_node->index != idx_goal)
    {
	//Choose the current node with the criterion of the MINIMUM DISTANCE and add it to the closed list.
	current_node = open_list.top();  
	open_list.pop();                   
	current_node->in_closed_list = true;
	
	//Get the list of neighbors of the current node (using 4 connectivity).
	node_neighbors[0] = current_node->index + map.info.width;
	node_neighbors[1] = current_node->index + 1;
	node_neighbors[2] = current_node->index - map.info.width;
	node_neighbors[3] = current_node->index - 1;
	
	for(size_t i=0; i < node_neighbors.size(); i++)
	{
	    //If it is an occupied cell or it is in the closed list, ignore it.
	    if(map.data[node_neighbors[i]] > 40 || map.data[node_neighbors[i]] < 0 || nodes[node_neighbors[i]].in_closed_list)
		continue;
	    
	    //If the distance from the current node is less than the previously found distance, then change it,
	    //and set the current node as parent of this neighbor.
	    Node* neighbor = &nodes[node_neighbors[i]];
	    int dist = current_node->distance + 1;
	    if(dist < neighbor->distance)
	    {
		neighbor->distance = dist;
		neighbor->parent   = current_node;
	    }
	    //If it is not in the open list, add it.
	    if(!neighbor->in_open_list)
	    {
		neighbor->in_open_list = true;
		open_list.push(neighbor);
	    }
	    
	    //Increment just to have a measure of the running time.
	    runtime_steps++; 
	}
    }
    //Check if the path was found
    if(current_node->index != idx_goal)
	return false;
    /*
     * THIS IS WHERE THE ALGORITHM ENDS
    */

    std::cout << "Path found using DIJKSTRA after " << runtime_steps << " steps." << std::endl;
    /*
     * Transform all nodes (cells with a corresponding index) to metric coordinates and add them to the resulting path.
     */
    result.header.frame_id = "map";
    result.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while(current_node->parent != NULL)
    {
	p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
	p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
	result.poses.insert(result.poses.begin(), p);
	current_node = current_node->parent;
    }
    return true;
}

bool PathPlanner::AStar(float start_x, float start_y, float goal_x, float goal_y,
			nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{

  /*
     * Calculate the corresponding cell indices in the occupancy grid for the start and goal position.
     */
    int   idx_start;
    int   idx_goal;
    idx_start  = (int)((start_y - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_start += (int)((start_x - map.info.origin.position.x)/map.info.resolution);
    idx_goal   = (int)((goal_y  - map.info.origin.position.y)/map.info.resolution)*map.info.width;
    idx_goal  += (int)((goal_x  - map.info.origin.position.x)/map.info.resolution);

    /*
     * Variables:
     * 'nodes':          Array of nodes. One for each cell in the occupancy grid. 
     * 'current_node':   A pointer to the node selected from the open list according to the MINIMUM F-VALUE criterion.
     * 'runtime_steps':  An auxiliar counter for benchmarking purposes.
     * 'node_neighbors': Array for storing the indices of the neighbors of the current node.
     */
    std::vector<Node> nodes;
    Node* current_node; 
    int runtime_steps = 0;
    std::vector<int> node_neighbors;
    nodes.resize(map.data.size());
    node_neighbors.resize(4);
    
    /*
     * Since the current node is selected according to the f-values, a convenient data structure is the PRIORITY QUEUE
     * with the comparison function defined to compare to nodes by their f-values.
     */
    std::priority_queue<Node*, std::vector<Node*>, CompareByFValue>   open_list;

    /*
     * THIS IS WHERE THE ALGORITHM BEGINS
     */
    /*
     * Initialization:
     * All distances are set to infinity (or in this case, to the maximum value).
     * All nodes are marked as not-belonging to the open nor closed list. 
     * After this, start node is added to the open list, marked as in-the-open-list and its distance is set to zero.
     */
    for(size_t i=0;  i < map.data.size(); i++)
    {
	nodes[i].index          = i;
	nodes[i].distance       = INT_MAX;
        nodes[i].in_open_list   = false;
        nodes[i].in_closed_list = false;
        nodes[i].parent         = NULL;
    }
    current_node = &nodes[idx_start];
    current_node->distance     = 0;
    current_node->in_open_list = true;
      
    float h = abs(start_x - goal_x) + abs(start_y - goal_y);
    current_node->f_value = h;
    
    open_list.push(current_node);

    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the node with the MINIMUM F-VALUE from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */

    while(!open_list.empty() && current_node->index != idx_goal)
    {
	//Choose the current node with the criterion of the MINIMUM F-VALUE and add it to the closed list.
	current_node = open_list.top();  
       	open_list.pop();                   
	current_node->in_closed_list = true;
	
	//Get the list of neighbors of the current node (using 4 connectivity).
	node_neighbors[0] = current_node->index + map.info.width;
	node_neighbors[1] = current_node->index + 1;
	node_neighbors[2] = current_node->index - map.info.width;
	node_neighbors[3] = current_node->index - 1;

	
	for(size_t i=0; i < node_neighbors.size(); i++)
	{
	    //If it is an occupied cell or it is in the closed list, ignore it.
	    if(map.data[node_neighbors[i]] > 40 || map.data[node_neighbors[i]] < 0 || nodes[node_neighbors[i]].in_closed_list)
		continue;
	    
	    //If the distance from the current node is less than the previously found distance, then change it,
	    //and set the current node as parent of this neighbor.
	    Node* neighbor = &nodes[node_neighbors[i]];
	    int dist = current_node->distance + 1;

	    float neighbor_x = (neighbor->index % map.info.width) * map.info.resolution + map.info.origin.position.x;
	    float neighbor_y = (neighbor->index / map.info.width) * map.info.resolution + map.info.origin.position.y;
	    float h_n = abs(goal_x - neighbor_x) + abs(goal_y - neighbor_y);

	 
	    if(dist < neighbor->distance)
	    {
      	        neighbor->distance = dist;
		neighbor->parent   = current_node;
		neighbor->f_value  = neighbor->distance + h_n; 
	    }
	    //If it is not in the open list, add it.
	    if(!neighbor->in_open_list)
	    {
		neighbor->in_open_list = true;
		open_list.push(neighbor);
	    }
	    
	    //Increment just to have a measure of the running time.
	    runtime_steps++; 
	}
    }
    //Check if the path was found
    if(current_node->index != idx_goal)
	return false;
    /*
     * THIS IS WHERE THE ALGORITHM ENDS
    */

    std::cout << "Path found using A* after " << runtime_steps << " steps." << std::endl;
    /*
     * Transform all nodes (cells with a corresponding index) to metric coordinates and add them to the resulting path.
     */
    result.header.frame_id = "map";
    result.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while(current_node->parent != NULL)
    {
	p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
	p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
	result.poses.insert(result.poses.begin(), p);
	current_node = current_node->parent;
    }
    return true;
}

Node::Node()
{
    this->index            = -1;
    this->distance         = INT_MAX;
    this->f_value          = INT_MAX;
    this->in_open_list     = false;
    this->in_closed_list   = false;
    this->parent           = NULL;  
}

Node::~Node()
{
}
