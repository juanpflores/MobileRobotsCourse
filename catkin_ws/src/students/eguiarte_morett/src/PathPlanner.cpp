#include "PathPlanner.h"

bool PathPlanner::algorithm(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result, PathPlanner::MODE mode){
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

    //Defining the OPEN list for each algorithm
    /*
     * In the Breadth First Search algorithm, the open list is a QUEUE.
     */
    std::queue<Node*>   open_list_bfs;
    /*
     * In the Depth First Search algorithm, the open list is a STACK.
     */
    std::stack<Node*>   open_list_dfs;
    /*
     * In A* 
     * Since the current node is selected according to the f-values, a convenient data structure is the PRIORITY QUEUE
     * with the comparison function defined to compare to nodes by their f-values.
     */
    std::priority_queue<Node*, std::vector<Node*>, CompareByFValue>   open_list_astar;
    /*
     * In Dijkstra 
     * Since the current node is selected according to the distances, a convenient data structure is the PRIORITY QUEUE
     * with the comparison function defined to compare to nodes by their distances.
    */
    std::priority_queue<Node*, std::vector<Node*>, CompareByDistance>  open_list_dijkstra;

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
    current_node->f_value = Node::manhattan_distance((int) start_x, (int) start_y, (int)goal_x, (int)goal_y);  

    if(mode == BFS) open_list_bfs.push(current_node);
    if(mode == DFS) open_list_dfs.push(current_node);
    if(mode == DIJKSTRA) open_list_dijkstra.push(current_node);
    if(mode == ASTAR) open_list_astar.push(current_node);



    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node with the criterium of the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    while((mode == BFS && !open_list_bfs.empty() && current_node->index != idx_goal)
    ||(mode == DFS && !open_list_dfs.empty() && current_node->index != idx_goal)
    ||(mode == DIJKSTRA && !open_list_dijkstra.empty() && current_node->index != idx_goal) 
    ||(mode == ASTAR && !open_list_astar.empty() && current_node->index != idx_goal) )
    {
        //Choose the current node with the criterion of the LEAST RECENT and add it to the closed list.
        if(mode == BFS){current_node = open_list_bfs.front(); open_list_bfs.pop();}
        //DFS : Choose the current node with the criterion of the MOST RECENT and add it to the closed list.
        if(mode == DFS){current_node = open_list_dfs.top(); open_list_dfs.pop();}
        //DIJKSTRA : Choose the current node as the node with the MINIMUM DISTANCE from the open list
        if(mode == DIJKSTRA){current_node = open_list_dijkstra.top(); open_list_dijkstra.pop();}
        //ASTAR : Choose the current node as the node with the MINIMUM F-VALUE from the open list
        if(mode == ASTAR){current_node = open_list_astar.top(); open_list_astar.pop();}
                  
        current_node->in_closed_list = true;
        
        //Get the list of neighbors of the current node (using 4 connectivity).
        node_neighbors[0] = current_node->index + map.info.width; //Up
        node_neighbors[1] = current_node->index + 1; //Right
        node_neighbors[2] = current_node->index - map.info.width; //Down
        node_neighbors[3] = current_node->index - 1; //Left
        
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
                neighbor->distance = dist; //This distance is the criteria for Dijkstra's Heap
                neighbor->parent   = current_node;
                //For ASTAR use manhattan distance heuristic for the cost function heap criteria
                if(mode == ASTAR){
                    int x, y;
                    x = (int)(neighbor->index % map.info.width)*map.info.resolution +  map.info.origin.position.x;
                    y = (int)(neighbor->index / map.info.width)*map.info.resolution +  map.info.origin.position.y;
                    neighbor->f_value = neighbor->distance + Node::manhattan_distance(x, y, (int)goal_x, (int)goal_y);
                }
            }
            //If it is not in the open list, add it.
            if(!neighbor->in_open_list)
            {
                neighbor->in_open_list = true;
                if(mode==BFS) open_list_bfs.push(neighbor);
                if(mode==DFS) open_list_dfs.push(neighbor);
                if(mode==DIJKSTRA) open_list_dijkstra.push(neighbor);
                if(mode==ASTAR) open_list_astar.push(neighbor);
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

    if(mode == BFS) std::cout << "Path found using BREADTH FIRST SEARCH after " << runtime_steps << " steps." << std::endl;
    if(mode == DFS) std::cout << "Path found using DEPTH FIRST SEARCH after " << runtime_steps << " steps." << std::endl;
    if(mode == DIJKSTRA) std::cout << "Path found using DIJKSTRA after " << runtime_steps << " steps." << std::endl;
    if(mode == ASTAR) std::cout << "Path found using ASTAR after " << runtime_steps << " steps." << std::endl;
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

bool PathPlanner::BreadthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
			       nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{

    /*
     * WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the LEAST recent node from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    return PathPlanner::algorithm(start_x, start_y, goal_x, goal_y, map, result, BFS);
}

bool PathPlanner::DepthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
			     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{
    /*
     * WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the MOST recent node from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    return PathPlanner::algorithm(start_x, start_y, goal_x, goal_y, map, result, DFS);

}

bool PathPlanner::Dijkstra(float start_x, float start_y, float goal_x, float goal_y,
		     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{   
    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the node with the MINIMUM DISTANCE from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    return PathPlanner::algorithm(start_x, start_y, goal_x, goal_y, map, result, DIJKSTRA);
}

bool PathPlanner::AStar(float start_x, float start_y, float goal_x, float goal_y,
			nav_msgs::OccupancyGrid& map, nav_msgs::Path& result)
{
    /*
     * Main loop of the algorithm: WHILE open list is NOT empty and the goal node has not been found:
     *     Choose the current node as the node with the MINIMUM F-VALUE from the open list
     *     For each neighbor of the current node
     *         If it is not in the open list, add it
     *         If the distance arriving by the current node is less than previous distance, then
     *             Change the distance of the neighbor
     *             Set the current node as parent of such neighbor
     */
    return PathPlanner::algorithm(start_x, start_y, goal_x, goal_y, map, result, ASTAR);
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
