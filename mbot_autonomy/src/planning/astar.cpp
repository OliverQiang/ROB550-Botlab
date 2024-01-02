#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>

float maxLaserDistance = 50000;
int8_t hitOdds = 5;
int8_t missOdds = 2;


using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    // printf("Start Node: %d, %d\n", startCell.x, startCell.y);
    // printf("Goal Node: %d, %d\n", goalCell.x, goalCell.y);

    PriorityQueue open;
    PriorityQueue closed;
    PriorityQueue visited;

    open.push(startNode);
    Node * current_node;
    Node * new_node;
    while(!open.empty()){
        current_node = open.pop();
        // printf("Current Node: %d, %d\n", current_node->cell.x, current_node->cell.y);
        if((*current_node == *goalNode)){ 
            found_path = true;
            goalNode->parent = current_node->parent;
            break;
        }
        std::vector<Node*> children = expand_node(current_node, distances, params);
        for(auto& child : children){
            if((!closed.is_member(child)) && (distances.isCellInGrid(child->cell.x,child->cell.y)) && ((distances(child->cell.x,child->cell.y) > 0 || distances(child->cell.x, child->cell.y) == -1))){   
                // If the child is not visited, calculate costs for it and push to open & visited list
                if(!visited.is_member(child)){
                    child->g_cost = g_cost(current_node, child, distances, params);
                    child->h_cost = h_cost(child, goalNode, distances);
                    child->parent = current_node;
                    open.push(child);
                    visited.push(child);
                }
                // If child is already visited but with lower g_cost, update its cost in the visited list and push to open
                else if(visited.get_member(child)->g_cost > g_cost(current_node, child, distances, params)){
                    visited.get_member(child)->g_cost = g_cost(current_node, child, distances, params);
                    visited.get_member(child)->parent = current_node;
                    open.push(visited.get_member(child));
                }
            }
            // // Do we need to free memory space?
            // delete child;
        }
        // // Clear the vector after freeing memory
        // children.clear();
        closed.push(current_node);
    }
    /////////////////////////////////////////////////////////////////////////////////
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {   
        auto nodePath = extract_node_path(goalNode, startNode);
        // // Optional: Prune pose
        // nodePath =  prune_node_path(nodePath);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }
    else{
         printf("[A*] Didn't find a path\n");
    }
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    int dx = std::abs(from->cell.x - goal->cell.x);
    int dy = std::abs(from->cell.y - goal->cell.y);
    if(dx >= dy)
        h_cost = (dx+dy)+(1.414 - 2)*dy;
    else
        h_cost = (dx+dy)+(1.414 - 2)*dx;
    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    int dx = std::abs(from->cell.x - goal->cell.x);
    int dy = std::abs(from->cell.y - goal->cell.y);
    if((dx==1) && (dy==1))
        g_cost = from->g_cost + 1.414;
    else
        g_cost = from->g_cost + 1;
    double obstacle_cost_weight = 1;
    g_cost += obstacle_cost_weight * 1/distances(goal->cell.x, goal->cell.y);
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    const int dx[] = {1, -1, 0,  0, 1, 1, -1, -1};
    const int dy[] = {0,  0, 1, -1, 1, -1, 1, -1};
    for(int i = 0; i < 8; i++){
        Node* newChild = new Node(node->cell.x + dx[i], node->cell.y + dy[i]);
        children.push_back(newChild);
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node * current_node = goal_node;
    if ((current_node == NULL) || (current_node->parent == NULL)){
        printf("Current node/current node's parent is NULL; path cannot be extracted");
        return path;
    }
    while (!(current_node == start_node)){
        path.push_back(current_node);
        if (current_node->parent != NULL){
            current_node = current_node->parent;
        }
    }
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    int node_length = nodes.size();
    for(int i = 0; i < node_length - 1; i++){
        mbot_lcm_msgs::pose2D_t nextPose;
        nextPose.theta = std::atan((nodes[i+1]->cell.y - nodes[i]->cell.y)/(nodes[i+1]->cell.x - nodes[i]->cell.x));
        nextPose.x = grid_position_to_global_position(Point<double>(nodes[i+1]->cell.x, nodes[i+1]->cell.y), distances).x;
        nextPose.y = grid_position_to_global_position(Point<double>(nodes[i+1]->cell.x, nodes[i+1]->cell.y), distances).y;
        path.push_back(nextPose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath, const OccupancyGrid& map)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    
    if (nodePath.empty()){
        return new_node_path;
    }

    Node * current = nodePath.front();
    new_node_path.push_back(current);

    for (size_t i = 2; i < nodePath.size(); ++i) {
        if (!has_line_of_sight(current, nodePath[i], map)) {
            current = nodePath[i - 1];
            new_node_path.push_back(current);
        }
    }


    new_node_path.push_back(nodePath.back());
    return new_node_path;

}

bool has_line_of_sight(Node* from, Node* to, const OccupancyGrid& map){
    Point<int> gridFrom = global_position_to_grid_cell(Point<double>(from->cell.x, from->cell.y), map);
    Point<int> gridTo = global_position_to_grid_cell(Point<double>(to->cell.x, to->cell.y), map);

    adjusted_ray_t ray;
    ray.origin = Point<float>(gridFrom.x, gridFrom.y);
    ray.range = sqrt(pow(gridTo.x - gridFrom.x, 2) + pow(gridTo.y - gridFrom.y, 2));
    ray.theta = atan2(gridTo.y - gridFrom.y, gridTo.x - gridFrom.x);

    // Mapping mapping(maxLaserDistance, hitOdds, missOdds); 
    std::vector<Point<int>> cells = bresenham(ray, map);

    for (const auto& cell : cells) {
        if (map.isCellInGrid(cell.x, cell.y) && map.logOdds(cell.x, cell.y) > 0) {
            return false;
        }
    }
    return true;
}


std::vector<Point<int>> bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);

    int x0 = static_cast<int> (rayStart.x);
    int y0 = static_cast<int> (rayStart.y);

    // we don't want to score any cells that are outside of our range
    float range_corrected = ray.range;
    if(ray.range > maxLaserDistance){
        range_corrected = maxLaserDistance;
    }

     Point<float> endpoint = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
            map
        );

    int x1 = static_cast<int> (endpoint.x);
    int y1 = static_cast<int> (endpoint.y);

    // int x1 = static_cast<int> (x0 + range_corrected * std::cos(ray.theta) * map.cellsPerMeter());
    // int y1 = static_cast<int> (y0 + range_corrected * std::sin(ray.theta) * map.cellsPerMeter());
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    // find sign values
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    std::vector<Point<int>> cells; 
    cells.push_back(Point<int>(x,y));

    while(x!=x1 || y!=y1){
        int e2 = 2 * err;
        if(e2 >= -dy){
            err = err - dy;
            x = x + sx;
        }
        if(e2 <= dx){
            err = err + dx;
            y = y + sy;
        }
        cells.push_back(Point<int>(x,y));
    }
    return cells;
    
}