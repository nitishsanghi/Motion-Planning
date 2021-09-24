#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    //std::cout << "In routeplanner" << std::endl;
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    std::cout << start_x << start_y << end_x << end_y << std::endl;
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    //std::cout << start_node << " "<< start_node->g_value << " " << start_node->h_value<<std::endl;
    //std::cout << end_node << " "<< end_node->g_value << " "<< end_node->h_value << std::endl;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //std::cout << "Add Neighbors Start" << std::endl;
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors) {
        if(std::find(open_list.begin(), open_list.end(), node) == open_list.end()){
            node->visited = true;
            node->parent = current_node;
            node->g_value = current_node->g_value + current_node->distance(*node);
            node->h_value = CalculateHValue(node);
            open_list.emplace_back(node);
        }
        if(std::find(open_list.begin(), open_list.end(), node) != open_list.end()){
            if ((current_node->g_value + current_node->distance(*node)) < node->g_value ){
                node->parent = current_node;
                node->g_value = current_node->g_value + current_node->distance(*node);
            }
        }
    }
    //std::cout << "Add Neighbors End" << std::endl;
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool compare(RouteModel::Node *a, RouteModel::Node *b){
    return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    //std::cout << "Next node " << std::endl;
    std::sort(open_list.begin(), open_list.end(), compare );
    RouteModel::Node *temp = open_list.back();
    temp->visited = true;
    for (auto node : open_list){
        //std::cout << "f-value : " << node->g_value + node->h_value << std::endl;
        
        //std::cout << node << " " << open_list.size() << std::endl;
    }
    open_list.pop_back();
    //std::cout<<temp->g_value<<std::endl;
    //std::cout<<temp<<std::endl;
    //std::cout << "Next node end" << std::endl;
    return temp;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    //std::cout << "Constructing final path" << std::endl;
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.emplace_back(*current_node);
    while(current_node->parent != nullptr){
        //std::cout << "Constructing path." << std::endl;
        //std::cout << current_node->parent << std::endl;
        distance = distance + current_node->distance(*current_node->parent);
        current_node = current_node->parent;
         path_found.emplace_back(*current_node);
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    //std::cout << "Path # nodes : " << path_found.size()<< std::endl;
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    //std::cout << "AStarSearch" << std::endl;
    // TODO: Implement your solution here.
    current_node = start_node;
    current_node->h_value = CalculateHValue(current_node);
    current_node->visited = true;
    AddNeighbors(current_node);

    //std::cout << current_node << " "<< current_node->g_value << " " << current_node->h_value<<std::endl;
    
    //std::cout << "Open list size : " << open_list.size() << std::endl;
    int counter = 0;
    while (open_list.size()){
        //std::cout << "Open list size : " << open_list.size() << std::endl;
        current_node = NextNode();
        AddNeighbors(current_node);
        
        //std::cout << "Inter coordinates : " << current_node->x << " " << current_node->y << std::endl;
        counter++;
        //std::cout << "Counter : " << counter  << std::endl;
        if (current_node == end_node){
            //std::cout << "End coordinates : " << current_node->x << " " << current_node->y << std::endl;
           m_Model.path = ConstructFinalPath(current_node);
            break;
        }
    }
}