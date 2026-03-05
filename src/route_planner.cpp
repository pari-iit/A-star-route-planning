#include "route_planner.h"
#include <algorithm>

static bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  float f1 = a->g_value + a->h_value; // f1 = g1 + h1
  float f2 = b->g_value + b->h_value; // f2 = g2 + h2
  return f1 > f2; 
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance( (*end_node) );
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();

    for(RouteModel::Node* n:current_node->neighbors){
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = current_node->g_value + n->distance(*current_node);
        n->visited = true;
        open_list.emplace_back(n);
    }
    
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),[](const RouteModel::Node* a, const RouteModel::Node* b) {
          float f1 = a->g_value + a->h_value; // f1 = g1 + h1
          float f2 = b->g_value + b->h_value; // f2 = g2 + h2
          return f1 > f2; 
        });    
    // std::reverse(open_list.begin(),open_list.end());
    RouteModel::Node *nearest = open_list.back();
    open_list.pop_back();
    return nearest;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node != start_node){
        RouteModel::Node *parent = current_node->parent;
        distance += current_node->distance(*parent);
        path_found.push_back(*current_node);
        current_node = parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
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

    // TODO: Implement your solution here.    

    start_node->visited = true;
    open_list.push_back(start_node);
        
    while(!open_list.empty()){
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else{
            AddNeighbors(current_node);
        }
        

    }    
    

    

}
