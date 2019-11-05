#include "route_planner.h"
#include <algorithm>

// Finds the closest nodes to the starting and ending coordinates
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node =  &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}

// Assigns the distance to the end_node for the h value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*this->end_node);
}

// Adds all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

// Sorts the open list according to the f-value and return the next node
RouteModel::Node *RoutePlanner::NextNode() 
{
    RouteModel::Node *lowest_sum_node = nullptr;

    // Sort the open list using lamda for custom comparator
    std::sort(open_list.begin(), open_list.end(), [](const auto& lhs, const auto& rhs) {
        float f_lhs = lhs->h_value + lhs->g_value;
        float f_rhs = rhs->h_value + rhs->g_value;
        return f_lhs < f_rhs;
    });

    lowest_sum_node = this->open_list.front();
    this->open_list.erase(this->open_list.begin());
    return lowest_sum_node;
}

// Returns the final path found from your A* search
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr ) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    
    path_found.push_back(*this->start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// The A* Search algorithm
void RoutePlanner::AStarSearch() 
{
    RouteModel::Node *current_node = nullptr;
    current_node = this->start_node;
    current_node->visited = true;
    this->open_list.push_back(current_node);

    while (this->open_list.size() > 0) {
        current_node = this->NextNode();
        if (current_node == this->end_node) {
            m_Model.path = this->ConstructFinalPath(current_node);
        } else {
            this->AddNeighbors(current_node);
        }
    }
}