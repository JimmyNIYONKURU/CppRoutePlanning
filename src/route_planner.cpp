#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage and initialize start_node and end_node
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

RouteModel::Node* RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b) {
        return a->h_value + a->g_value < b->h_value + b->g_value;
    });

    RouteModel::Node *lowest_sum_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_sum_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node) {
        path_found.push_back(*current_node);
        if (current_node->parent) {
            distance += current_node->distance(*(current_node->parent));
        }
        current_node = current_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Convert the distance from nodes to meters
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    current_node->visited = true;
    open_list.push_back(current_node);

    while (!open_list.empty()) {
        current_node = NextNode();

        // Check if we've reached our goal
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        AddNeighbors(current_node);
    }
}
