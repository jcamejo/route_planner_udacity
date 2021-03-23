#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_x);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) { return node->distance(*end_node); }

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node *neighbor : current_node->neighbors) {
        float current_g = current_node->g_value;
        float neighbor_d = current_node->distance(*neighbor);
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = current_g + neighbor_d;
        neighbor->parent = current_node;
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}

bool RoutePlanner::Compare(RouteModel::Node *node1, RouteModel::Node *node2) {
    const float f1 = node1->g_value + node1->h_value;
    const float f2 = node2->g_value + node2->h_value;
    return f1 > f2;
}

void RoutePlanner::NodeSort() { std::sort(open_list.begin(), open_list.end(), RoutePlanner::Compare); }

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *next_node;
    this->NodeSort();
    next_node = open_list.back();
    open_list.pop_back();

    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::vector<RouteModel::Node> path_found{};
    distance = 0.0f;

    while (current_node != start_node) {
        distance += current_node->distance(*(current_node->parent));
        path_found.emplace_back(*current_node);
        current_node = current_node->parent;
    }

    // Start node
    path_found.emplace_back(*current_node);

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    std::reverse(path_found.begin(), path_found.end());

    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->h_value = this->CalculateHValue(current_node);
    current_node->visited = true;

    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = this->NextNode();
    }

    m_Model.path = this->ConstructFinalPath(current_node);
}
