#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include "route_model.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() const { return distance; }
    void AStarSearch();

    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();
    void NodeSort();

  private:
    std::vector<RouteModel::Node *> open_list;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    static bool Compare(RouteModel::Node *, RouteModel::Node *);

    float distance = 0.0f;
    RouteModel &m_Model;
};

#endif
