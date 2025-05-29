#pragma once

#include <iostream>
#include <vector>

#include "helper_functions.h"
#include "ilcplex\cplex.h"
#include "ilcplex\ilocplex.h"
#include "ilopl\iloopl.h"


struct CustomerNode;
struct Route;
int uas_imagery_mission();
std::vector<Route> clarke_wright_savings_from_matrix(
    int depot_idx,                             // Index of the depot in the dist_matrix
    const std::vector<CustomerNode>& customers, // List of customers/waypoints
    const std::vector<vector<double>>& dist_matrix,
    double vehicle_capacity);
int main_cplex_integration_example();