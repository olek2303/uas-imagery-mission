#include "helper_functions.h"
#include "ilcplex\cplex.h"
#include "ilcplex\ilocplex.h"
#include "ilopl\iloopl.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <stdexcept>
#include <cmath>

struct CustomerNode {
    int id;
    double demand;
};

struct Route {
    std::vector<int> point_indices;
    double total_load;
    double total_distance;

    Route() : total_load(0), total_distance(0) {}
};

struct Saving {
    int point1_idx;
    int point2_idx;
    double value;

    bool operator<(const Saving& other) const {
        return value > other.value;
    }
};

double calculate_route_distance_from_matrix(
    const Route& route,
    int depot_idx,
    const std::vector<std::vector<double>>& dist_matrix) {

    if (route.point_indices.empty()) {
        return 0.0;
    }
    double distance = 0.0;
    distance += dist_matrix[depot_idx][route.point_indices.front()];
    
    for (size_t i = 0; i < route.point_indices.size() - 1; ++i) {
        distance += dist_matrix[route.point_indices[i]][route.point_indices[i + 1]];
    }
    
    distance += dist_matrix[route.point_indices.back()][depot_idx];
    return distance;
}


std::vector<Route> clarke_wright_savings_from_matrix(
    int depot_idx,
    const std::vector<CustomerNode>& customers,
    const std::vector<std::vector<double>>& dist_matrix,
    double vehicle_capacity) {

    if (customers.empty()) {
        return {};
    }
    
    std::vector<Route> current_routes;
    for (const auto& customer : customers) {
        if (customer.demand > vehicle_capacity) {
            std::cerr << "Warning: Customer ID " << customer.id << " (matrix index potentially different) demand ("
                << customer.demand << ") exceeds vehicle capacity (" << vehicle_capacity
                << "). Skipping." << std::endl;
            continue;
        }
        Route initial_route;
        initial_route.point_indices.push_back(customer.id);
        initial_route.total_load = customer.demand;
        current_routes.push_back(initial_route);
    }
    
    std::vector<Saving> savings_list;
    for (size_t i = 0; i < customers.size(); ++i) {
        for (size_t j = i + 1; j < customers.size(); ++j) {
            const CustomerNode& cust_node_i = customers[i];
            const CustomerNode& cust_node_j = customers[j];
            
            int idx_i = cust_node_i.id;
            int idx_j = cust_node_j.id;

            if (idx_i == depot_idx || idx_j == depot_idx) continue;

            double d_depot_i = dist_matrix[depot_idx][idx_i];
            double d_depot_j = dist_matrix[depot_idx][idx_j];
            double d_i_j = dist_matrix[idx_i][idx_j];

            Saving s;
            s.point1_idx = idx_i;
            s.point2_idx = idx_j;
            s.value = d_depot_i + d_depot_j - d_i_j;
            if (s.value > 0) {
                savings_list.push_back(s);
            }
        }
    }
    
    std::sort(savings_list.begin(), savings_list.end());
    
    for (const auto& saving : savings_list) {
        int p1_matrix_idx = saving.point1_idx;
        int p2_matrix_idx = saving.point2_idx;

        int route1_vec_idx = -1, route2_vec_idx = -1;
        bool p1_is_front_in_r1 = false;
        bool p2_is_front_in_r2 = false;
        
        for (size_t i = 0; i < current_routes.size(); ++i) {
            const auto& route = current_routes[i];
            if (route.point_indices.empty()) continue;

            if (route.point_indices.front() == p1_matrix_idx) {
                route1_vec_idx = i;
                p1_is_front_in_r1 = true;
            }
            else if (route.point_indices.back() == p1_matrix_idx) {
                route1_vec_idx = i;
                p1_is_front_in_r1 = false;
            }

            if (route.point_indices.front() == p2_matrix_idx) {
                route2_vec_idx = i;
                p2_is_front_in_r2 = true;
            }
            else if (route.point_indices.back() == p2_matrix_idx) {
                route2_vec_idx = i;
                p2_is_front_in_r2 = false;
            }
        }

        if (route1_vec_idx != -1 && route2_vec_idx != -1 && route1_vec_idx != route2_vec_idx) {
            Route& r1 = current_routes[route1_vec_idx];
            Route& r2 = current_routes[route2_vec_idx];
            
            if (r1.total_load + r2.total_load <= vehicle_capacity) {
                Route merged_route;
                merged_route.total_load = r1.total_load + r2.total_load;
                
                if (!p1_is_front_in_r1 && p2_is_front_in_r2) {
                    merged_route.point_indices = r1.point_indices;
                    merged_route.point_indices.insert(merged_route.point_indices.end(), r2.point_indices.begin(), r2.point_indices.end());
                }
                else if (!p2_is_front_in_r2 && p1_is_front_in_r1) {
                    merged_route.point_indices = r2.point_indices;
                    merged_route.point_indices.insert(merged_route.point_indices.end(), r1.point_indices.begin(), r1.point_indices.end());
                }
                else if (p1_is_front_in_r1 && p2_is_front_in_r2) {
                    std::reverse(r1.point_indices.begin(), r1.point_indices.end());
                    merged_route.point_indices = r1.point_indices;
                    merged_route.point_indices.insert(merged_route.point_indices.end(), r2.point_indices.begin(), r2.point_indices.end());
                }
                else if (!p1_is_front_in_r1 && !p2_is_front_in_r2) {
                    merged_route.point_indices = r1.point_indices;
                    std::reverse(r2.point_indices.begin(), r2.point_indices.end());
                    merged_route.point_indices.insert(merged_route.point_indices.end(), r2.point_indices.begin(), r2.point_indices.end());
                }
                else { continue; }


                current_routes[route1_vec_idx] = merged_route;
                current_routes[route2_vec_idx].point_indices.clear();
                current_routes[route2_vec_idx].total_load = 0;
            }
        }
    }
    
    std::vector<Route> final_routes;
    for (const auto& route : current_routes) {
        if (!route.point_indices.empty()) {
            Route final_route_obj = route;
            final_route_obj.total_distance = calculate_route_distance_from_matrix(route, depot_idx, dist_matrix);
            final_routes.push_back(final_route_obj);
        }
    }
    return final_routes;
}

int main_cplex_integration_example() {
    IloEnv env;
    try {
        const int num_total_points_in_matrix = 34;
        const int depot_matrix_idx = 0;
        const int num_waypoints = 32;

        IloArray<IloNumArray> d_ilo;
        IloArray<IloNumArray> c_ilo;

        readData("uas-cpp-data.dat", d_ilo, c_ilo, env, num_total_points_in_matrix);
        
        std::vector<std::vector<double>> dist_matrix_std(
            num_total_points_in_matrix,
            std::vector<double>(num_total_points_in_matrix)
        );
        for (int i = 0; i < num_total_points_in_matrix; ++i) {
            for (int j = 0; j < num_total_points_in_matrix; ++j) {
                dist_matrix_std[i][j] = d_ilo[i][j];
            }
        }
        
        std::vector<CustomerNode> customers_list_std;
        for (int i = 1; i <= num_waypoints; ++i) {
            customers_list_std.push_back({ i, 1.0 });
        }

        double vehicle_capacity_cpp = 10.0;

        std::cout << "Clarke & Wright Savings Algorithm (from Matrix) for Initial Route Generation" << std::endl;
        std::cout << "----------------------------------------------------------------------------" << std::endl;
        std::cout << "Depot Matrix Index: " << depot_matrix_idx << std::endl;
        std::cout << "Number of Waypoints: " << customers_list_std.size() << std::endl;
        std::cout << "Vehicle Capacity: " << vehicle_capacity_cpp << std::endl << std::endl;

        std::vector<Route> initial_routes = clarke_wright_savings_from_matrix(
            depot_matrix_idx,
            customers_list_std,
            dist_matrix_std,
            vehicle_capacity_cpp
        );

        std::cout << "Generated Initial Routes:" << std::endl;
        double total_dist_all_routes = 0;
        for (size_t i = 0; i < initial_routes.size(); ++i) {
            const auto& route = initial_routes[i];
            std::cout << "Route " << i + 1 << ": ";
            std::cout << "Depot(" << depot_matrix_idx << ") -> ";
            for (size_t j = 0; j < route.point_indices.size(); ++j) {
                std::cout << route.point_indices[j] << (j == route.point_indices.size() - 1 ? "" : " -> ");
            }
            std::cout << " -> Depot(" << depot_matrix_idx << ")";
            std::cout << " (Load: " << route.total_load
                << ", Distance: " << std::fixed << std::setprecision(2) << route.total_distance << ")" << std::endl;
            total_dist_all_routes += route.total_distance;
        }
        if (!initial_routes.empty()) {
            std::cout << std::endl << "Total distance for all initial routes: " << total_dist_all_routes << std::endl;
        }
        else {
            std::cout << "No routes generated." << std::endl;
        }

    }
    catch (const std::exception& e) {
        std::cerr << "Standard exception caught: " << e.what() << std::endl;
        env.end();
        return 1;
    }
    env.end();
    return 0;
}