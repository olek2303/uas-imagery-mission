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

struct DirectedSaving {
    int from_idx;
    int to_idx;
    double value;
    bool operator<(const DirectedSaving& other) const { return value > other.value; }
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

double calculate_route_distance_separate_depots(
    const Route& route,
    int start_depot_idx,
    int end_depot_idx,
    const std::vector<std::vector<double>>& dist_matrix) {
    if (route.point_indices.empty()) return 0.0;
    double distance = 0.0;
    distance += dist_matrix[start_depot_idx][route.point_indices.front()];
    for (size_t i = 0; i < route.point_indices.size() - 1; ++i) {
        distance += dist_matrix[route.point_indices[i]][route.point_indices[i + 1]];
    }
    distance += dist_matrix[route.point_indices.back()][end_depot_idx];
    return distance;
}


std::vector<Route> clarke_wright_separate_depots(
    int start_depot_idx,
    int end_depot_idx,
    const std::vector<CustomerNode>& customers,
    const std::vector<std::vector<double>>& dist_matrix,
    double vehicle_capacity) {
    if (customers.empty()) return {};
    std::vector<Route> current_routes;
    for (const auto& customer : customers) {
        if (customer.demand > vehicle_capacity) {
            std::cerr << "Ostrzeżenie: Zapotrzebowanie klienta " << customer.id << " (" << customer.demand
                << ") przekracza pojemność pojazdu (" << vehicle_capacity << "). Pomijanie." << std::endl;
            continue;
        }
        Route initial_route;
        initial_route.point_indices.push_back(customer.id);
        initial_route.total_load = customer.demand;
        current_routes.push_back(initial_route);
    }
    std::vector<DirectedSaving> savings_list;
    for (const auto& cust_i : customers) {
        for (const auto& cust_j : customers) {
            if (cust_i.id == cust_j.id) continue;
            int idx_i = cust_i.id;
            int idx_j = cust_j.id;
            double saving_value = dist_matrix[idx_i][end_depot_idx] +
                dist_matrix[start_depot_idx][idx_j] -
                dist_matrix[idx_i][idx_j];
            if (saving_value > 0) {
                savings_list.push_back({ idx_i, idx_j, saving_value });
            }
        }
    }
    std::sort(savings_list.begin(), savings_list.end());
    for (const auto& saving : savings_list) {
        int from_idx = saving.from_idx;
        int to_idx = saving.to_idx;
        int route1_vec_idx = -1;
        int route2_vec_idx = -1;
        for (size_t i = 0; i < current_routes.size(); ++i) {
            if (current_routes[i].point_indices.empty()) continue;
            if (current_routes[i].point_indices.back() == from_idx) route1_vec_idx = i;
            if (current_routes[i].point_indices.front() == to_idx) route2_vec_idx = i;
        }
        if (route1_vec_idx != -1 && route2_vec_idx != -1 && route1_vec_idx != route2_vec_idx) {
            Route& r1 = current_routes[route1_vec_idx];
            Route& r2 = current_routes[route2_vec_idx];
            if (r1.total_load + r2.total_load <= vehicle_capacity) {
                r1.point_indices.insert(r1.point_indices.end(), r2.point_indices.begin(), r2.point_indices.end());
                r1.total_load += r2.total_load;
                r2.point_indices.clear();
                r2.total_load = 0;
            }
        }
    }
    std::vector<Route> final_routes;
    for (const auto& route : current_routes) {
        if (!route.point_indices.empty()) {
            Route final_route_obj = route;
            final_route_obj.total_distance = calculate_route_distance_separate_depots(
                route, start_depot_idx, end_depot_idx, dist_matrix);
            final_routes.push_back(final_route_obj);
        }
    }
    return final_routes;
}

int main_cplex_integration_example() {
    IloEnv env;
    try {
        const int num_total_points_in_matrix = 34;
        const int start_depot_idx = 0;
        const int end_depot_idx = 33;
        const int num_waypoints = 32;
        const char* filename = "uas-cpp-data.dat";
        const double vehicle_capacity = 90.0;

        IloArray<IloNumArray> d_ilo;
        IloArray<IloNumArray> c_ilo;
        readData(filename, d_ilo, c_ilo, env, num_total_points_in_matrix);
        std::cout << "Wczytano macierz odleglosci z pliku: " << filename << std::endl;

        std::vector<std::vector<double>> dist_matrix_std(num_total_points_in_matrix, std::vector<double>(num_total_points_in_matrix));
        for (int i = 0; i < num_total_points_in_matrix; ++i) {
            for (int j = 0; j < num_total_points_in_matrix; ++j) {
                dist_matrix_std[i][j] = d_ilo[i][j];
            }
        }
        
        std::vector<CustomerNode> customers_list;
        for (int i = 1; i <= num_waypoints; ++i) {
            customers_list.push_back({ i, 1.0 });
        }

        std::cout << "\nUruchamianie algorytmu oszczednosci dla osobnego startu i mety..." << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Indeks Startu: " << start_depot_idx << ", Indeks Mety: " << end_depot_idx << std::endl;
        std::cout << "Liczba Waypointow: " << num_waypoints << " (indeksy 1.." << num_waypoints << ")" << std::endl;
        std::cout << "Pojemnosc Pojazdu: " << vehicle_capacity << std::endl << std::endl;

        std::vector<Route> initial_routes = clarke_wright_separate_depots(
            start_depot_idx,
            end_depot_idx,
            customers_list,
            dist_matrix_std,
            vehicle_capacity
        );

        std::cout << "Wygenerowane Trasy Poczatkowe:" << std::endl;
        double total_distance_all_routes = 0;
        for (size_t i = 0; i < initial_routes.size(); ++i) {
            const auto& route = initial_routes[i];
            std::cout << "Trasa " << i + 1 << ": ";
            std::cout << "Start(" << start_depot_idx << ") -> ";
            for (size_t j = 0; j < route.point_indices.size(); ++j) {
                std::cout << route.point_indices[j] << (j == route.point_indices.size() - 1 ? "" : " -> ");
            }
            std::cout << " -> Meta(" << end_depot_idx << ")";
            std::cout << " (Ladunek: " << route.total_load
                << ", Dystans: " << std::fixed << std::setprecision(2) << route.total_distance << ")" << std::endl;
            total_distance_all_routes += route.total_distance;
        }
        if (!initial_routes.empty()) {
            std::cout << std::endl << "Calkowity dystans dla wszystkich tras: " << total_distance_all_routes << std::endl;
        }
        else {
            std::cout << "Nie wygenerowano zadnych tras. Sprawdz pojemnosc pojazdu i zapotrzebowania." << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Wyjatek standardowy: " << e.what() << std::endl;
    }
    env.end();
    return 0;
}