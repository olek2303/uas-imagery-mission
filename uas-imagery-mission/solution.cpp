#include "helper_functions.h"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "ilopl/iloopl.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <random>
#include <set>

struct CustomerNode {
    int id;
    double demand;
};

struct Route {
    std::vector<int> point_indices;
    double total_distance;

    Route() : total_distance(0) {}
};

struct DirectedSaving {
    int from_idx;
    int to_idx;
    double value;
    bool operator<(const DirectedSaving& other) const { return value > other.value; }
};

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
    const std::vector<CustomerNode>& nodes,
    const std::vector<std::vector<double>>& dist_matrix,
    std::vector<DirectedSaving>& savings_list) {

    if (nodes.empty()) return {};

    std::vector<Route> current_routes;
    for (const auto& customer : nodes) {
        Route initial_route;
        initial_route.point_indices.push_back(customer.id);
        current_routes.push_back(initial_route);
    }

 

    std::sort(savings_list.begin(), savings_list.end());

    for (const auto& saving : savings_list) {
        int from_idx = saving.from_idx;
        int to_idx = saving.to_idx;

        int route1_idx = -1;
        int route2_idx = -1;

        for (size_t i = 0; i < current_routes.size(); ++i) {
            const auto& pts = current_routes[i].point_indices;
            if (pts.empty()) continue;
            if (pts.back() == from_idx) route1_idx = static_cast<int>(i);
            if (pts.front() == to_idx) route2_idx = static_cast<int>(i);
        }

        if (route1_idx != -1 && route2_idx != -1 && route1_idx != route2_idx) {
            auto& r1 = current_routes[route1_idx];
            auto& r2 = current_routes[route2_idx];

            bool has_overlap = false;
            for (int pid : r2.point_indices) {
                if (std::find(r1.point_indices.begin(), r1.point_indices.end(), pid) != r1.point_indices.end()) {
                    has_overlap = true;
                    //break;
                }
            }

            if (true) {
                r1.point_indices.insert(
                    r1.point_indices.end(),
                    r2.point_indices.begin(),
                    r2.point_indices.end()
                );
                r2.point_indices.clear();
            }
        }
    }

    std::vector<Route> final_routes;
    for (const auto& route : current_routes) {
        if (!route.point_indices.empty()) {
            Route final_route = route;
            final_route.total_distance = calculate_route_distance_separate_depots(
                route, start_depot_idx, end_depot_idx, dist_matrix);
            final_routes.push_back(final_route);
        }
    }

   /* if (final_routes.size() != 1 || final_routes[0].point_indices.size() != nodes.size()) {
        std::cerr << "Błąd: nie udało się znaleźć jednej pełnej trasy obejmującej wszystkie punkty.\n";
    }*/

    return final_routes;
}

std::vector<Route> generate_routes_with_variants(
    int start_depot_idx,
    int end_depot_idx,
    const std::vector<CustomerNode>& nodes,
    const std::vector<std::vector<double>>& dist_matrix,
    int num_variants = 3) {

    std::vector<Route> all_valid_routes;

    std::vector<DirectedSaving> base_savings;
    for (const auto& cust_i : nodes) {
        for (const auto& cust_j : nodes) {
            if (cust_i.id == cust_j.id) continue;
            double saving_value =
                dist_matrix[cust_i.id][end_depot_idx] +
                dist_matrix[start_depot_idx][cust_j.id] -
                dist_matrix[cust_i.id][cust_j.id];
            if (saving_value > 0) {
                base_savings.push_back({ cust_i.id, cust_j.id, saving_value });
            }
        }
    }

    std::sort(base_savings.begin(), base_savings.end());

    int seed = 42;
    for (int k = 0; k < num_variants; ++k) {
        std::vector<DirectedSaving> savings_variant = base_savings;

        std::shuffle(savings_variant.begin(), savings_variant.end(), std::default_random_engine(seed + k));

        std::vector<Route> routes_variant = clarke_wright_separate_depots(
            start_depot_idx, end_depot_idx, nodes, dist_matrix, savings_variant);

        if (!routes_variant.empty()) {
            all_valid_routes.push_back(routes_variant[0]);
        }

        std::sort(all_valid_routes.begin(), all_valid_routes.end(),
            [](const Route& a, const Route& b) {
                return a.point_indices.size() > b.point_indices.size();
            });



        if ((int)all_valid_routes.size() >= num_variants)
            break;
    }

    return all_valid_routes;
}


int main_cplex_integration_example() {
    IloEnv env;
    try {
        const int num_total_points_in_matrix = 101;
        const char* filename = "data05.dat";
   /*     const int num_total_points_in_matrix = 34;
        const char* filename = "uas-cpp-data.dat";*/
        const int start_depot_idx = 0;
        const int end_depot_idx = num_total_points_in_matrix - 1;
        const int num_waypoints = num_total_points_in_matrix - 2;
        

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

        std::vector<CustomerNode> nodes_list;
        for (int i = 1; i <= num_waypoints; ++i) {
            nodes_list.push_back({ i, 1.0 });
        }

        std::cout << "\nUruchamianie algorytmu oszczednosci..." << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Indeks Startu: " << start_depot_idx << ", Indeks Mety: " << end_depot_idx << std::endl;
        std::cout << "Liczba Waypointow: " << num_waypoints << " (indeksy 1.." << num_waypoints << ")" << std::endl;

    

        std::vector<Route> routes = generate_routes_with_variants(
            start_depot_idx, end_depot_idx, nodes_list, dist_matrix_std, 10);

        std::cout << "Wygenerowano " <<  routes.size() <<" trasy:" << std::endl;
      
        for (size_t i = 0; i < routes.size(); ++i) {
            const auto& route = routes[i];
            std::cout << "Trasa " << i + 1 << ": ";
            std::cout << "Start(" << start_depot_idx << ") -> ";
            for (size_t j = 0; j < route.point_indices.size(); ++j) {
                std::cout << route.point_indices[j] << (j == route.point_indices.size() - 1 ? "" : " -> ");
            }
            std::cout << " -> Meta(" << end_depot_idx << ")";
            std::cout << " (Dystans: " << std::fixed << std::setprecision(2) << route.total_distance << ")" << std::endl;
            
        }



        std::vector<std::pair<int, int>> all_edges;
        for (int i = 0; i < num_total_points_in_matrix; ++i) {
            for (int j = 0; j < num_total_points_in_matrix; ++j) {
                if (i != j) {
                    all_edges.emplace_back(i, j);
                }
            }
        }

        std::ofstream csv_file("edge_route_matrix.csv");
        if (!csv_file.is_open()) {
            std::cerr << "Nie można otworzyć pliku edge_route_matrix.csv do zapisu\n";
        }
        else {
            csv_file << "Edge";
            for (size_t t = 0; t < routes.size(); ++t) {
                csv_file << ",T" << (t + 1);
            }
            csv_file << "\n";

            for (const auto& edge : all_edges) {
                csv_file << edge.first << "->" << edge.second;

                for (size_t t = 0; t < routes.size(); ++t) {
                    const auto& route = routes[t];
                    bool used = false;
                    if (!route.point_indices.empty() && edge == std::make_pair(start_depot_idx, route.point_indices.front())) {
                        used = true;
                    }
                    else if (!route.point_indices.empty() && edge == std::make_pair(route.point_indices.back(), end_depot_idx)) {
                        used = true;
                    }
                    else {
                        for (size_t i = 0; i + 1 < route.point_indices.size(); ++i) {
                            if (edge == std::make_pair(route.point_indices[i], route.point_indices[i + 1])) {
                                used = true;
                                break;
                            }
                        }
                    }
                    csv_file << "," << (used ? "1" : "0");
                }
                csv_file << "\n";
            }

            csv_file.close();
            std::cout << "\nMacierz krawedz-trasa zapisana do pliku edge_route_matrix.csv\n";
        }

    }
    catch (const std::exception& e) {
        std::cerr << "Wyjątek: " << e.what() << std::endl;
    }
    env.end();
    return 0;
}


