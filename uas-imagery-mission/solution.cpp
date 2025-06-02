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
    const std::vector<CustomerNode>& customers,
    const std::vector<std::vector<double>>& dist_matrix) {

    if (customers.empty()) return {};

    std::vector<Route> current_routes;
    for (const auto& customer : customers) {
        Route initial_route;
        initial_route.point_indices.push_back(customer.id);
        current_routes.push_back(initial_route);
    }

    std::vector<DirectedSaving> savings_list;
    for (const auto& cust_i : customers) {
        for (const auto& cust_j : customers) {
            if (cust_i.id == cust_j.id) continue;
            double saving_value =
                dist_matrix[cust_i.id][end_depot_idx] +
                dist_matrix[start_depot_idx][cust_j.id] -
                dist_matrix[cust_i.id][cust_j.id];
            if (saving_value > 0) {
                savings_list.push_back({ cust_i.id, cust_j.id, saving_value });
            }
        }
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
                    break;
                }
            }

            if (!has_overlap) {
                r1.point_indices.insert(
                    r1.point_indices.end(),
                    r2.point_indices.begin(),
                    r2.point_indices.end()
                );
                r2.point_indices.clear();  // Oznacz r2 jako scaloną
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

    if (final_routes.size() != 1 || final_routes[0].point_indices.size() != customers.size()) {
        std::cerr << "Błąd: nie udało się znaleźć jednej pełnej trasy obejmującej wszystkie punkty.\n";
    }

    return final_routes;
}

int main_cplex_integration_example() {
    IloEnv env;
    try {
        /*const int num_total_points_in_matrix = 36;
        const int start_depot_idx = 0;
        const int end_depot_idx = 35;
        const int num_waypoints = 34;
        const char* filename = "uas-cpp-data — kopia.dat";*/
        const int num_total_points_in_matrix = 34;
        const int start_depot_idx = 0;
        const int end_depot_idx = 33;
        const int num_waypoints = 32;
        const char* filename = "uas-cpp-data.dat";

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

        std::vector<Route> routes = clarke_wright_separate_depots(
            start_depot_idx,
            end_depot_idx,
            customers_list,
            dist_matrix_std
        );

        std::cout << "Wygenerowane Trasy:" << std::endl;
        double total_distance_all_routes = 0;
        for (size_t i = 0; i < routes.size(); ++i) {
            const auto& route = routes[i];
            std::cout << "Trasa " << i + 1 << ": ";
            std::cout << "Start(" << start_depot_idx << ") -> ";
            for (size_t j = 0; j < route.point_indices.size(); ++j) {
                std::cout << route.point_indices[j] << (j == route.point_indices.size() - 1 ? "" : " -> ");
            }
            std::cout << " -> Meta(" << end_depot_idx << ")";
            std::cout << " (Dystans: " << std::fixed << std::setprecision(2) << route.total_distance << ")" << std::endl;
            total_distance_all_routes += route.total_distance;
        }

        std::cout << "\nCałkowity dystans: " << total_distance_all_routes << std::endl;

        // --- NOWY KOD: wypisywanie macierzy krawędź-trasa ---

        // 1. Zbierz WSZYSTKIE możliwe krawędzie między punktami (w tym start i meta)
        std::vector<std::pair<int, int>> all_edges;
        for (int i = 0; i < num_total_points_in_matrix; ++i) {
            for (int j = 0; j < num_total_points_in_matrix; ++j) {
                if (i != j) {
                    all_edges.emplace_back(i, j);
                }
            }
        }

        // 2. Wypisz nagłówki kolumn (trasy)
        std::cout << "\nMacierz incydencji krawędź-trasa (wiersz: krawędź (from->to), kolumna: trasa):\n";
        std::cout << std::setw(12) << "Krawędź";
        for (size_t t = 0; t < routes.size(); ++t) {
            std::cout << std::setw(6) << ("T" + std::to_string(t + 1));
        }
        std::cout << std::endl;

        // --- OTWÓRZ PLIK CSV ---
        std::ofstream csv_file("edge_route_matrix.csv");
        if (!csv_file.is_open()) {
            std::cerr << "Nie można otworzyć pliku edge_route_matrix.csv do zapisu\n";
        }
        else {
            // Zapis nagłówka do CSV
            csv_file << "Edge";
            for (size_t t = 0; t < routes.size(); ++t) {
                csv_file << ",T" << (t + 1);
            }
            csv_file << "\n";

            // 3. Dla każdej krawędzi i każdej trasy wypisz 1/0 na konsolę i do pliku CSV
            for (const auto& edge : all_edges) {
                std::cout << std::setw(3) << edge.first << "->" << std::setw(3) << edge.second;
                csv_file << edge.first << "->" << edge.second;

                for (size_t t = 0; t < routes.size(); ++t) {
                    const auto& route = routes[t];
                    bool used = false;

                    // Sprawdź, czy krawędź jest użyta w trasie:
                    // uwzględniając start i meta

                    // Start -> pierwszy punkt
                    if (!route.point_indices.empty() && edge == std::make_pair(start_depot_idx, route.point_indices.front())) {
                        used = true;
                    }
                    // Ostatni punkt -> meta
                    else if (!route.point_indices.empty() && edge == std::make_pair(route.point_indices.back(), end_depot_idx)) {
                        used = true;
                    }
                    else {
                        // Sprawdź połączenia między punktami w trasie
                        for (size_t i = 0; i + 1 < route.point_indices.size(); ++i) {
                            if (edge == std::make_pair(route.point_indices[i], route.point_indices[i + 1])) {
                                used = true;
                                break;
                            }
                        }
                    }

                    std::cout << std::setw(6) << (used ? 1 : 0);
                    csv_file << "," << (used ? "1" : "0");
                }
                std::cout << std::endl;
                csv_file << "\n";
            }

            csv_file.close();
            std::cout << "\nMacierz krawędź-trasa zapisana do pliku edge_route_matrix.csv\n";
        }

    }
    catch (const std::exception& e) {
        std::cerr << "Wyjątek: " << e.what() << std::endl;
    }
    env.end();
    return 0;
}


