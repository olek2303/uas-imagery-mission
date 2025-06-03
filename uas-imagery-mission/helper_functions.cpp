#include "helper_functions.h"

MyIloException::MyIloException(const char* msg) : IloException(msg, IloTrue) {}


void printMatrix(const IloArray<IloNumArray>& mat, int n, const char* name) {
    std::cout << "Matrix " << name << " (" << n << "x" << n << "):" << std::endl;
    for (int i = 0; i < n; i++) {
        cout << "===" << i << endl;
        for (int j = 0; j < n; j++) {
            std::cout << mat[i][j] << " ";
        }
        std::cout << std::endl << std::endl;
    }
}

void readData(const char* filename, IloArray<IloNumArray>& d, IloArray<IloNumArray>& c, IloEnv& env, int n) {
    std::ifstream file(filename);
    if (!file) {
        throw MyIloException("Cannot open data file");
    }

    d = IloArray<IloNumArray>(env, n);
    c = IloArray<IloNumArray>(env, n);
    for (int i = 0; i < n; i++) {
        d[i] = IloNumArray(env, n);
        c[i] = IloNumArray(env, n);
    }

    std::string line;
    for (int i = 0; i < n; i++) {
        std::getline(file, line);
        int idx = 0;
        for (int j = 0; j < n; j++) {
            const int number_length = 6;
            const int space_length = 1;
            
            std::string char_val = line.substr(idx, number_length);

            size_t first = char_val.find_first_not_of(' ');
            size_t last = char_val.find_last_not_of(' ');
            if (first != std::string::npos && last != std::string::npos)
                char_val = char_val.substr(first, last - first + 1);
            else
                char_val = "";

            double val = 0.0;
            if (!char_val.empty()) {
                try {
                    val = std::stod(char_val);
                }
                catch (const std::exception& e) {
                    std::cerr << "Blad konwersji '" << char_val << "' na double: " << e.what() << "\n";
                }
            }
            else {
                std::cerr << "Pusty substring w wierszu " << i << ", kolumna " << j << "\n";
            }

            d[i][j] = val;
            idx += number_length + space_length;
        }
        std::cout << std::endl;
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            c[i][j] = d[i][j];
        }
    }

    file.close();
}