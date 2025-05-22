#include "solution.h"
#include "helper_functions.h"

ILOSTLBEGIN

#define RC_EPS 1.0e-6

int uas_imagery_mission() {
    IloEnv env;
    IloModel model(env);

    int n = 32;
    int k = 1;
    IloNum D = 100.0;
    IloArray<IloNumArray> d;
    IloArray<IloNumArray> c;

    const char* filename = "./uas-cpp-data.dat";
    readData(filename, d, c, env);

    printMatrix(c, n, "d");

    try {
        readData(filename, d, c, env);

        printMatrix(c, n, "d");
    } catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }

    env.end();
    return 0;
}

