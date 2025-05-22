#pragma once
#include <iostream>
#include "ilcplex\cplex.h"
#include "ilcplex\ilocplex.h"
#include "ilopl\iloopl.h"

using namespace std;
class MyIloException : public IloException {
public:
    MyIloException(const char* msg);
};
void readData(const char* filename, IloArray<IloNumArray>& d, IloArray<IloNumArray>& c, IloEnv& env);
void printMatrix(const IloArray<IloNumArray>& mat, int n, const char* name);