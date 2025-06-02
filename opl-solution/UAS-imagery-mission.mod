/*********************************************
 * OPL 22.1.2.0 Model
 * Author: xiii5
 * Creation Date: 11 maj 2025 at 10:01:57
 *********************************************/

int n = ...; // liczba przystanków
int k = ...; //liczba pojazdów
range NODES = 0..n+1; //zakres przystanków + start + koniec
range WAYPOINTS = 1..n; //zakres przystanków
range VEHICLES = 0..k-1; //zakres pojazdów

// Parametry
float D = ...; // maksymalny dystans dla pojazdu
float c[NODES][NODES][VEHICLES] = ...;
float d[NODES][NODES][VEHICLES] = ...; // dystans od i do j dla pojazdu k, przyjmujemy że w tym przypadku koszt to po prostu dystans


dvar boolean x[NODES][NODES][VEHICLES]; //zmienna przyjmująca 0 albo 1, opisująca czy k przebywa drogę od i do j
dvar float+ y[NODES][NODES][VEHICLES];
dvar int mu[WAYPOINTS][VEHICLES] in 1..n;

dvar float+ totalDistance;

//cel - minimalizacja całkowitego kosztu (1)
minimize
  sum(i in NODES, j in NODES : i != j)
    c[i][j][0] * x[i][j][0];


subject to {
  //każdy wierzchołek odwiedzony dokładnie 1 (2)
	forall(i in WAYPOINTS)
		sum(j in NODES : i != j)
      	x[i][j][0] == 1;
   
   		// ograniczenie, aby nie było bezpośredniego połączenia między startem a metą
   		// x[0][n+1][0] == 0;

		// każdy pojazd zaczyna w wierzchołku 0 (3)
		sum(j in NODES : j != 0)
		x[0][j][0] == 1;
      
 	// pojazd opuszcza każdy odwiedzony wierzchołek (4)
	forall(h in WAYPOINTS)
    	sum(i in NODES : i != h)
      	x[i][h][0] - sum(j in NODES : j != h)
      	x[h][j][0] == 0;

    	// Każdy pojazd kończy w n + 1 (5)
		sum(i in NODES : i != n+1)
		x[i][n+1][0] == 1;
      
	
	//(6) przerobione na MTZ!
	forall(i in WAYPOINTS, k in VEHICLES)
		mu[i][k] <= n;
	 
	forall(i in WAYPOINTS, j in WAYPOINTS : i != j, k in VEHICLES)
		mu[j][k] >= mu[i][k] + 1 - n * (1 - x[i][j][k]);
 

	// Distance constraints
	//(7)
	forall(j in NODES : j != 0) 
	    y[0][j][0] == d[0][j][0] * x[0][j][0];
	
	//(8)
	forall(i in NODES, j in NODES : i != j)
		y[i][j][0] <= (D - d[j][0][0]) * x[i][j][0];
	
	//(9)
	forall(i in NODES : i != n+1)
		y[i][n+1][0] <= D * x[i][n+1][0];
	
	//(10)
	forall(i in NODES, j in NODES : i != j)
		y[i][j][0] >= (d[0][i][0] + d[i][j][0]) * x[i][j][0];
	    
	
	//zmienna do liczenia dlugosci trasy
	totalDistance == 
		sum(i in NODES, j in NODES : i != j)
			d[i][j][0] * x[i][j][0]; 
}







 