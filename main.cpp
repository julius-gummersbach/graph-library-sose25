#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cassert>

#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"
#include "graph/AdjacentMatrixGraph.h"
#include "graph/AdjacentListGraph.h"

#include "0x01.cpp"
#include "0x02.cpp"
#include "0x03.cpp"

using namespace std;
using edge_t = graph::SuperGraph::edge_t;

const string INPUT_DIR = "../input/";

void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult);
double evaluatePrim(graph::SuperGraph &graph);
double evaluateKruskal(graph::SuperGraph &graph);
double evaluateNearestNeighbor(graph::SuperGraph &graph);
double evaluateDoubleTree(graph::SuperGraph &graph);
double evaluateTspBruteForce(graph::SuperGraph &graph);
double evaluateTspBnB(graph::SuperGraph &graph);


int main() {
  // 0x01, connected components
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x01, Connected Components" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph1.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 2);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph2.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph3.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph4.txt", new graph::AdjacentListGraph(), getConnectedComponents, 222);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph5.txt", new graph::AdjacentListGraph(), getConnectedComponents, 9560);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph6.txt", new graph::AdjacentListGraph(), getConnectedComponents, 306);

  // 0x02, prim
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x02, prim" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentListGraph(), evaluatePrim, 287.32286);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_20.txt", new graph::AdjacentListGraph(), evaluatePrim, 36.86275);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 12.68182);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_20.txt", new graph::AdjacentListGraph(), evaluatePrim, 2785.62417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 372.14417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_100_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 27550.51488);

  // 0x02, kruskal
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x02, kruskal" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::EdgeListGraph(), evaluateKruskal, 287.32286);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_20.txt", new graph::EdgeListGraph(), evaluateKruskal, 36.86275);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 12.68182);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_20.txt", new graph::EdgeListGraph(), evaluateKruskal, 2785.62417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 372.14417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_100_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 27550.51488);

  // 0x03, nearest neighbor
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x03, nearest neighbor" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10e.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12e.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15e.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_20.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_30.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_50.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_70.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_100.txt", new graph::AdjacentListGraph(), evaluateNearestNeighbor, 1);

  // 0x03, double tree
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x03, double tree" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10e.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12e.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15e.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_20.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_30.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_50.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_70.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_100.txt", new graph::AdjacentListGraph(), evaluateDoubleTree, 1);

  // 0x03, TSP Brute-Force und BnB
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x03, TSP Brute-Force und BnB" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_20.txt", new graph::AdjacentMatrixGraph(), evaluateTspBnB, 1);  // 1.5s
  // K_30 would take a couple million years...

  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_10e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_12e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  /* these would be expected to take roughly 30min
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  assertFunctionOnGraph(INPUT_DIR + "0x03/K_15e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1); */

  return 0;
}

/**
 * Asserts if the expected results occurs and prints execution times to std::cout
 * @param inputFile file to read input from into graph
 * @param graph graph object to use for calculation
 * @param function a function that runs on a graph object and returns a double
 * @param expectedResult expected result of function if run on graph
 */
void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult) {
  cout << "Reading from file " << inputFile << endl;
  auto start = chrono::high_resolution_clock::now();
  ifstream file(inputFile);
  if (!file) {
    cerr << "Failed to open input file: " << inputFile << endl;
    return;
  }
  // Read the entire file into memory
  ostringstream buffer;
  buffer << file.rdbuf();
  istringstream input(buffer.str());
  graph->initializeFromInput(input);
  
  cout << "Starting calculation..." << endl;
  auto setupComplete = chrono::high_resolution_clock::now();

  const double result = (*function)(*graph);
  auto end = chrono::high_resolution_clock::now();
  auto setupDuration = chrono::duration_cast<chrono::milliseconds>(setupComplete - start);
  auto calcDuration = chrono::duration_cast<chrono::milliseconds>(end - setupComplete);

  cout << "Set up: " << setupDuration.count() << " ms" << endl;
  cout << "Calculation: " << calcDuration.count() << " ms" << endl;
  cout << "Total time: " << (setupDuration + calcDuration).count() << " ms" << endl;
  cout << "Result: " << result << " expected: " << expectedResult << endl;

  assert(("Test failed: Result does not match expected value", abs(result - expectedResult) < 0.000001));
  cout << "============================" << endl << endl;

  delete graph;
}

/**
 * @return weight of the result of Prim if executed on graph
 */
double evaluatePrim(graph::SuperGraph &graph) {
  const auto mst = getMSTPrim(graph);
  double weight = 0;
  for (const auto edge: mst) {
    weight += get<2>(edge);
  }
  return weight;
}

/**
 * @return weight of the result of Kruskal if executed on graph
 */
double evaluateKruskal(graph::SuperGraph &graph) {
  const auto mst = getMSTKruskal(graph);
  double weight = 0;
  for (const auto edge: mst) {
    weight += get<2>(edge);
  }
  return weight;
}

/**
 * checks, if the given edge list is a hamilton-circle and prints it and its weight to std::cout
 * @param circle an edge list
 * @return 1 if circle is a hamilton circle, 0 otherwise
 */
double printAndCheckHamilton(vector<edge_t> circle) {
  const int startNode = get<0>(circle[0]);
  double weight = 0;
  int current = startNode;
  cout << "Hamiltonian evaluation: ";
  for (const auto edge: circle) {
    if (get<0>(edge) != current) return 0;
    cout << get<0>(edge) << "-";
    weight += get<2>(edge);
    current = get<1>(edge);
  }
  if (get<1>(circle.at(circle.size()-1)) != startNode) return 0;
  cout << startNode << endl;
  cout << "Weight: " << weight << endl;
  return 1;
}

/**
 * @return weight of the result of NearestNeighbor if executed on graph
 */
double evaluateNearestNeighbor(graph::SuperGraph &graph) {
  const auto hamilton = nearestNeighbors(graph, 0);
  if (hamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(hamilton);
}

/**
 * @return weight of the result of DoubleTree if executed on graph
 */
double evaluateDoubleTree(graph::SuperGraph &graph) {
  const auto hamilton = doubleTreeAlgorthm(graph);
  if (hamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(hamilton);
}

/**
 * @return weight of the result of TspBruteForce if executed on graph with branch and bound disabled
 */
double evaluateTspBruteForce(graph::SuperGraph &graph) {
  if (graph.numNodes > 20) cout << "This is a bad idea...";
  auto shortestHamilton = bruteForceTsp(graph, false);
  if (shortestHamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(shortestHamilton);
}

/**
 * @return weight of the result of TspBruteForce if executed on graph with branch and bound enabled
 */
double evaluateTspBnB(graph::SuperGraph &graph) {
  auto shortestHamilton = bruteForceTsp(graph, true);
  if (shortestHamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(shortestHamilton);
}
