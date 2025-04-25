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

using namespace std;
using edge_t = graph::SuperGraph::edge_t;

const string INPUT_DIR = "../input/";

void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult);
double evaluatePrim(graph::SuperGraph &graph);
double evaluateKruskal(graph::SuperGraph &graph);

int main() {
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph1.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 2);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph2.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph3.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph4.txt", new graph::AdjacentListGraph(), getConnectedComponents, 222);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph5.txt", new graph::AdjacentListGraph(), getConnectedComponents, 9560);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph6.txt", new graph::AdjacentListGraph(), getConnectedComponents, 306);

  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentListGraph(), evaluatePrim, 287.32286);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_20.txt", new graph::AdjacentListGraph(), evaluatePrim, 36.86275);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 12.68182);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_20.txt", new graph::AdjacentListGraph(), evaluatePrim, 2785.62417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 372.14417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_100_200.txt", new graph::AdjacentListGraph(), evaluatePrim, 27550.51488);

  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::EdgeListGraph(), evaluateKruskal, 287.32286);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_20.txt", new graph::EdgeListGraph(), evaluateKruskal, 36.86275);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 12.68182);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_20.txt", new graph::EdgeListGraph(), evaluateKruskal, 2785.62417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 372.14417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_100_200.txt", new graph::EdgeListGraph(), evaluateKruskal, 27550.51488);

  return 0;
}

void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult) {
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
  
  cout << "Done reading the graph, starting calculation task..." << endl;
  auto setupComplete = chrono::high_resolution_clock::now();

  const double result = (*function)(*graph);
  auto end = chrono::high_resolution_clock::now();
  auto setupDuration = chrono::duration_cast<chrono::milliseconds>(setupComplete - start);
  auto calcDuration = chrono::duration_cast<chrono::milliseconds>(end - setupComplete);

  cout << "Time taken to set up the graph: " << setupDuration.count() << " ms" << endl;
  cout << "Time taken to calculate task: " << calcDuration.count() << " ms" << endl;
  cout << "Total time taken: " << (setupDuration + calcDuration).count() << " ms" << endl;

  assert(("Test failed: Result does not match expected value", result - expectedResult < 0.000001));
  delete graph;
}

double evaluatePrim(graph::SuperGraph &graph) {
  const auto mst = getMSTPrim(graph);
  double weight = 0;
  for (const auto edge: mst) {
    weight += get<2>(edge);
  }
  return weight;
}

double evaluateKruskal(graph::SuperGraph &graph) {
  const auto mst = getMSTKruskal(graph);
  double weight = 0;
  for (const auto edge: mst) {
    weight += get<2>(edge);
  }
  return weight;
}
