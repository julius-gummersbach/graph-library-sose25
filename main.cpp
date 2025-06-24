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

using namespace std;

#include "0x01.cpp"
#include "0x02.cpp"
#include "0x03.cpp"
#include "0x04.cpp"
#include "0x05.cpp"
#include "0x06.cpp"

const string INPUT_DIR = "../input/";

void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult, bool directed = false);
double evaluatePrim(graph::SuperGraph &graph);
double evaluateKruskal(graph::SuperGraph &graph);
double evaluateNearestNeighbor(graph::SuperGraph &graph);
double evaluateDoubleTree(graph::SuperGraph &graph);
double evaluateTspBruteForce(graph::SuperGraph &graph);
double evaluateTspBnB(graph::SuperGraph &graph);
double evaluateDijkstra2to0(graph::SuperGraph &graph);
double evaluateDijkstra0to1(graph::SuperGraph &graph);
double evaluateMooreBellmanFord2to0(graph::SuperGraph &graph);
double evaluateMooreBellmanFord0to1(graph::SuperGraph &graph);
double evaluateEdmondsKarp0to7(graph::SuperGraph &graph);
double evaluateCycleCancelling(graph::SuperGraph &graph);
double evaluateSuccessiveShortestPath(graph::SuperGraph &graph);


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
  // these would be expected to take roughly 30min
  // assertFunctionOnGraph(INPUT_DIR + "0x03/K_15.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);
  // assertFunctionOnGraph(INPUT_DIR + "0x03/K_15e.txt", new graph::AdjacentMatrixGraph(), evaluateTspBruteForce, 1);

  // 0x04, Shortest Paths Dijkstra
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x04, Shortest Paths Dijkstra" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x04/Wege1.txt", new graph::AdjacentListGraph(), evaluateDijkstra2to0, 6, true);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentListGraph(), evaluateDijkstra0to1, 5.56283, true);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentListGraph(), evaluateDijkstra0to1, 2.36802, false);
  assertFunctionOnGraph(INPUT_DIR + "0x04/WegeCustom.txt", new graph::AdjacentListGraph(), evaluateDijkstra2to0, 6, true);

  // 0x04, Shortest Paths Moore-Bellman-Ford
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x04, Shortest Paths Moore-Bellman-Ford" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x04/Wege1.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord2to0, 6, true);
  assertFunctionOnGraph(INPUT_DIR + "0x04/Wege2.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord2to0, 2, true);
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x04/Wege3.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord2to0, NAN, true);
  } catch (const std::invalid_argument& e) {
    cout << "Negative Cycle detected successfully: " << e.what() << endl << endl << endl;
  }
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord0to1, 5.56283, true);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord0to1, 2.36802, false);
  assertFunctionOnGraph(INPUT_DIR + "0x04/WegeCustom.txt", new graph::EdgeListGraph(), evaluateMooreBellmanFord2to0, 6, true);

  // 0x05, Max Flow EdmondsKarp
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x05, Max Flow EdmondsKarp" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x05/Fluss.txt", new graph::AdjacentMatrixGraph(), evaluateEdmondsKarp0to7, 4, true);
  assertFunctionOnGraph(INPUT_DIR + "0x05/Fluss2.txt", new graph::AdjacentMatrixGraph(), evaluateEdmondsKarp0to7, 5, true);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentMatrixGraph(), evaluateEdmondsKarp0to7, 0.75447, true);

  // 0x06, Cycle-Cancelling
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x06, Cycle-Cancelling" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal1.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 3, true);
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal2.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 0, true);
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal3.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal4.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross1.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 1537, true);
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross2.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 1838, true);
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross3.txt", new graph::EdgeListGraph(), evaluateCycleCancelling, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }

  // 0x06, Successive-Shortest-Path
  cout << "############################################" << endl;
  cout << "############################################" << endl;
  cout << "0x06, Successive-Shortest-Path" << endl;
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal1.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 3, true);
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal2.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 0, true);
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal3.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal4.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross1.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 1537, true);
  assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross2.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 1838, true);
  try {
    assertFunctionOnGraph(INPUT_DIR + "0x06/Kostenminimal_gross3.txt", new graph::EdgeListGraph(), evaluateSuccessiveShortestPath, 3, true);
  } catch (const std::invalid_argument& e) {
    cout << "No b-flow possible detected successfully: " << e.what() << endl << endl << endl;
  }
  return 0;
}

/**
 * Asserts if the expected results occurs and prints execution times to std::cout
 * @param inputFile file to read input from into graph
 * @param graph graph object to use for calculation
 * @param function a function that runs on a graph object and returns a double
 * @param expectedResult expected result of function if run on graph
 * @param directed if the graph should be read as directed
 */
void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), const double expectedResult, const bool directed) {
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
  graph->directed = directed;
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

  assert(("Test failed: Result does not match expected value", abs(result - expectedResult) < 0.00001));
  cout << "============================" << endl << endl;

  delete graph;
}

/**
 * @return weight of the result of Prim if executed on graph
 */
double evaluatePrim(graph::SuperGraph &graph) {
  const auto mst = getMSTPrim(graph);
  double weight = 0;
  for (const auto& edge: mst) {
    weight += edge->getCost();
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
    weight += edge->getCost();
  }
  return weight;
}

/**
 * checks, if the given edge list is a hamilton-circle and prints it and its cost to std::cout
 * @param circle an edge list
 * @param directed whether the check should assume a directed graph
 * @return 1 if circle is a hamilton circle, 0 otherwise
 */
double printAndCheckHamilton(const vector<shared_ptr<const edge::CostCapEdge>> &circle, const bool directed) {
  const int startNode = circle[0]->getFrom();
  double weight = 0;
  int current = startNode;
  cout << "Hamiltonian evaluation: ";
  for (const auto& edge: circle) {
    if (edge->getFrom() != current && (directed || edge->getTo() != current)) return 0;
    int from = edge->getFrom();
    int to = edge->getTo();
    if (!directed && edge->getTo() == current) {
      swap(from, to);
    }
    cout << from << "-";
    weight += edge->getCost();
    current = to;
  }
  if (circle.at(circle.size()-1)->getTo() != startNode && (directed || circle.at(circle.size()-1)->getFrom() != startNode)) return 0;
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
  return printAndCheckHamilton(hamilton, graph.directed);
}

/**
 * @return weight of the result of DoubleTree if executed on graph
 */
double evaluateDoubleTree(graph::SuperGraph &graph) {
  const auto hamilton = doubleTreeAlgorthm(graph);
  if (hamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(hamilton, graph.directed);
}

/**
 * @return weight of the result of TspBruteForce if executed on graph with branch and bound disabled
 */
double evaluateTspBruteForce(graph::SuperGraph &graph) {
  if (graph.numNodes > 20) cout << "This is a bad idea...";
  auto shortestHamilton = bruteForceTsp(graph, false);
  if (shortestHamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(shortestHamilton, graph.directed);
}

/**
 * @return weight of the result of TspBruteForce if executed on graph with branch and bound enabled
 */
double evaluateTspBnB(graph::SuperGraph &graph) {
  auto shortestHamilton = bruteForceTsp(graph, true);
  if (shortestHamilton.size() != graph.numNodes) return 0;
  return printAndCheckHamilton(shortestHamilton, graph.directed);
}

double checkAndPrintPath(const vector<shared_ptr<const edge::CostCapEdge>> &path) {
  int currentNode = path[0]->getFrom();
  double weight = 0;
  cout << currentNode;
  for (const auto& edge: path) {
    if (edge->getFrom() != currentNode) return false;
    currentNode = edge->getTo();
    weight += edge->getCost();
    cout << "-" << currentNode;
  }
  cout << ". Weight: " << weight << endl;
  return weight;
}

/**
 * @return weight of the shortest path from "from" to "to"
 */
double evaluateDijkstra(graph::SuperGraph &graph, const int from, const int to) {
  double weightFromTo = NAN;
  auto shortestPaths = dijkstra(graph, from);
  for (int i = 0; i < shortestPaths.size(); i++) {
    if (i == from) continue;
    cout << "Path from " << from << " to " << i << ": ";
    const auto weight = checkAndPrintPath(shortestPaths[i]);
    if (i == to) weightFromTo = weight;
  }
  return weightFromTo;
}

/**
 * @return weight of the shortest path from 2 to 0
 */
double evaluateDijkstra2to0(graph::SuperGraph &graph) {
  return evaluateDijkstra(graph, 2, 0);
}

/**
 * @return weight of the shortest path from 0 to 1
 */
double evaluateDijkstra0to1(graph::SuperGraph &graph) {
  return evaluateDijkstra(graph, 0, 1);
}


/**
 * @return weight of the shortest path from "from" to "to"
 */
double evaluateMooreBellmanFord(graph::SuperGraph &graph, const int from, const int to) {
  double weightFromTo = NAN;
  auto reached = vector(graph.numNodes, false);
  reached[from] = true;
  auto shortestPaths = mooreBellmanFord(graph, from);
  if (shortestPaths.contains(-1)) {
    throw std::invalid_argument("Negative Cycle in graph detected");
  }
  for (auto& p : shortestPaths) {
    if (p.first == from) continue;
    reached[p.first] = true;
    cout << "Path from " << from << " to " << p.first << ": ";
    const auto weight = checkAndPrintPath(p.second);
    if (p.first == to) weightFromTo = weight;
  }
  auto prefix = "Unreachable nodes: ";
  auto suffix = "All nodes have been reached.";
  for (int i = 0; i < graph.numNodes; i++) {
    if (!reached[i]) {
      cout << prefix << i;
      prefix = ", ";
      suffix = ".";
    }
  }
  cout << suffix << endl;
  return weightFromTo;
}

/**
 * @return weight of the shortest path from 2 to 0
 */
double evaluateMooreBellmanFord2to0(graph::SuperGraph &graph) {
  return evaluateMooreBellmanFord(graph, 2, 0);
}

/**
 * @return weight of the shortest path from 0 to 1
 */
double evaluateMooreBellmanFord0to1(graph::SuperGraph &graph) {
  return evaluateMooreBellmanFord(graph, 0, 1);
}

double evaluateEdmondsKarp0to7(graph::SuperGraph &graph) {
  return get<0>(edmondsKarp(graph, 0, 7));
}

double evaluateCycleCancelling(graph::SuperGraph &graph) {
  return cycleCancelling(graph);
}

double evaluateSuccessiveShortestPath(graph::SuperGraph &graph) {
  return successiveShortestPath(graph);
}
