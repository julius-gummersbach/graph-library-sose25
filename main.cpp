#include <iostream>
#include <fstream>
#include <set>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <stack>
#include <cassert>

#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"
#include "graph/AdjacentMatrixGraph.h"
#include "graph/AdjacentListGraph.h"

using namespace std;


const string INPUT_DIR = "../input/";

std::vector<int> depthFirstSearch(graph::SuperGraph &graph, int startNode, vector<bool> &visited);
void assertFunctionOnGraph(const std::string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult);
double getConnectedComponents(graph::SuperGraph &graph);
double getMSTWeight(graph::SuperGraph &graph);
std::vector<graph::EdgeListGraph::edge_t> getMSTPrim(graph::SuperGraph &graph);


int main() {
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph1.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 2);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph2.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph3.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph4.txt", new graph::AdjacentListGraph(), getConnectedComponents, 222);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph5.txt", new graph::AdjacentListGraph(), getConnectedComponents, 9560);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph6.txt", new graph::AdjacentListGraph(), getConnectedComponents, 306);

  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_2.txt", new graph::AdjacentListGraph(), getMSTWeight, 287.32286);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_20.txt", new graph::AdjacentListGraph(), getMSTWeight, 36.86275);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_1_200.txt", new graph::AdjacentListGraph(), getMSTWeight, 12.68182);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_20.txt", new graph::AdjacentListGraph(), getMSTWeight, 2785.62417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_10_200.txt", new graph::AdjacentListGraph(), getMSTWeight, 372.14417);
  assertFunctionOnGraph(INPUT_DIR + "0x02/G_100_200.txt", new graph::AdjacentListGraph(), getMSTWeight, 27550.51488);

  return 0;
}

void assertFunctionOnGraph(const std::string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult) {
  auto start = std::chrono::high_resolution_clock::now();
  std::ifstream file(inputFile);
  if (!file) {
    std::cerr << "Failed to open input file: " << inputFile << std::endl;
    return;
  }
  // Read the entire file into memory
  std::ostringstream buffer;
  buffer << file.rdbuf();
  std::istringstream input(buffer.str());
  graph->initializeFromInput(input);
  
  cout << "Done reading the graph, starting calculation task..." << endl;
  auto setupComplete = std::chrono::high_resolution_clock::now();

  const double result = (*function)(*graph);
  auto end = std::chrono::high_resolution_clock::now();
  auto setupDuration = std::chrono::duration_cast<std::chrono::milliseconds>(setupComplete - start);
  auto calcDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - setupComplete);

  std::cout << "Time taken to set up the graph: " << setupDuration.count() << " ms" << std::endl;
  std::cout << "Time taken to calculate task: " << calcDuration.count() << " ms" << std::endl;
  std::cout << "Total time taken: " << (setupDuration + calcDuration).count() << " ms" << std::endl;

  assert(("Test failed: Result does not match expected value", result - expectedResult < 0.000001));
  delete graph;
}

double getConnectedComponents(graph::SuperGraph &graph) {
  double connectedComponents = 0;
  vector visited(graph.numNodes, false);

  for (int node = 0; node < graph.numNodes; node++) {
    if (visited[node]) continue;
    connectedComponents++;
    depthFirstSearch(graph, node, visited);
  }
  return connectedComponents;
}

double getMSTWeight(graph::SuperGraph &graph) {
  const auto mst = getMSTPrim(graph);
  double weight = 0;
  for (const auto edge: mst) {
    weight += graph.getWeight(edge);
  }
  return weight;
}

std::vector<graph::SuperGraph::edge_t> getMSTPrim(graph::SuperGraph &graph) {
  using edge_t = graph::SuperGraph::edge_t;

  auto weightFunc = [&](const edge_t &e) {
    return graph.getWeight(e);  // if 'graph' is passed in and has getWeight
  };

  auto cmp = [&](const edge_t &a, const edge_t &b) {
    return weightFunc(a) < weightFunc(b);
  };
  std::priority_queue<edge_t, std::vector<edge_t>, decltype(cmp)> pq(cmp);
  for (const auto node: graph.getAdjacentNodes(0)) {
    pq.push(std::array{0, node});
  }

  std::vector<graph::SuperGraph::edge_t> mst;
  std::vector found(graph.numNodes, false);
  found[0] = true;
  while (mst.size() < graph.numNodes - 1) {
    auto edge = pq.top();
    pq.pop();
    if (!found[edge[1]]) {
      found[edge[1]] = true;
      mst.push_back(edge);
      for (const auto node: graph.getAdjacentNodes(edge[1])) {
        if (found[node]) continue;
        pq.push(std::array{edge[1], node});
      }
    }
  }
  return mst;
}

std::vector<int> depthFirstSearch(graph::SuperGraph &graph, const int startNode, vector<bool> &visited) {
  std::vector<int> subGraph;
  subGraph.reserve(graph.numNodes / 2);
  std::stack<int> stack{};
  stack.push(startNode);
  while (!stack.empty()) {
    int currentNode = stack.top();
    stack.pop();
    subGraph.push_back(currentNode);
    visited[currentNode] = true;
    for (const vector<int> &adjacent = graph.getAdjacentNodes(currentNode); int node: adjacent) {
      if (visited[node]) continue;
      stack.push(node);
    }
  }
  return subGraph;
}
