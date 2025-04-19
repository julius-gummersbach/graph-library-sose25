#include <iostream>
#include <fstream>
#include <filesystem>
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
void assertFunctionOnGraph(const std::string& inputFile, graph::SuperGraph* graph, int function(graph::SuperGraph&), int expectedResult);
int getConnectedComponents(graph::SuperGraph &graph);


int main() {
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph1.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 2);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph2.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph3.txt", new graph::AdjacentMatrixGraph(), getConnectedComponents, 4);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph4.txt", new graph::AdjacentListGraph(), getConnectedComponents, 222);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph5.txt", new graph::AdjacentListGraph(), getConnectedComponents, 9560);
  assertFunctionOnGraph(INPUT_DIR + "0x01/Graph6.txt", new graph::AdjacentListGraph(), getConnectedComponents, 306);

  return 0;
}

void assertFunctionOnGraph(const std::string& inputFile, graph::SuperGraph* graph, int function(graph::SuperGraph&), int expectedResult) {
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
  
  cout << "Done reading the graph, starting to calculating task..." << endl;
  auto setupComplete = std::chrono::high_resolution_clock::now();

  const int result = (*function)(*graph);
  auto end = std::chrono::high_resolution_clock::now();
  auto setupDuration = std::chrono::duration_cast<std::chrono::milliseconds>(setupComplete - start);
  auto calcDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - setupComplete);

  std::cout << "Time taken to set up the graph: " << setupDuration.count() << " ms" << std::endl;
  std::cout << "Time taken to calculate task: " << calcDuration.count() << " ms" << std::endl;
  std::cout << "Total time taken: " << (setupDuration + calcDuration).count() << " ms" << std::endl;

  assert(("Test failed: Result does not match expected value", result == expectedResult));
  delete graph;
}

int getConnectedComponents(graph::SuperGraph &graph) {
  int connectedComponents = 0;
  vector visited(graph.numNodes, false);

  for (int node = 0; node < graph.numNodes; node++) {
    if (visited[node]) continue;
    connectedComponents++;
    depthFirstSearch(graph, node, visited);
  }
  return connectedComponents;
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
