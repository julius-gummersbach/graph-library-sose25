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
#include "helper/UnionFind.h"

using namespace std;


const string INPUT_DIR = "../input/";

using edge_t = graph::SuperGraph::edge_t;
vector<int> depthFirstSearch(graph::SuperGraph &graph, int startNode, vector<bool> &visited);
void assertFunctionOnGraph(const string& inputFile, graph::SuperGraph* graph, double function(graph::SuperGraph&), double expectedResult);
double getConnectedComponents(graph::SuperGraph &graph);
double evaluatePrim(graph::SuperGraph &graph);
double evaluateKruskal(graph::SuperGraph &graph);
vector<edge_t> getMSTPrim(graph::SuperGraph &graph);
vector<edge_t> getMSTKruskal(graph::SuperGraph &graph);

auto edgeComparator = [](const edge_t& lhs, const edge_t& rhs)
{
  return get<2>(lhs) > get<2>(rhs);
};


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

vector<int> depthFirstSearch(graph::SuperGraph &graph, const int startNode, vector<bool> &visited) {
  vector<int> subGraph;
  subGraph.reserve(graph.numNodes / 2);
  stack<int> stack{};
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

vector<edge_t> getMSTPrim(graph::SuperGraph &graph) {
  priority_queue<edge_t, vector<edge_t>, decltype(edgeComparator)> pq(edgeComparator);
  for (const auto node: graph.getAdjacentNodes(0)) {
    pq.emplace(0, node, graph.getWeight(0, node));
  }

  vector<edge_t> mst;
  vector found(graph.numNodes, false);
  found[0] = true;
  while (mst.size() < graph.numNodes - 1) {
    auto edge = pq.top(); pq.pop();
    int v = get<1>(edge);
    if (!found[v]) {
      found[v] = true;
      mst.push_back(edge);
      for (const auto node: graph.getAdjacentNodes(v)) {
        if (found[node]) continue;
        pq.emplace(v, node, graph.getWeight(v, node));
      }
    }
  }
  return mst;
}

vector<edge_t> getMSTKruskal(graph::SuperGraph &graph) {
  auto cmp = [](const edge_t& lhs, const edge_t& rhs)
  {
    return get<2>(lhs) > get<2>(rhs);
  };

  priority_queue<edge_t, vector<edge_t>, decltype(cmp)> pq(cmp);
  auto edges = graph.getEdges();
  for (const auto edge: edges) {
    pq.emplace(edge);
  }

  vector<edge_t> mst;
  helper::UnionFind uf(graph.numNodes);
  while (mst.size() < graph.numNodes - 1) {
    if (pq.empty()) {
      throw std::invalid_argument("Graph is not connected");
    }
    auto edge = pq.top(); pq.pop();
    if (uf.unionSets(get<0>(edge), get<1>(edge))) {
      mst.push_back(edge);
    }
  }
  return mst;
}
