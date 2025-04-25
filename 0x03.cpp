#include <cfloat>
#include <vector>

#include "graph/SuperGraph.h"

using namespace std;
using edge_t = graph::SuperGraph::edge_t;


vector<edge_t> nearestNeighbors(graph::SuperGraph &graph, int startNode) {
  auto edgeComparator = [](const edge_t& lhs, const edge_t& rhs)
  {
    return get<2>(lhs) > get<2>(rhs);
  };

  vector<edge_t> hamilton;
  hamilton.reserve(graph.numNodes);
  vector visited(graph.numNodes, false);

  int currentNode = startNode;
  visited[startNode] = true;
  while (hamilton.size() < graph.numNodes) {
    auto adjacent = graph.getAdjacentNodes(currentNode);
    priority_queue<edge_t, vector<edge_t>, decltype(edgeComparator)> sortedNeighbors(edgeComparator);
    for (const auto node: adjacent) {
      sortedNeighbors.emplace(currentNode, node, graph.getWeight(node, currentNode));
    }
    if (hamilton.size() == graph.numNodes - 1) {
      // find edge back to startNode to complete circle
      while (get<1>(sortedNeighbors.top()) != startNode) sortedNeighbors.pop();
    } else {
      // find shortest edge to unvisited node
      while (visited[get<1>(sortedNeighbors.top())]) sortedNeighbors.pop();
    }
    visited[get<1>(sortedNeighbors.top())] = true;
    hamilton.push_back(sortedNeighbors.top());
    currentNode = get<1>(sortedNeighbors.top());
  }
  return hamilton;
}

void bruteForceTspRec(graph::SuperGraph &graph, bool branchAndBound, int startNode, vector<bool>& visited, vector<int>& current, vector<int>& best, double& currentWeight, double& bestWeight);
bool boundCondition(graph::SuperGraph &graph, int startNode, const vector<bool>& visited, const vector<int>& current, const double& currentWeight, const double& bestWeight);

vector<edge_t> bruteForceTsp(graph::SuperGraph &graph, bool branchAndBound) {
  auto visited = vector(graph.numNodes, false);
  vector<int> current;
  current.reserve(graph.numNodes);
  vector<int> best;
  best.reserve(graph.numNodes);
  double currentWeight = 0.0;
  double bestWeight = DBL_MAX;

  for (int i = 0 ; i < graph.numNodes; i++) {
    current.push_back(i);
    visited[i] = true;
    bruteForceTspRec(graph, branchAndBound, i, visited, current, best, currentWeight, bestWeight);
    current.pop_back();
    visited[i] = false;
  }

  // construct edge list from node list
  vector<edge_t> result;
  result.reserve(graph.numNodes);
  for (int i = 0 ; i < best.size() - 1; i++) {
    result.emplace_back(best[i], best[i + 1], graph.getWeight(best[i], best[i + 1]));
  }
  result.emplace_back(best.back(), best.front(), graph.getWeight(best.back(), best.front()));
  return result;
}

void bruteForceTspRec(graph::SuperGraph &graph, const bool branchAndBound, const int startNode, vector<bool>& visited, vector<int>& current, vector<int>& best, double& currentWeight, double& bestWeight) {
  if (current.size() == graph.numNodes) {
    if (currentWeight + graph.getWeight(current.back(), startNode) < bestWeight) {
      best = current;
      bestWeight = currentWeight + graph.getWeight(current.back(), startNode);
    }
    return;
  }
  if (branchAndBound && boundCondition(graph, startNode, visited, current, currentWeight, bestWeight)) {
    return;
  }
  for (int i = 0 ; i < graph.numNodes; i++) {
    if (visited[i]) continue;
    visited[i] = true;
    currentWeight += graph.getWeight(current.back(), i);
    current.push_back(i);
    bruteForceTspRec(graph, branchAndBound, startNode, visited, current, best, currentWeight, bestWeight);
    visited[i] = false;
    current.pop_back();
    currentWeight -= graph.getWeight(current.back(), i);
  }
}

bool boundCondition(graph::SuperGraph &graph, const int startNode, const vector<bool>& visited, const vector<int>& current, const double& currentWeight, const double& bestWeight) {
  if (current.size() >= graph.numNodes - 2) return false;
  double bestCaseWeight = currentWeight;
  for (int u = 0; u < graph.numNodes; u++) {
    if (visited[u] && u != startNode) continue;  // include path back to start node in calculation, even though it is marked as "visited"
    double cheapestWeight = DBL_MAX;
    double secondCheapestWeight  = DBL_MAX;
    for (int v = 0; v < graph.numNodes; v++) {
      if ((visited[v] && u != startNode) || u == v) continue;
      const double weight = graph.getWeight(u, v);
      if (weight < cheapestWeight) {
        secondCheapestWeight = cheapestWeight;
        cheapestWeight = weight;
      }
      else if (weight < secondCheapestWeight) {
        secondCheapestWeight = weight;
      }
    }
    bestCaseWeight += 0.5 * (cheapestWeight + secondCheapestWeight);
  }
  return bestCaseWeight >= bestWeight;
}
