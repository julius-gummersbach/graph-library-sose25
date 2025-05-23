#include <cfloat>
#include <vector>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"

using namespace std;


/**
 * @param graph a complete, weighted graph
 * @param startNode
 * @return edge list of a hamilton path
 */
vector<shared_ptr<const edge::WeightedEdge>> nearestNeighbors(graph::SuperGraph &graph, const int startNode) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  vector<shared_ptr<const edge::WeightedEdge>> hamilton;
  hamilton.reserve(graph.numNodes);
  vector visited(graph.numNodes, false);

  int currentNode = startNode;
  visited[startNode] = true;
  while (hamilton.size() < graph.numNodes - 1) {
    int closestNode = 0;
    double minDist = INFINITY;
    // find closest unvisited node
    for (int node = 0; node < graph.numNodes; node++) {
      if (visited[node]) continue;
      double weight = dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(currentNode, node))->getWeight();
      if (weight < minDist) {
        closestNode = node;
        minDist = weight;
      }
    }
    visited[closestNode] = true;
    hamilton.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(currentNode, closestNode)));
    currentNode = closestNode;
  }
  hamilton.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(currentNode, startNode)));
  return hamilton;
}

/**
 * @param graph a complete graph
 * @return edge list of a hamilton path, that is at most 2 times worse than the optimal solution
 */
vector<shared_ptr<const edge::WeightedEdge>> doubleTreeAlgorthm(graph::SuperGraph &graph) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  // create adjacency list for mst
  auto mst = getMSTPrim(graph);
  graph::AdjacentListGraph mstGraph(mst);

  vector visited(graph.numNodes, false);
  vector<int> order = depthFirstSearch(mstGraph, mst[0]->getFrom(), visited);

  // build hamilton edge list from node order
  vector<shared_ptr<const edge::WeightedEdge>> hamilton;
  hamilton.reserve(order.size());
  for (int i = 0; i < order.size() - 1; i++) {
    hamilton.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(order[i], order[i+1])));
  }
  hamilton.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(order.back(), order.front())));
  return hamilton;
}

void bruteForceTspRec(graph::SuperGraph &graph, bool branchAndBound, int startNode, vector<bool>& visited, vector<int>& current, vector<int>& best, double currentWeight, double& bestWeight);
bool boundCondition(graph::SuperGraph &graph, int startNode, const vector<bool>& visited, const vector<int>& current, const double& currentWeight, const double& bestWeight);

/**
 * Brute forces all possible hamilton paths in the given graph and returns an optimal one
 * @param graph a complete graph with no negative edge weights that satisfies the triangle inequality
 * @param branchAndBound whether to use branch and bound
 * @return an optimal hamilton path
 */
vector<shared_ptr<const edge::WeightedEdge>> bruteForceTsp(graph::SuperGraph &graph, const bool branchAndBound) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  auto visited = vector(graph.numNodes, false);
  vector<int> current;
  current.reserve(graph.numNodes);
  vector<int> best;
  best.reserve(graph.numNodes);
  double currentWeight = 0.0;
  double bestWeight = INFINITY;

  constexpr int startNode = 0;  // it is sufficient to start from one of the nodes, because the circles found from the other nodes will be equivalent
  current.push_back(startNode);
  visited[startNode] = true;
  bruteForceTspRec(graph, branchAndBound, startNode, visited, current, best, currentWeight, bestWeight);

  // construct edge list from node list
  vector<shared_ptr<const edge::WeightedEdge>> result;
  result.reserve(graph.numNodes);
  for (int i = 0 ; i < best.size() - 1; i++) {
    result.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(best[i], best[i + 1])));
  }
  result.push_back(dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(best.back(), best.front())));
  return result;
}

/**
 * Recursive subroutine for brute-forcing the TSP
 * @param graph a complete graph with no negative edge weights that satisfies the triangle inequality
 * @param branchAndBound whether to use branch and bound
 * @param startNode the node the search was started from. Needed to check the cost of returning to the start node
 * @param visited which nodes were already visited in the current call
 * @param current path
 * @param best best hamilton circle found so far
 * @param currentWeight
 * @param bestWeight weight sum of best hamilton circle found so far
 */
void bruteForceTspRec(graph::SuperGraph &graph, const bool branchAndBound, const int startNode, vector<bool>& visited, vector<int>& current, vector<int>& best, const double currentWeight, double& bestWeight) {
  if (current.size() == graph.numNodes) {
    double weightBackToStartNode = dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(current.back(), startNode))->getWeight();
    if (currentWeight + weightBackToStartNode < bestWeight) {
      best = current;
      bestWeight = currentWeight + weightBackToStartNode;
    }
    return;
  }
  if (branchAndBound && boundCondition(graph, startNode, visited, current, currentWeight, bestWeight)) {
    return;
  }
  for (int i = 0 ; i < graph.numNodes; i++) {
    if (visited[i]) continue;
    visited[i] = true;
    const double newWeight = dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(current.back(), i))->getWeight();
    current.push_back(i);
    bruteForceTspRec(graph, branchAndBound, startNode, visited, current, best, currentWeight + newWeight, bestWeight);
    visited[i] = false;
    current.pop_back();
  }
}

/**
 * Condition for bounding the current branch while traversing a TSP graph
 * @param graph a complete graph with no negative edge weights that satisfies the triangle inequality
 * @param startNode the node the search was started from. Needed to check the cost of returning to the start node
 * @param visited which nodes were already visited in the current call
 * @param current current node
 * @param currentWeight
 * @param bestWeight weight sum of best hamilton circle found so far
 * @return true, if the current branch can not yield a better solution than bestWeight, false if it might
 */
bool boundCondition(graph::SuperGraph &graph, const int startNode, const vector<bool>& visited, const vector<int>& current, const double& currentWeight, const double& bestWeight) {
  if (current.size() >= graph.numNodes - 2) return false;  // if only two nodes are left, might as well brute-force
  double bestCaseWeight = currentWeight;

  // for all nodes that still need to be visited (unvisited nodes + start node)
  for (int u = 0; u < graph.numNodes; u++) {
    if (visited[u] && u != startNode) continue;
    // find cheapest and secondToCheapest edges
    double cheapestWeight = INFINITY;
    double secondCheapestWeight  = INFINITY;
    for (int v = 0; v < graph.numNodes; v++) {
      if ((visited[v] && u != startNode) || u == v) continue;
      const double weight = dynamic_pointer_cast<const edge::WeightedEdge>(graph.getEdge(u, v))->getWeight();
      if (weight < cheapestWeight) {
        secondCheapestWeight = cheapestWeight;
        cheapestWeight = weight;
      }
      else if (weight < secondCheapestWeight) {
        secondCheapestWeight = weight;
      }
    }
    // assume these can be used, what would the best weight of the current branch be
    // because edges will be counted twice by this approach: * 0.5
    bestCaseWeight += 0.5 * cheapestWeight;
    if (u != startNode) {
      bestCaseWeight += 0.5 * secondCheapestWeight;
    }
  }
  return bestCaseWeight >= bestWeight;
}
