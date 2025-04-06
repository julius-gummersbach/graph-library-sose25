#include <string>
#include <sstream>
#include "AdjacentMatrixGraph.h"

namespace graph {
using namespace std;

    AdjacentMatrixGraph::AdjacentMatrixGraph(ifstream& input) : SuperGraph(input) {
        input >> numNodes;
        adjacencyMatrix = new double[numNodes * numNodes];
        int a, b;
        while (input >> a >> b) {
            adjacencyMatrix[a * numNodes + b] = 1;
        }
        input.close();
    }

    AdjacentMatrixGraph::~AdjacentMatrixGraph() {
        delete[] adjacencyMatrix;
    }

    vector<int> AdjacentMatrixGraph::getAdjacentNodes(int node) const {
        vector<int> adjacentNodes;
        for (int i = 0; i < numNodes; i++) {
            if (adjacencyMatrix[node * numNodes + i] != 0) {
                adjacentNodes.push_back(i);
            }
        }
        return adjacentNodes;
    }
}