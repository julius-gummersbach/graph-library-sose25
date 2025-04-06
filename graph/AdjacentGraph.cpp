#include <string>
#include <sstream>
#include "AdjacentGraph.h"

namespace graph {
using namespace std;

    AdjacentGraph::AdjacentGraph(ifstream& input) : SuperGraph(input) {
        input >> numNodes;
        adjacencyMatrix = new double[numNodes * numNodes];
        int a, b;
        while (input >> a >> b) {
            adjacencyMatrix[a * numNodes + b] = 1;
        }
        input.close();
    }

    AdjacentGraph::~AdjacentGraph() {
        delete[] adjacencyMatrix;
    }

    vector<int> AdjacentGraph::getAdjacentNodes(int node) const {
        vector<int> adjacentNodes;
        for (int i = 0; i < numNodes; i++) {
            if (adjacencyMatrix[node * numNodes + i] != 0) {
                adjacentNodes.push_back(i);
            }
        }
        return adjacentNodes;
    }
}