#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"
#include "graph/AdjacentGraph.h"

using namespace std;

void depthFirstSearch(const graph::SuperGraph& graph, int startNode, vector<int>& subGraph, vector<bool>& visited);
int getConnectedComponents(const graph::SuperGraph& graph);


int main() {
    cout << "Press 1 to use an EdgeListGraph, 2 to use an AdjacentGraph" << endl;
    int choice;
    cin >> choice;
    while (choice != 1 && choice != 2) {
        cout << "Invalid choice. Please enter 1 or 2:" << endl;
        cin >> choice;
    }

    cout << "Select one of the available graphs:" << endl;
    for (const auto & entry : filesystem::directory_iterator(path))
        cout << entry.path() << endl;

    graph::SuperGraph graph;
    if (choice == 1) {
        graph = graph::EdgeListGraph();
    }
    else {
        graph = graph::AdjacentGraph();
    }

    return 0;
}

int getConnectedComponents(const graph::SuperGraph& graph) {

    int connectedComponents = 0;

    vector<bool> visited(graph.numNodes, false);
    vector<int> subGraph {};

    for (int node = 0; node < graph.numNodes; node++) {
        if (visited[node]) continue;
        depthFirstSearch(graph, node, subGraph, visited);
        connectedComponents++;
    }
    return connectedComponents;
}

void depthFirstSearch(const graph::SuperGraph& graph, int startNode, vector<int>& subGraph, vector<bool>& visited) {
    subGraph.push_back(startNode);
    visited[startNode] = true;

    vector<int> adjacent = graph.getAdjacentNodes(startNode);
    for (int node : adjacent) {
        if (visited[node]) continue;
        depthFirstSearch(graph, node, subGraph, visited);
    }
}