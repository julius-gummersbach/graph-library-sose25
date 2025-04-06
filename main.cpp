#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"
#include "graph/AdjacentMatrixGraph.h"
#include "graph/AdjacentListGraph.h"

using namespace std;

const string INPUT_PATH = "../input/";

void depthFirstSearch(const graph::SuperGraph& graph, int startNode, vector<int>& subGraph, vector<bool>& visited);
int getConnectedComponents(const graph::SuperGraph& graph);


int main() {
    cout << "Press 1 to use an EdgeListGraph, 2 to use an AdjacentMatrixGraph, or 3 to use an AdjacentListGraph" << endl;
    int graphModeChoice;
    cin >> graphModeChoice;
    while (graphModeChoice != 1 && graphModeChoice != 2&& graphModeChoice != 3) {
        cout << "Invalid choice. Please enter 1, 2 or 3:" << endl;
        cin >> graphModeChoice;
    }

    cout << "Select one of the available graphs:" << endl;
    auto availInputs = filesystem::directory_iterator(INPUT_PATH);
    std::vector<std::filesystem::path> graphPaths;
    int size = 0;
    for (const auto& path : availInputs) {
        cout << size++ << ": " << path.path() << endl;
        graphPaths.push_back(path.path());
    }
    int inputChoice;
    cin >> inputChoice;
    while (inputChoice < 0 || inputChoice >= size) {
        cout << "Invalid choice. Please enter a number between 0 and " << size - 1 << ":" << endl;
        cin >> inputChoice;
    }
    string inputPath = graphPaths[inputChoice].string();

    cout << "Starting to read the graph from " << inputPath << endl;
    auto start = std::chrono::high_resolution_clock::now();

    // Read from the text file
    ifstream inputFile(inputPath);
    std::unique_ptr<graph::SuperGraph> graph;
    if (graphModeChoice == 1) {
        graph = std::make_unique<graph::EdgeListGraph>(inputFile);
    } else if (graphModeChoice == 2) {
        graph = std::make_unique<graph::AdjacentMatrixGraph>(inputFile);
    } else if (graphModeChoice == 3) {
        graph = std::make_unique<graph::AdjacentListGraph>(inputFile);
    } else {
        cout << "Invalid graph mode choice." << endl;
        return 1;
    }

    cout << "Done reading the graph, starting to calculate connected components" << endl;
    auto setupComplete = std::chrono::high_resolution_clock::now();

    const int connectedComponents = getConnectedComponents(*graph);

    auto end = std::chrono::high_resolution_clock::now();
    auto setupDuration = std::chrono::duration_cast<std::chrono::milliseconds>(setupComplete - start);
    auto calcDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - setupComplete);

    std::cout << "Number of connected components: " << connectedComponents << std::endl;
    std::cout << "Time taken to set up the graph: " << setupDuration.count() << " ms" << std::endl;
    std::cout << "Time taken to find connected components: " << calcDuration.count() << " ms" << std::endl;

    return 0;
}

int getConnectedComponents(const graph::SuperGraph& graph) {

    int connectedComponents = 0;

    vector visited(graph.numNodes, false);
    vector<int> subGraph {};

    for (int node = 0; node < graph.numNodes; node++) {
        if (visited[node]) continue;
        connectedComponents++;
        depthFirstSearch(graph, node, subGraph, visited);
    }
    return connectedComponents;
}

void depthFirstSearch(const graph::SuperGraph& graph, const int startNode, vector<int>& subGraph, vector<bool>& visited) {
    subGraph.push_back(startNode);
    visited[startNode] = true;

    vector<int> adjacent = graph.getAdjacentNodes(startNode);
    for (int node : adjacent) {
        if (visited[node]) continue;
        depthFirstSearch(graph, node, subGraph, visited);
    }
}