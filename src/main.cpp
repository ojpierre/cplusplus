#include "graph.hpp"
#include <iostream>
#include <vector>
#include <limits>

void testGraphFunctions() {
    // Create a graph
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.addEdge(1, 2, 2);
    G.addEdge(1, 3, 3);
    G.addEdge(3, 4, 4);

    // Test isSubgraph
    Graph<int> H(5);
    H.addEdge(0, 1, 1);
    H.addEdge(1, 2, 2);
    H.addEdge(3, 4, 4);

    std::cout << "Is H a subgraph of G? " << (isSubgraph(H, G) ? "Yes" : "No") << std::endl;

    // Test isTreePlusIsolated
    std::cout << "Is G a tree plus isolated vertices? " << (isTreePlusIsolated(G, 0) ? "Yes" : "No") << std::endl;

    // Test pathLengthsFromRoot
    std::vector<int> distances = pathLengthsFromRoot(G, 0);
    std::cout << "Path lengths from root 0: ";
    for (int d : distances) {
        std::cout << d << " ";
    }
    std::cout << std::endl;

    // Test allEdgesRelaxed
    std::vector<int> bestDistanceTo = {0, 1, 3, 4, 8};
    std::cout << "Are all edges relaxed? " << (allEdgesRelaxed(bestDistanceTo, G, 0) ? "Yes" : "No") << std::endl;
}

int main() {
    testGraphFunctions();
    return 0;
}
