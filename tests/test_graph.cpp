#include "graph.hpp"
#include <gtest/gtest.h>

TEST(GraphTest, AddEdge) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    EXPECT_TRUE(G.isEdge(0, 1));
    EXPECT_EQ(G.getEdgeWeight(0, 1), 1);
}

TEST(GraphTest, RemoveEdge) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.removeEdge(0, 1);
    EXPECT_FALSE(G.isEdge(0, 1));
}

TEST(GraphTest, IsSubgraph) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.addEdge(1, 2, 2);

    Graph<int> H(5);
    H.addEdge(0, 1, 1);

    EXPECT_TRUE(isSubgraph(H, G));
}

TEST(GraphTest, IsTreePlusIsolated) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.addEdge(1, 2, 2);
    G.addEdge(1, 3, 3);
    G.addEdge(3, 4, 4);

    EXPECT_TRUE(isTreePlusIsolated(G, 0));
}

TEST(GraphTest, PathLengthsFromRoot) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.addEdge(1, 2, 2);
    G.addEdge(1, 3, 3);
    G.addEdge(3, 4, 4);

    std::vector<int> distances = pathLengthsFromRoot(G, 0);
    std::vector<int> expected = {0, 1, 3, 4, 8};
    EXPECT_EQ(distances, expected);
}

TEST(GraphTest, AllEdgesRelaxed) {
    Graph<int> G(5);
    G.addEdge(0, 1, 1);
    G.addEdge(1, 2, 2);
    G.addEdge(1, 3, 3);
    G.addEdge(3, 4, 4);

    std::vector<int> bestDistanceTo = {0, 1, 3, 4, 8};
    EXPECT_TRUE(allEdgesRelaxed(bestDistanceTo, G, 0));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
