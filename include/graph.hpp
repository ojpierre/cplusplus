#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <iostream>
#include <fstream>
#include <utility>
#include <functional>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <unordered_map>
#include <limits>
#include <stdexcept>

template <typename T>
class Graph {
 private:
  std::vector<std::unordered_map<int, T> > adjList {};
  int numVertices {};

 public:
  // empty graph with N vertices
  explicit Graph(int N);

  // construct graph from edge list in filename
  explicit Graph(const std::string& filename);

  // add an edge directed from vertex i to vertex j with given weight
  void addEdge(int i, int j, T weight);

  // removes edge from vertex i to vertex j
  void removeEdge(int i, int j);

  // is there an edge from vertex i to vertex j?
  bool isEdge(int i, int j) const;

  // return weight of edge from i to j
  // will throw an exception if there is no edge from i to j
  T getEdgeWeight(int i, int j) const;

  // returns number of vertices in the graph
  int size() const;

  // alias a const iterator to our adjacency list type to iterator
  using iterator = typename std::vector<std::unordered_map<int, T> >::const_iterator;

  // cbegin returns const iterator pointing to first element of adjList
  iterator begin() const {
    return adjList.cbegin();
  }

  iterator end() const {
    return adjList.cend();
  }

  // return iterator to a particular vertex
  iterator neighbours(int a) const {
    return adjList.begin() + a;
  }
};

template <typename T>
Graph<T>::Graph(int N) : adjList(N), numVertices {N} {}

template <typename T>
Graph<T>::Graph(const std::string& inputFile) {
  std::ifstream infile {inputFile};
  if (!infile) {
    std::cerr << inputFile << " could not be opened\n";
    return;
  }
  infile >> numVertices;
  adjList.resize(numVertices);
  int u, v;
  T w;
  while (infile >> u >> v >> w) {
    addEdge(u, v, w);
  }
}

template <typename T>
void Graph<T>::addEdge(int i, int j, T weight) {
  adjList[i][j] = weight;
}

template <typename T>
void Graph<T>::removeEdge(int i, int j) {
  adjList[i].erase(j);
}

template <typename T>
bool Graph<T>::isEdge(int i, int j) const {
  return adjList[i].find(j) != adjList[i].end();
}

template <typename T>
T Graph<T>::getEdgeWeight(int i, int j) const {
  if (!isEdge(i, j)) {
    throw std::runtime_error("No edge between the vertices");
  }
  return adjList[i].at(j);
}

template <typename T>
int Graph<T>::size() const {
  return numVertices;
}

// Additional functions for shortest path verification

template <typename T>
bool isTreePlusIsolated(const Graph<T>& G, int root);

template <typename T>
bool isSubgraph(const Graph<T>& H, const Graph<T>& G);

template <typename T>
std::vector<T> pathLengthsFromRoot(const Graph<T>& tree, int root);

template <typename T>
bool allEdgesRelaxed(const std::vector<T>& bestDistanceTo, const Graph<T>& G, int source);

template <typename T>
T infinity() {
  if (std::numeric_limits<T>::has_infinity) {
    return std::numeric_limits<T>::infinity();
  } else if constexpr (std::is_same_v<T, MyInteger>) {
    return MyInteger(std::numeric_limits<int>::max());
  } else {
    return std::numeric_limits<T>::max();
  }
}

// Definitions of the additional functions

template <typename T>
bool isTreePlusIsolated(const Graph<T>& G, int root) {
    int N = G.size();
    std::vector<bool> visited(N, false);
    std::queue<int> q;
    q.push(root);
    visited[root] = true;

    int countVisited = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        countVisited++;

        for (auto it = G.neighbours(u)->begin(); it != G.neighbours(u)->end(); ++it) {
            int v = it->first;
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }

    int edgeCount = 0;
    for (int i = 0; i < N; ++i) {
        edgeCount += G.neighbours(i)->size();
    }

    edgeCount /= 2;

    if (edgeCount != countVisited - 1) {
        return false;
    }

    for (int i = 0; i < N; ++i) {
        if (!visited[i] && !G.neighbours(i)->empty()) {
            return false;
        }
    }

    return true;
}

template <typename T>
bool isSubgraph(const Graph<T>& H, const Graph<T>& G) {
    if (H.size() > G.size()) {
        return false;
    }

    for (int u = 0; u < H.size(); ++u) {
        for (auto it = H.neighbours(u)->begin(); it != H.neighbours(u)->end(); ++it) {
            int v = it->first;
            T weight = it->second;
            if (!G.isEdge(u, v) || G.getEdgeWeight(u, v) != weight) {
                return false;
            }
        }
    }

    return true;
}

template <typename T>
std::vector<T> pathLengthsFromRoot(const Graph<T>& tree, int root) {
    int N = tree.size();
    std::vector<T> distances(N, infinity<T>());
    std::queue<int> q;

    distances[root] = 0;
    q.push(root);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (auto it = tree.neighbours(u)->begin(); it != tree.neighbours(u)->end(); ++it) {
            int v = it->first;
            T weight = it->second;
            if (distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
                q.push(v);
            }
        }
    }

    return distances;
}

template <typename T>
bool allEdgesRelaxed(const std::vector<T>& bestDistanceTo, const Graph<T>& G, int source) {
    if (bestDistanceTo.at(source) != 0) {
        return false;
    }

    for (int u = 0; u < G.size(); ++u) {
        for (auto it = G.neighbours(u)->begin(); it != G.neighbours(u)->end(); ++it) {
            int v = it->first;
            T weight = it->second;
            if (bestDistanceTo.at(u) + weight < bestDistanceTo.at(v)) {
                return false;
            }
        }
    }

    return true;
}

#endif // GRAPH_HPP_
