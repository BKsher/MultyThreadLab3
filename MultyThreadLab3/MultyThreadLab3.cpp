#include <thread>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h> 
#include <queue>
using namespace std;

class Graph {
public:
    int vertexesNumber, edgesNumber;
    vector<vector<int> > g;

    Graph(vector<pair<int, int> >& edges) {
        edgesNumber = edges.size();
        g.resize(1e4);
        int maxVertex = 0;
        for (pair<int, int> e : edges) {
            g[e.first].push_back(e.second);
            g[e.second].push_back(e.first);
            maxVertex = max(maxVertex, max(e.first, e.second));
        }
        vertexesNumber = maxVertex + 1;
        g.resize(vertexesNumber);
    }
};

class DFS {
public:
    Graph *graph;

    DFS(Graph *graph) {
        this->graph = graph;
        visited.resize(graph->vertexesNumber, false);
        tin.resize(graph->vertexesNumber, 0);
        tout.resize(graph->vertexesNumber, 0);
        timerIn = timerOut = 1;
    }

    void run(int s = 0) {
        dfs(s);
    }

    vector<int> getTimesIn() {
        return tin;
    }

    vector<int> getTimesOut() {
        return tout;
    }

private:
    vector<bool> visited;
    vector<int> tin, tout;
    int timerIn, timerOut;

    void dfs(int v) {
        visited[v] = true;
        tin[v] = timerIn++;
        for (int u : graph->g[v])
            if (!visited[u]) dfs(u);
        tout[v] = timerOut++;
    }
};

class BFS {
public:
    Graph* graph;

    BFS(Graph* graph) {
        this->graph = graph;
        visited.resize(graph->vertexesNumber, false);
        distances.resize(graph->vertexesNumber, 1e9);
    }

    void run(int s = 0) {
        bfs(s);
    }

    vector<int> getDistances() {
        return distances;
    }

private:
    vector<bool> visited;
    vector<int> distances;

    void bfs(int s) {
        distances[s] = 0;
        queue <int> q;
        q.push(s);
        while (!q.empty()) {
            int v = q.front();
            q.pop();
            for (int u : graph->g[v]) {
                if (distances[u] > distances[v] + 1) {
                    distances[u] = distances[v] + 1;
                    q.push(u);
                }
            }
        }
    }
};

void runDFS(Graph *graph) {
    DFS dfs = DFS(graph);
    dfs.run();
}

void runBFS(Graph* graph) {
    BFS bfs = BFS(graph);
    bfs.run();
}

int main() {

    srand(time(0));
    int vertexes = 10000, edges_number = 10000;
    vector<pair<int, int> > edges;
    for (int i = 0; i < edges_number; i++) {
        int a = rand() % vertexes;
        int b = rand() % vertexes;
        edges.push_back(make_pair(a, b));
    }
    Graph* graph = new Graph(edges);

    auto begin = std::chrono::high_resolution_clock::now();

    runDFS(graph);  // around 0.04 seconds
    runBFS(graph);  // around 0.028 seconds

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    cout << "Time without parallel execution " << elapsed.count() * 1e-9 << endl << endl;  // should be around 0.068 seconds

    begin = std::chrono::high_resolution_clock::now();

    std::thread tA(runDFS, graph);
    std::thread tB(runBFS, graph);
    tA.join();
    tB.join();

    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    cout << "Time with parallel execution " << elapsed.count() * 1e-9 << endl; // usually around 0.05 seconds, which is faster then 0.068!
}
