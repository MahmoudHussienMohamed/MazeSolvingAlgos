#pragma once
#include <algorithm>
#include "Graph.h"
#include <stack>
#include <queue>
class GraphTraversalAlgorithm {
protected:
    Indices path, traversed;
    size_t distance;
    static const size_t infinity = INT64_MAX;
    static void completePath(Index from, Index to, Indices &path){
        size_t *idx;
        int8_t val;
        if(from.row == to.row){
            idx = &from.col;
            val = (from.col < to.col? 1 : -1);
        } else {
            idx = &from.row;
            val = (from.row < to.row? 1 : -1);
        }
        while(from != to){
            (*idx) += val;
            path.push_back(from);
        }
    }
    template<typename GTAType>          // GTA stands for (G)raph (T)raversal (A)lgorithm
    static void reconstructPathIfValid(GTAType& gta) {
        if (!gta.solved<GTAType>(gta))
            return;
        for (typename GTAType::Node* node = gta.graph.destination(); node; node = node->parentNode()){
            if(gta.path.empty() || gta.graph.ManhattanDistance(gta.path.back(), node->value) == 1)
                gta.path.push_back(node->value);
            else
                completePath(gta.path.back(), node->value, gta.path);
        }
    }
public:
    GraphTraversalAlgorithm() : distance(0) {}
    template<typename GTAType> 
    static bool solved(GTAType& gta){
        return gta.traversed.back() == gta.graph.destination()->value;
    }
    void setDistance(size_t dist){
        distance = dist;
    }
    const Indices& SrcToDestPath()   { return      path; }
    const Indices& TraversedNodes()  { return traversed; }
    size_t SrcToDestDistance()       { return  distance; }
    size_t TraversedNodesNo(){ return  traversed.size(); }
};
class DepthFirstSearch : public GraphTraversalAlgorithm {
    public:
    struct Node : WeightedNode{
        Edge<Node> parentEdge;
        vector<Edge<Node>> edges;
        Node(Index val = {}) : WeightedNode(val), parentEdge() {}
        Node *parentNode(){ return parentEdge.node; }
    };
    WeightedGraph<Node> graph;
    private:
    void DFS(Node* src, Node* dest) {
        std::stack<Node*> stack;
        stack.push(src);
        while(!stack.empty()){
            Node *curr = stack.top();
            stack.pop();
            if(curr->visited)
                continue;
            curr->visited = true;
            traversed.push_back(curr->value);
            if(curr == dest)
                return;
            for(auto& edge : curr->edges)
                if(!edge.node->visited){
                    edge.node->parentEdge.set(curr, edge.weight);
                    stack.push(edge.node);
                }
        }
    }
public:
    DepthFirstSearch(Grid& grid, Index start, Index end)
        : GraphTraversalAlgorithm(), graph(grid, start, end){}
    void setDistance(){
        for (Node* node = graph.destination(); node; node = node->parentEdge.node)
            distance += node->parentEdge.weight;
    }
    void solve() {
        DFS(graph.source(), graph.destination());
        reconstructPathIfValid<DepthFirstSearch>(*this);
        if(solved<DepthFirstSearch>(*this))
            setDistance();
    }
};
class BreadthFirstSearch : public GraphTraversalAlgorithm {
public:
    struct Node : public UnWeightedNode {
        Node* parent;
        vector<Node*> neighbors;
        Node(Index val = {}, Node* p = nullptr)
            : UnWeightedNode(val), parent(p) {}
        Node *parentNode(){ return parent; }
    };
    UnWeightedGraph<Node> graph;
private:
    void BFS(Node* src, Node* dest) {
        std::queue<Node*> Q;
        Q.push(src);
        src->visited = true;
        while (!Q.empty()) {
            Node* node = Q.front();
            traversed.push_back(node->value);
            if (node == dest)
                return;
            Q.pop();
            for (auto neighbor : node->neighbors)
                if (!neighbor->visited) {
                    neighbor->visited = true;
                    neighbor->parent = node;
                    Q.push(neighbor);
                }
        }
    }
public:
    BreadthFirstSearch(Grid& grid, Index start, Index end)
        : graph(grid, start, end) {}
    template<typename GTAType>          // GTA stands for (G)raph (T)raversal (A)lgorithm
    static void reconstructPathIfValid(BreadthFirstSearch& bfs) {
        for(BreadthFirstSearch::Node* node = bfs.graph.destination(); node; node = node->parentNode())
            bfs.path.push_back(node->value);
    }
    void solve() {
        BFS(graph.source(), graph.destination());
        reconstructPathIfValid<BreadthFirstSearch>(*this);
        if(solved<BreadthFirstSearch>(*this))
            setDistance(path.size() - 1);
    }
};
class DijkstraAlgorithm : public GraphTraversalAlgorithm {
public:
    struct Node : public WeightedNode {
        Node* parent;
        size_t dist;
        vector<Edge<Node>> edges;
        Node(Index val = {}, Node* p = nullptr, size_t d = infinity)
            : WeightedNode(val), parent(p), dist(d) {}
        Node *parentNode(){ return parent; }
    };
    WeightedGraph<Node> graph;
private:
    struct compare {
        constexpr bool operator()(const Node* l, const Node* r) const {
            return l->dist > r->dist;
        }
    };
    void Dijkstra(Node* src, Node* dest) {
        std::priority_queue<Node*, vector<Node*>, compare> Q;
        src->dist = 0;
        src->visited = true;
        Q.push(src);
        while (!Q.empty()) {
            Node* node = Q.top();
            traversed.push_back(node->value);
            if (node == dest)
                return;
            Q.pop();
            for (auto& edge : node->edges) {
                size_t alt = node->dist + edge.weight;
                Node* neighbor = edge.node;
                if (alt < neighbor->dist) {                           // Relaxition
                    neighbor->dist = alt;
                    neighbor->parent = node;
                    if (!neighbor->visited) {
                        neighbor->visited = true;
                        Q.push(neighbor);
                    }
                }
            }
        }
    }
public:
    DijkstraAlgorithm(Grid& grid, Index start, Index end)
        : GraphTraversalAlgorithm(), graph(grid, start, end) {}
    void solve() {
        Dijkstra(graph.source(), graph.destination());
        reconstructPathIfValid<DijkstraAlgorithm>(*this);
        if(solved<DijkstraAlgorithm>(*this))
            setDistance(graph.destination()->dist);
    }
};
class AStar : public GraphTraversalAlgorithm {
public:
    struct Node : public WeightedNode {
        Node* parent;
        size_t gScore,                                              // cost of the cheapest path from the `start` node to the current (this) Node 
               fScore;                                              // `gScore` + estimated cost to reach `goal` from the current Node
        vector<Edge<Node>> edges;
        Node(Index val = {}, Node* p = nullptr, size_t g = infinity)
            : WeightedNode(val), parent(p), gScore(g), fScore(infinity) {}
        Node *parentNode(){ return parent; }
    };
    WeightedGraph<Node> graph;
private:
    size_t heuristic(Node* currNode) {
        return graph.ManhattanDistance(currNode, graph.destination());
    }
    struct compare {
        constexpr bool operator()(const Node* l, const Node* r) const {
            return l->fScore > r->fScore;
        }
    };
    void DijkstraWithHeuristic(Node* src, Node* dest) {
        src->visited = true;
        src->gScore = 0;
        src->fScore = heuristic(src);
        std::priority_queue<Node*, vector<Node*>, compare> Q;
        Q.push(src);
        while (!Q.empty()) {
            Node* node = Q.top();
            traversed.push_back(node->value);
            if (node == dest)
                return;
            Q.pop();
            for (auto& edge : node->edges) {
                size_t alt = node->gScore + edge.weight;
                Node* neighbor = edge.node;
                if (alt < neighbor->gScore) {                         // Relaxition
                    neighbor->gScore = alt;
                    neighbor->parent = node;
                    neighbor->fScore = alt + heuristic(neighbor);
                    if (!neighbor->visited) {
                        neighbor->visited = true;
                        Q.push(neighbor);
                    }
                }
            }
        }
    }
public:
    AStar(Grid& grid, Index start, Index end)
        : graph(grid, start, end) {}
    void solve() {
        DijkstraWithHeuristic(graph.source(), graph.destination());
        reconstructPathIfValid<AStar>(*this);
        if(solved<AStar>(*this))
            setDistance(graph.destination()->gScore);
    }
};
class BellmanFord : public GraphTraversalAlgorithm {
public:
    using Node = DijkstraAlgorithm::Node;
    WeightedGraph<Node> graph;
private:
    void Bellman_Ford(Node* src, Node* dest) {
        src->dist = 0;
        vector<Node>& nodes = graph.allNodes();
        size_t V = nodes.size() - 1;
        for (size_t i = 0; i < V; i++) {
            bool nothing_changed = true;
            for (auto& node : nodes) {
                Node* u = &node;
                for (auto& edge : u->edges) {
                    size_t alt = u->dist + edge.weight;
                    Node* v = edge.node;
                    if (alt < v->dist) {                           // Relaxition
                        v->dist = alt;
                        v->parent = u;
                        traversed.push_back(v->value);
                        nothing_changed = false;
                    }
                }
            }
            if (nothing_changed)
                break;
        }
    }

public:
    BellmanFord(Grid& grid, Index start, Index end)
        : GraphTraversalAlgorithm(), graph(grid, start, end) {}
    template<typename GTAType> 
    static bool solved(BellmanFord &bf){
        return bf.graph.destination()->dist != infinity;
    }
    void solve() {
        Bellman_Ford(graph.source(), graph.destination());
        reconstructPathIfValid<BellmanFord>(*this);
        if(solved<BellmanFord>(*this))
            setDistance(graph.destination()->dist);
    }
};
class FloydWarshall : public GraphTraversalAlgorithm {
public:
    struct Node : WeightedNode {
        size_t indexInVector;
        vector<Edge<Node>> edges;
        Node(Index val, size_t idx = 0)
            : WeightedNode(val), indexInVector(idx) {}
    };
    WeightedGraph<Node> graph;
private:
    void reconstructPathIfValid(Node* src, Node* dest, vector<vector<size_t>>& dist, vector<vector<Node*>>& next) {
        if (!next[src->indexInVector][dest->indexInVector])
            return;
        distance = dist[src->indexInVector][dest->indexInVector];
        path.push_back(src->value);
        Node *node = src;
        while (node != dest) {
            node = next[node->indexInVector][dest->indexInVector];
            if(graph.ManhattanDistance(path.back(), node->value) == 1){
                path.push_back(node->value);
                continue;
            }
            completePath(path.back(), node->value, path);
        }
    }
    void Floyd_Warshall(Node* src, Node* dest) {
        auto& nodes = graph.allNodes();
        size_t V = nodes.size();
        for (size_t i = 0; i < V; i++)
            nodes[i].indexInVector = i;
        vector<vector<size_t>> dist(V, vector<size_t>(V, infinity));
        vector<vector<Node *>> next(V, vector<Node*> (V, nullptr));
        for (auto& node : nodes) {
            size_t u = node.indexInVector;
            dist[u][u] = 0;
            next[u][u] = &node;
            for (auto& edge : node.edges) {
                size_t v = edge.node->indexInVector;
                dist[u][v] = edge.weight;
                next[u][v] = edge.node;
            }
        }
        for (size_t k = 0; k < V; k++)
            for (size_t i = 0; i < V; i++)
                for (size_t j = 0; j < V; j++)
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
        traversed.reserve(nodes.size());
        for (auto& node : nodes)
            traversed.push_back(node.value);
        reconstructPathIfValid(src, dest, dist, next);
    }

public:
    size_t TraversedNodesNo() {
        size_t n = traversed.size();
        return n * n * n; 
    }
    FloydWarshall(Grid& grid, Index start, Index end)
        : GraphTraversalAlgorithm(), graph(grid, start, end) {
    }
    void solve() {
        Floyd_Warshall(graph.source(), graph.destination());
    }
};
class BidirectionalSearch : public GraphTraversalAlgorithm {
public:
    const enum VisitType{
        notVisited      = 0b0000,
        fromSource      = 0b0001,
        fromDestination = 0b0010
    };
    struct Node : public UnWeightedNode {
        Node* parent;
        VisitType visited;
        vector<Node*> neighbors;
        Node(Index val = {}, Node* p = nullptr)
            : UnWeightedNode(val), parent(p), visited(notVisited){}
        Node *parentNode(){ return parent; }
        bool is(VisitType type){ return visited == type; }
    }*IFS, *IFD;
    UnWeightedGraph<Node> graph;
private:
    class OneStepBFS{
        Node *src, *dest, **ifs, **ifd;
        std::queue<Node*> Q;
        Indices *traversed;
        VisitType type;
    public:
        OneStepBFS(Node* s, Node* d, VisitType t, Node **ifs, Node **ifd, Indices *traversed):
            src(s), dest(d), ifs(ifs), ifd(ifd), type(t), traversed(traversed) {
            Q.push(src);
        }
        void advance(){
            if (noNeedToAdvance())
                return;
            Node* node = Q.front();
            traversed->push_back(node->value);
            Q.pop();
            for (auto neighbor : node->neighbors){
                if (neighbor->is(type))         // already visited from this side
                    continue;
                else if (neighbor->is(notVisited)) {
                    neighbor->visited = type;
                    neighbor->parent = node;
                    Q.push(neighbor);
                } else {                        // visited from the other side  
                    traversed->push_back(neighbor->value);
                    *ifs = node;
                    *ifd = neighbor;
                    return;
                } 
            }
        }
        bool hasNoSolution(){ return Q.empty(); }
        bool intersectionFound(){ return *ifs && *ifd; }
        bool noNeedToAdvance() { return hasNoSolution() || intersectionFound(); }
    };
public:
    BidirectionalSearch(Grid& grid, Index start, Index end)
        : graph(grid, start, end), IFS(nullptr), IFD(nullptr) {}
    template<typename GTAType> 
    static bool solved(BidirectionalSearch& gta){
        return gta.IFS && gta.IFD;
    }
    template<typename GTAType>          // GTA stands for (G)raph (T)raversal (A)lgorithm
    static void reconstructPathIfValid(BidirectionalSearch& bds) {
        if (!solved<BidirectionalSearch>(bds))
            return;
        auto reconstructPath = [&](BidirectionalSearch::Node* startNode, BidirectionalSearch::Node* endNode){
            for (BidirectionalSearch::Node* node = startNode; node != endNode; node = node->parentNode())
                bds.path.push_back(node->value);
            bds.path.push_back(endNode->value);
        };
        reconstructPath(bds.IFS, bds.graph.source());       // reconstruct path from source node to intersection from source side node
        std::reverse(bds.path.begin(), bds.path.end());
        reconstructPath(bds.IFD, bds.graph.destination());  // reconstruct path from intersection from destination side node to destination node
    }
    void solve() {
        OneStepBFS srcToDest(graph.source(), graph.destination(), fromSource,      &IFS, &IFD, &traversed),
                   destToSrc(graph.destination(), graph.source(), fromDestination, &IFD, &IFS, &traversed);
        while (!(srcToDest.intersectionFound() || destToSrc.intersectionFound())){
            if (srcToDest.hasNoSolution() && destToSrc.hasNoSolution())
                return;
            srcToDest.advance();
            destToSrc.advance();
        }
        reconstructPathIfValid<BidirectionalSearch>(*this);
        if(solved<BidirectionalSearch>(*this))
            setDistance(path.size() - 1);
    }
};