#pragma once
#include <iostream>
#include <vector>
#include <string>
using std::string;
using std::vector;
using Grid = vector<vector<bool>>;
struct Index{
    static const size_t noPos = UINT64_MAX;
    size_t row, col;
    Index(size_t r = 0, size_t c = 0): row(r), col(c){}
    bool operator==(const Index &other){
        return row == other.row && col == other.col;
    }
    bool operator!=(const Index &other){
        return !(*this == other);
    }
    string as_string(){
        return std::to_string(row) + "," + std::to_string(col); 
    }
};
using Indices = vector<Index>;
template <typename NodeType>
struct Edge{
    NodeType *node;
    size_t  weight;
    Edge(NodeType *n = nullptr, size_t w = 0): node(n), weight(w){}
    void set(NodeType *n, size_t w = 0){ node = n; weight = w;}
};
struct WeightedNode{
    Index value;
    bool visited;
    vector<Edge<WeightedNode>> edges;
    WeightedNode(Index val = {}): value(val), visited(false){}
};
template <typename NodeType>
class Graph{
protected:
    NodeType *StartNode, *EndNode;
    vector<NodeType> Nodes;
public:
    Graph(): StartNode(nullptr), EndNode(nullptr){}
    NodeType *source()          { return StartNode; }
    NodeType *destination()     { return   EndNode; }
    vector<NodeType> &allNodes(){ return     Nodes; }
};
template <typename NodeType>
class WeightedGraph : public Graph<NodeType>{ 
    static void connect(NodeType *lhs, NodeType *rhs){
        size_t weight = ManhattanDistance(lhs, rhs); 
        lhs->edges.push_back(Edge<NodeType>(rhs, weight));
        rhs->edges.push_back(Edge<NodeType>(lhs, weight));
    }
    void setNodes(Grid &grid, Index &start, Index &end){
        size_t node_cnt{};
        Index gsize(grid.size(), grid[0].size());
        for(size_t i = 0; i < gsize.row; i++)
            for(size_t j = 0; j < gsize.col; j++)
                node_cnt += (grid[i][j]);
        this->Nodes.reserve(node_cnt);
        vector<NodeType*> deepestNodeForCol(gsize.col + 1, nullptr);
        for(size_t i = 0; i < gsize.row; i++){
            NodeType *lastConnectableNodeInRow = nullptr;
            for(size_t j = 0; j < gsize.col; j++){
                if(!grid[i][j]){
                    lastConnectableNodeInRow = deepestNodeForCol[j] = nullptr;
                    continue;
                }
                Index cell(i, j);
                bool hasLeftNeighbor {j > 0 && grid[i][j - 1]}, 
                     hasUpperNeighbor{i > 0 && grid[i - 1][j]},
                     hasRightNeighbor{j + 1 < gsize.col && grid[i][j + 1]},
                     hasDownNeighbor {i + 1 < gsize.row && grid[i + 1][j]},
                     hasVrtNeighbor  {hasLeftNeighbor || hasRightNeighbor},                         // has left or right neighbor (vertical) 
                     hasHrzNeighbor  {hasUpperNeighbor || hasDownNeighbor},                         // has upper or lower neighbor (horizontal)
                     Xor = hasLeftNeighbor ^ hasRightNeighbor ^ hasUpperNeighbor ^ hasDownNeighbor, // Execlusive-Or of 4 expressions equal 1 if there's odd number of them equal 1 (1[for dead-end] or 3[for junction])  
                     isStart{start == cell}, isEnd{end == cell};
                if(isStart || isEnd || Xor || (hasVrtNeighbor && hasHrzNeighbor)){                  // decision node 
                    this->Nodes.push_back(NodeType(cell));
                    NodeType *node = &this->Nodes.back();
                    if(hasLeftNeighbor && lastConnectableNodeInRow)
                        connect(node, lastConnectableNodeInRow);
                    if(hasUpperNeighbor && deepestNodeForCol[j])
                        connect(node, deepestNodeForCol[j]);
                    lastConnectableNodeInRow = deepestNodeForCol[j] = node;
                    if(isStart) this->StartNode = node;
                    if(isEnd)   this->EndNode   = node;
                } 
            }
        }
    }
public:
    static size_t abs_diff(size_t x, size_t y){
        return x > y? x - y : y - x;
    }
    static size_t ManhattanDistance(Index lhs, Index rhs){
        return abs_diff(lhs.row, rhs.row) + abs_diff(lhs.col, rhs.col);
    }
    static size_t ManhattanDistance(NodeType *lhs, NodeType *rhs){
        return ManhattanDistance(lhs->value, rhs->value);
    }
   WeightedGraph(Grid &grid, Index start, Index end): Graph<NodeType>(){
        setNodes(grid, start, end);
    }
};

struct UnWeightedNode{
    Index value;
    bool visited;
    vector<UnWeightedNode*> neighbors;
    UnWeightedNode(Index val = {}): value(val), visited(false){}
};
template <typename NodeType>
class UnWeightedGraph : public Graph<NodeType>{
    void setNodes(Grid &grid){
        for(size_t i = 0, cnt; i < grid.size(); i++)
            for(size_t j = 0; j < grid[i].size(); j++)
                if(grid[i][j])
                    this->Nodes.push_back(Index(i, j));
    }
    size_t NodeIndex(Index target){
        int64_t l{}, r = this->Nodes.size() - 1, mid;
        while(r >= l){
            mid = l + (r - l) / 2;
            Index &current = this->Nodes[mid].value;
            if(current.row == target.row){
                if(current.col == target.col)
                    return mid;
                else if(current.col > target.col)
                    r = mid - 1;
                else
                    l = mid + 1;
            }
            else if(current.row > target.row)
                r = mid - 1;
            else
                l = mid + 1;
        }
        return Index::noPos;
    }
    void setNeighbors(Index gsize){
        for(size_t i = 0; i < this->Nodes.size(); i++){
            size_t &row{this->Nodes[i].value.row}, 
                   &col{this->Nodes[i].value.col}, lastNeighbor{0};
            this->Nodes[i].neighbors.resize(4);                                                     // Each cell has 4 neighbors if it's not border(3) nor corner cell(2)
            auto setNeighbor = [&](Index val){
                size_t idx = NodeIndex(val);
                if(idx != Index::noPos)
                    this->Nodes[i].neighbors[lastNeighbor++] = &this->Nodes[idx];
            };
            if(row > 0)
                setNeighbor(Index(row - 1, col));
            if(row + 1 < gsize.row)
                setNeighbor(Index(row + 1, col));
            if(col > 0)
                setNeighbor(Index(row, col - 1));
            if(col + 1 < gsize.col)
                setNeighbor(Index(row, col + 1));
            if(lastNeighbor != this->Nodes[i].neighbors.size())
                this->Nodes[i].neighbors.resize(lastNeighbor);
        }
    }
public:
    UnWeightedGraph(Grid &grid, Index start, Index end): 
        Graph<NodeType>(){
        setNodes(grid);
        setNeighbors(Index(grid.size(), grid[0].size()));
        size_t idx = NodeIndex(start);
        this->StartNode = &this->Nodes[idx];
        idx = NodeIndex(end);
        this->EndNode = &this->Nodes[idx];
    }
};