#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <stack>
#include <ctime>
#include <random>
#define isOdd(x) (x % 2)
#define randBool(RNG) bool(isOdd(RNG()))
using namespace std;
class RandomMazeGenerator{
    size_t width, height;
    enum Direction{
        North = 0b0001, South = 0b0010, 
        East  = 0b0100, West  = 0b1000
    };
    Direction opposite[9];
    vector<vector<int8_t>> grid;
    int8_t dx[9];
    int8_t dy[9];

    void recurse(int64_t cx, int64_t cy){
        Direction directions[]{North, South, East, West};
        int64_t nx, ny;
        random_shuffle(directions, directions+4);
        for(size_t i = 0; i < 4; i++){
            nx = cx + dx[directions[i]];
            ny = cy + dy[directions[i]];
            if(nx >= 0 && ny >= 0 && nx < grid.size() && ny < grid[0].size() && !grid[nx][ny]){
                grid[cx][cy] = (grid[cx][cy] | directions[i]);
                grid[nx][ny] = (grid[nx][ny] | opposite[directions[i]]);
                recurse(nx, ny);
            }
        }
    }
public:
    RandomMazeGenerator(size_t h, size_t w): 
        width(w), height(h), grid(h/2, vector<int8_t>(w/2)){
        opposite[North] = South;
        opposite[South] = North;
        opposite[East] = West;
        opposite[West] = East;
        dx[North] = dx[South] = 0;
        dx[East] = 1;
        dx[West] = -1;
        dy[North] = -1;
        dy[East] = dy[West] = 0;
        dy[South] = 1;
    }
    vector<vector<bool>> generate(){
        size_t halfWidth = width / 2, halfHeight = height / 2; 
        mt19937 rnd(time(nullptr));
        recurse(rnd() % halfHeight, rnd() % halfWidth);
        vector<vector<bool>> generated;
        generated.reserve(height);
        for(size_t i = 0; i < halfHeight; i++){
            vector<bool> row;
            row.reserve(width);
            for(size_t j = 0; j < halfWidth; j++){
                if(j == halfWidth - 1)
                    row.push_back(!(grid[i][j-1] & West));
                else
                    row.push_back(true);
                row.push_back(!(grid[i][j] & East));
            }
            if(isOdd(width))
                row.push_back(!(grid[i].back() & East) || randBool(rnd));
            generated.push_back(row);
            row.clear();
            row.reserve(width);
            for(size_t j = 0; j < halfWidth; j++){
                row.push_back(!(grid[i][j] & South));
                if(j == halfWidth - 1)
                    row.push_back(!(grid[i][j-1] & West));
                else
                    row.push_back(false);
            }
            if(isOdd(width))
                row.push_back(!(grid[i].back() & East) || randBool(rnd));
            generated.push_back(row);
        }
        if(isOdd(height)){
            vector<bool> row(width);
            vector<int8_t> &lastRow{grid.back()};
            for(size_t j = 0; j < width; j++)
                row[j] = !(lastRow[j/2] & South) || randBool(rnd);
            generated.push_back(row);
        }
        return generated;
    }
};