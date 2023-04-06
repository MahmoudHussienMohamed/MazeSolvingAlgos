#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <stack>
#include <ctime>
#include <random>
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
            if(nx >= 0 && ny >= 0 && nx < height && ny < width && !grid[nx][ny]){
                grid[cx][cy] = (grid[cx][cy] | directions[i]);
                grid[nx][ny] = (grid[nx][ny] | opposite[directions[i]]);
                recurse(nx, ny);
            }
        }
    }
public:
    RandomMazeGenerator(size_t h, size_t w): 
        width(w/2), height(h/2), grid(height, vector<int8_t>(width)){
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
        mt19937 rnd(time(nullptr));
        recurse(rnd() % height, rnd() % width);
        vector<vector<bool>> generated;
        size_t w = width * 2, h = height * 2; 
        generated.reserve(h);
        for(size_t i = 0; i < height; i++){
            vector<bool> row;
            row.reserve(width);
            for(size_t j = 0; j < width; j++){
                row.push_back(true);
                row.push_back(!(grid[i][j] & East));
            }
            generated.push_back(row);
            row.clear();
            row.reserve(width);
            for(size_t j = 0; j < width; j++){
                row.push_back(!(grid[i][j] & South));
                row.push_back(false);
            }
            generated.push_back(row);
        }
        return generated;
    }
};