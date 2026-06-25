#pragma once
#include<vector>
#include<string>

using CetPoint = std::pair<double, double>;
using CetVertices = std::vector<CetPoint>;

struct TetPolygonData
{
    int Id = 0;                                        // 多边形编号
    std::string Name = "Polygon";
    CetVertices Vertices; // 顶点序列 (x, y)

    std::vector<CetVertices> Holes;

};
struct TetBoardData
{
    bool Enabled = false;
    CetVertices Vertices;
    std::vector<CetVertices> Holes;
};
struct TetNestDataOptions
{
    double BinWidth = 0.0;
    double BinHeight = 0.0;
    double Spacing = 0.0;
    int Rotations = 0;
    float PlacerAccuracy = 0.5f;
    int PlacerAlignment = 1;       // 0 = DONT_ALIGN, 1 = BOTTOM_LEFT
    int PlacerStartingPoint = 1;   // 0 = DONT_ALIGN, 1 = BOTTOM_LEFT

    bool PlacerParallel = true;
    bool PlacerExploreHoles = false;
};