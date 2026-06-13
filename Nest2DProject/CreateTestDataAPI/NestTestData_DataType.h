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

