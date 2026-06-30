#pragma once
#include"pch.h"
#include <vector>
#include <libnest2d/backends/clipper/geometries.hpp>
#include <libnest2d/libnest2d.hpp>
// 将别名定义在这里，让整个模块内部都能看到
//std::vector<int> FixedBinIds;
using CetTNestItemVector = std::vector<libnest2d::Item>;
using CetPackGround = libnest2d::_PackGroup<libnest2d::PolygonImpl>;
struct TetLib2DItemDataType
{

};
struct TetBoardBounds
{
    double MinX = 0.0;
    double MinY = 0.0;
    double MaxX = 0.0;
    double MaxY = 0.0;
    double Width = 0.0;
    double Height = 0.0;
    bool Valid = false;
};