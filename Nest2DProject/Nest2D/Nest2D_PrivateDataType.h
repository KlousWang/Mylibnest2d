#pragma once
#include "pch.h"
#include <vector>
#include <string>
#include <libnest2d/backends/clipper/geometries.hpp>
#include <libnest2d/libnest2d.hpp>


// 将别名定义在这里，让整个模块内部都能看到
using CetTNestItemVector = std::vector<libnest2d::Item>;
using CetNestItem = CetTNestItemVector::value_type;
using CetPackGround = libnest2d::_PackGroup<libnest2d::PolygonImpl>;
using CetPath = ClipperLib::Path;
using CetPolygonImpl = libnest2d::PolygonImpl;
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

struct TetHoleFillCandidate 
{
	bool Valid = false;
	std::size_t ItemIndex = 0;
	int OldBin = -1;
	int TargetBin = -1;
	libnest2d::Point Translation{ 0, 0 };
	libnest2d::Radians Rotation{ 0.0 }; 
	double Score = 0.0;
};

struct TetAutoPairBuildInput
{
    int AIndex = -1;
    int BIndex = -1;

    double ARotation = 0.0;
    double BRotation = 0.0;

    double BOffsetX = 0.0;
    double BOffsetY = 0.0;
};

struct TetAutoPairGridConfig {
    double ARot = 0.0;
    double BRot = 0.0;
    double RotWA = 0.0;
    double RotHA = 0.0;
    double RotWB = 0.0;
    double RotHB = 0.0;

    double RotAMinX = 0.0;
    double RotAMinY = 0.0;
    double RotAMaxX = 0.0;
    double RotAMaxY = 0.0;

    double RotBMinX = 0.0;
    double RotBMinY = 0.0;
    double RotBMaxX = 0.0;
    double RotBMaxY = 0.0;
    double MinOffsetX = 0.0;
    double MaxOffsetX = 0.0;
    double MinOffsetY = 0.0;
    double MaxOffsetY = 0.0;
    double Step = 0.0;
    int MaxCheckedCount = 0;
};
struct TetEdgeInfo
{
    ClipperLib::IntPoint Start;
    ClipperLib::IntPoint End;

    double Length = 0.0;
    double Angle = 0.0;
};

