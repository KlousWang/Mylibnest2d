#pragma once
#include <cstddef>
#include <string>
#include <vector>
//
// using CetTNestItemVector = std::vector<libnest2d::Item>;
typedef void (*NestProgressCallback)(int Acurrent_finished, int Atotal);
struct TetNestPoint
{
    double X = 0.0;
    double Y = 0.0;
};
enum class MetNestAlignment
{
    DontAlign = 0,
    BottomLeft = 1
};
struct TetNestPlacerOptions
{
    float Accuracy = 0.5f;

    MetNestAlignment Alignment = MetNestAlignment::BottomLeft;
    MetNestAlignment StartingPoint = MetNestAlignment::BottomLeft;

    bool Parallel = true;
    bool ExploreHoles = false;
};
struct TetNestPolygon
{
    int Id = 0;
    std::string Name = "Polygon";
    std::vector<TetNestPoint> Vertices;

    std::vector<std::vector<TetNestPoint>> Holes;

    int Out_bin = -1;
    double Out_x = 0.0;
    double Out_y = 0.0;
    double Out_angle = 0.0;
};

struct TetNestBoard
{
    std::vector<TetNestPoint> Vertices;
    std::vector<std::vector<TetNestPoint>> Holes;
    bool Enabled = false;
};

struct TetNestOptions
{
    double BinWidth = 0.0;
    double BinHeight = 0.0;
    double Spacing = 1.0;
    // int Sotations = 4;
    int Rotations = 4;

    TetNestBoard Board;
    TetNestPlacerOptions Placer;

    bool ExportSvg = false;
    bool EnableLastBinEvacuation = false;
    // Optional final pass: move or rotate one target while every other placement is frozen.
    // Enabled by default so existing nest files get the compacting pass.  A
    // file can still opt out explicitly with LOCAL_COMPACT_PASS 0.
    bool EnableLocalCompactPass = true;
    std::string SvgPath = "NestingResult";

    NestProgressCallback ProgressCallback = nullptr;
};

struct TetBoardUsageResult
{
    int BinId = -1;
    int PartCount = 0;
    double BoardArea = 0.0;
    double UsedArea = 0.0;
    double UsagePercent = 0.0;

    double PureArea = 0.0;
    double PureUsagePercnt = 0.0;
};

struct TetNestResult
{
    int Code = 0;
    std::size_t UsedBins = 0;
    std::vector<TetBoardUsageResult> BoardUsages;
    std::string Message = "";
};
