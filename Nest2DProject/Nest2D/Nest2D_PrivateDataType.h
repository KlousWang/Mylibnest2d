#pragma once
#include "pch.h"
#include <vector>
#include <string>
#include<array>
#include <libnest2d/backends/clipper/geometries.hpp>
#include <libnest2d/libnest2d.hpp>



using CetTNestItemVector = std::vector<libnest2d::Item>;
using CetNestItem = CetTNestItemVector::value_type;
using CetPath = ClipperLib::Path;
using CetPolygonImpl = libnest2d::PolygonImpl;
using CetPackGround = libnest2d::_PackGroup<CetPolygonImpl>;
using CetInpoint = ClipperLib::IntPoint;

constexpr double CET_CLUSTER_PI = 3.14159265358979323846;
constexpr double CET_CLUSTER_HALF_PI = CET_CLUSTER_PI * 0.5;
constexpr double CET_CLUSTER_TWO_PI = CET_CLUSTER_PI * 2.0;
constexpr double CET_CLUSTER_THREE_HALF_PI = CET_CLUSTER_PI * 1.5;

constexpr double CET_CIRCLE_SIZE_TOLERANCE = 0.01;
constexpr double CET_CIRCLE_HONEYCOMB_ROW_RATIO = 0.86602540378443864676;
constexpr double CET_CIRCLE_FILLER_MIN_SIZE_RATIO = 2.0;
constexpr std::size_t CET_CIRCLE_FILLER_SINGLE_RESERVE_MIN = 8;
constexpr std::size_t CET_CIRCLE_FILLER_SINGLE_RESERVE_MAX = 24;
constexpr std::size_t CET_CIRCLE_FILL_MAX_PAIR_PROBES = 32;
constexpr std::size_t CET_CIRCLE_GAP_FILL_MAX_ACCEPTED_ITEMS = 8;
constexpr double CET_ELLIPSE_SIZE_TOLERANCE = 0.05;
constexpr double CET_ELLIPSE_HONEYCOMB_ROW_RATIO = 0.90;
constexpr double CET_RECT_SIZE_TOLERANCE = 0.05;
constexpr double CET_RECT_SQUARE_TOLERANCE = 0.01;
constexpr double CET_RECT_MAX_AREA_LOSS_RATIO = 0.10;
constexpr std::size_t CET_RECT_MAX_CLUSTER_CHILDREN = 32;
constexpr std::size_t CET_TRIANGLE_MAX_CLUSTER_CHILDREN = 32;
constexpr std::size_t CET_GENERAL_TRIANGLE_MAX_CLUSTER_CHILDREN = 32;
constexpr int CET_TRIANGLE_PAIR_PITCH_SEARCH_STEPS = 20;
constexpr double CET_ROTATION_DUPLICATE_TOLERANCE = 1e-12;
constexpr double CET_SHAPE_EPSILON = 1e-9;
constexpr double CET_GENERAL_ARC_MIN_SWEEP = CET_CLUSTER_PI / 18.0;
constexpr double CET_GENERAL_ARC_MAX_SWEEP = CET_CLUSTER_TWO_PI - CET_CLUSTER_PI / 36.0;
constexpr double CET_GENERAL_ARC_SEMI_TOLERANCE = CET_CLUSTER_PI / 36.0;
constexpr int CET_CLUSTER_BOUNDARY_OFFSET_ATTEMPTS = 5;
constexpr double CET_CLUSTER_BOUNDARY_AREA_TOLERANCE = 16.0;
constexpr double CET_CLUSTER_BOUNDARY_RELATIVE_AREA_TOLERANCE = 1e-10;
constexpr double CET_CLUSTER_GEOMETRY_AREA_TOLERANCE = 16.0;
constexpr double CET_CLUSTER_GEOMETRY_RELATIVE_AREA_TOLERANCE = 1e-10;
constexpr std::size_t CET_REMNANT_SKYLINE_SAMPLES = 64;
constexpr double CET_LARGE_ANCHOR_RATIO = 0.15;
constexpr std::size_t CET_LARGE_ANCHOR_MAX_COUNT = 8;

constexpr double CET_CUSTOM_MAX_AREA_LOSS_RATIO = 0.08;
constexpr std::size_t CET_CUSTOM_MAX_CLUSTER_CHILDREN = 64;
// Geometry validation is quadratic in the number of children. Larger orders
// reuse the selected layout in multiple clusters instead of searching a single
// oversized custom cluster.
constexpr std::size_t CET_CUSTOM_SEARCH_MAX_CHILDREN = 16;

constexpr double CET_ARC_SIZE_TOLERANCE = 0.05;
constexpr double CET_ARC_SWEEP_TOLERANCE = CET_CLUSTER_PI / 36.0;
constexpr double CET_ARC_SAFETY_GAP_RATIO = 0.001;
// Coordinates are scaled to integer nesting units.  One unit keeps paired
// contours numerically separate without turning the requested spacing into an
// extra visible clearance.
constexpr double CET_CLUSTER_MIN_SAFETY_GAP = 1.0;
constexpr std::size_t CET_ARC_MAX_CLUSTER_CHILDREN = 32;

constexpr int CET_RECTANGLE_FILL_GRID_PROBE_COUNT = 7;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_AXIS_COORDINATES = 14;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_PROBE_COUNT = 96;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_ORDER_MAX_PROBE_COUNT = 56;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS = 64;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_BASE_CANDIDATES = 8;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES = 3;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_ACCEPTED_ITEMS_PER_BASE = 8;
constexpr std::size_t CET_RECTANGLE_FILL_MEDIUM_BASE_MAX_ACCEPTED_ITEMS = 4;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_BASE_MAX_ACCEPTED_ITEMS = 2;
constexpr double CET_RECTANGLE_FILL_POSITION_TOLERANCE = 1.0;

constexpr std::size_t CET_NEST_FULL_STRATEGY_ITEM_LIMIT = 96;
constexpr std::size_t CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT = 256;

struct TetLib2DItemDataType
{
    CetTNestItemVector NestItems;
};

struct TetRemnantPartBounds
{
    double MinX = 0.0;
    double MinY = 0.0;
    double MaxX = 0.0;
    double MaxY = 0.0;
};

struct TetCustomRotationPose
{
    double Rotation = 0.0;
    double MinX = 0.0;
    double MinY = 0.0;
    double Width = 0.0;
    double Height = 0.0;
};

struct TetCustomLayoutPattern
{
    const char* Name = "";
    double ColumnPitchRatio = 1.0;
    double RowPitchRatio = 1.0;
    double RowStaggerRatio = 0.0;
    bool AlternateHalfTurn = false;
};

struct TetRowLayoutEstimate
{
    std::size_t RowCount = 0;
    double Score = 0.0;
};

struct TetRectangleGridLayout
{
    int Rows = 0;
    int Cols = 0;
    double Width = 0.0;
    double Height = 0.0;
    double Area = 0.0;
    double AspectPenalty = 0.0;
};

using TetRightTriangleRectangleLayout = TetRectangleGridLayout;
using TetGeneralTriangleLayout = TetRectangleGridLayout;
struct TetCircleExportPoint
{
    double X = 0.0;
    double Y = 0.0;
};

struct TetCircleExportInfo
{
    bool Valid = false;

    TetCircleExportPoint CenterLocal;
    TetCircleExportPoint CenterWorld;

    double Radius = 0.0;
    double VertexRadius = 0.0;
    int Segments = 0;
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
   CetInpoint Start;
   CetInpoint End;

    double Length = 0.0;
    double Angle = 0.0;
};

struct TetAutoPairItemCache
{
    double W = 0.0;
    double H = 0.0;
    double Area = 0.0;
    double FillRatio = 0.0;
    bool Worth = false;
};


enum class MetShapeType
{
    Unknown = 0,

    CircleLike,
    EllipseLike,
    TriangleLike,
    RectangleLike,
    ArcLike,

    QuadrilateralLike,
    ConvexPolygon,
    ConcavePolygon
};

struct TetCustomShapeKey
{
    MetShapeType ShapeType = MetShapeType::Unknown;
    bool HasHoles = false;
    std::vector<long long> OuterSignature;
    std::vector<std::vector<long long>> HoleSignatures;

    bool operator<(const TetCustomShapeKey& AOther) const
    {
        if (ShapeType != AOther.ShapeType){
            return static_cast<int>(ShapeType) < static_cast<int>(AOther.ShapeType);
        }
        if (HasHoles != AOther.HasHoles){
            return HasHoles < AOther.HasHoles;
        }
        if (OuterSignature != AOther.OuterSignature){
            return OuterSignature < AOther.OuterSignature;
        }
        return HoleSignatures < AOther.HoleSignatures;
    }
};

struct TetShapeBucketKey {
    MetShapeType Type = MetShapeType::Unknown;
    long long ShortSideBucket = 0;
    long long LongSideBucket = 0;
    bool operator<(const TetShapeBucketKey& AOther) const {
        const int LeftType = static_cast<int>(Type);
        const int RightType = static_cast<int>(AOther.Type);
        if (LeftType != RightType) return LeftType < RightType;
        if (ShortSideBucket != AOther.ShortSideBucket) return ShortSideBucket < AOther.ShortSideBucket;
        return LongSideBucket < AOther.LongSideBucket;
    }
};
struct TetArcCandidateLocal
{
    bool Valid = false;
    double CenterX = 0.0;
    double CenterY = 0.0;
    double OuterRadius = 0.0;
    double InnerRadius = 0.0;
    double FitError = 1.0;
    double ChordAngle = 0.0;
    int BulgeSign = 0;
   CetInpoint ChordStart;
   CetInpoint ChordEnd;
};

struct TetRectanglePose
{
    double Rotation = 0.0;
    double MinX = 0.0;
    double MinY = 0.0;
    double Width = 0.0;
    double Height = 0.0;
};

enum class MetTriangleSideType
{
    Unknown = 0,
    Equilateral,
    Isosceles,
    Scalene
};
struct TetTriangleEdgePose
{
    CetInpoint Start;
    CetInpoint End;

    double Length = 0.0;
    double Angle = 0.0;
};
struct TetBaseOffset
{
    double X = 0.0;
    double Y = 0.0;
};

struct TetCircleLayoutSlot
{
    double CenterX = 0.0;
    double CenterY = 0.0;
};

struct TetCircleLayout
{
    std::vector<TetCircleLayoutSlot> Slots;
    double Width = 0.0;
    double Height = 0.0;
    std::string ClusterType;
};

struct TetCircleIndexInfo
{
    int Index = -1;
    double SizeKey = 0.0;
};

struct TetEllipseIndexInfo
{
    int Index = -1;
    double MajorAxis = 0.0;
    double MinorAxis = 0.0;
};

struct TetEllipseLayoutSlot
{
    double X = 0.0;
    double Y = 0.0;
    bool RotateToVertical = false;
};

struct TetEllipseLayout
{
    std::vector<TetEllipseLayoutSlot> Slots;
    double Width = 0.0;
    double Height = 0.0;
    std::string ClusterType;
};

enum class MetTriangleAngleType
{
    Unknown = 0,
    Acute,
    Right,
    Obtuse
};

enum class MetArcType
{
    None = 0,
    SemiCircleLike,
    GeneralArcLike
};


enum class MetArcSweepBucket
{
    Unknown = 0,
    LessThanSemiCircle,
    SemiCircle,
    MoreThanSemiCircle
};

struct TetArcIndexInfo
{
    int Index = -1;
    MetArcType ArcType = MetArcType::None;
    MetArcSweepBucket SweepBucket = MetArcSweepBucket::Unknown;
    double Radius = 0.0;
    double ChordLength = 0.0;
    double SweepAngle = 0.0;
    int BulgeSign = 0;
};

struct TetArcOrientationBounds
{
    double Rotation = 0.0;
    double MinX = 0.0;
    double MinY = 0.0;
    double Width = 0.0;
    double Height = 0.0;
};

struct TetArcLayoutSlot
{
    double X = 0.0;
    double Y = 0.0;
    bool ReverseChordDirection = false;
};

struct TetArcLayout
{
    std::vector<TetArcLayoutSlot> Slots;
    double Width = 0.0;
    double Height = 0.0;
    std::string ClusterType;
};

struct TetCircleFitResult
{
    bool Valid = false;
    double CenterX = 0.0;
    double CenterY = 0.0;
};

struct TetAngleSpanResult
{
    bool Valid = false;
    double StartAngle = 0.0;
    double EndAngle = 0.0;
    double SweepAngle = 0.0;
};

struct TetArcChainFitResult
{
    bool Valid = false;
    double CenterX = 0.0;
    double CenterY = 0.0;
    double InnerRadius = 0.0;
    double OuterRadius = 0.0;
    double AverageError = 1.0;
    double MaxError = 1.0;
};

struct TetShapeFeature
{
    int OriginalIndex = -1;
    MetShapeType ShapeType = MetShapeType::Unknown;
  
    CetPath NormalizedContour;
    bool HasHoles = false;
    int HoleCount = 0;

    double MinX = 0.0;
    double MinY = 0.0;
    double MaxX = 0.0;
    double MaxY = 0.0;

    double Width = 0.0;
    double Height = 0.0;

    double Area = 0.0;
    double BoxArea = 0.0;

    double FillRatio = 0.0;
    double AspectRatio = 0.0;
    double Circularity = 0.0;

    int VertexCount = 0;
    bool IsConvex = false;

    
    bool IsRotatedRectangle = false;

    double OrientedWidth = 0.0;
    double OrientedHeight = 0.0;
    double OrientedAngle = 0.0;
    double OrientedBoxArea = 0.0;
    double OrientedFillRatio = 0.0;
    
    MetTriangleSideType TriangleSideType =MetTriangleSideType::Unknown;
    MetTriangleAngleType TriangleAngleType =MetTriangleAngleType::Unknown;
    
    std::array<double, 3> TriangleSides{};   
    std::array<double, 3> TriangleAngles{};

    int LongestSideIndex = -1;

    
    double EllipseMajorAxis = 0.0;
    double EllipseMinorAxis = 0.0;
    double EllipseAngle = 0.0;
    double EllipseFitError = 1.0;

    
    MetArcType ArcType = MetArcType::None;

   CetInpoint ArcChordStart{};
   CetInpoint ArcChordEnd{};
   CetInpoint ArcCenter{};

    double ArcChordLength = 0.0;
    double ArcRadius = 0.0;
    double ArcChordAngle = 0.0;
    double ArcSweepAngle = 0.0;
    double ArcFitError = 1.0;

    
    int ArcBulgeSign = 0;

    std::size_t ShapeHash = 0;
};
