#pragma once
#include "pch.h"
#include <vector>
#include <string>
#include<array>
#include <libnest2d/backends/clipper/geometries.hpp>
#include <libnest2d/libnest2d.hpp>


// 将别名定义在这里，让整个模块内部都能看到
using CetTNestItemVector = std::vector<libnest2d::Item>;
using CetNestItem = CetTNestItemVector::value_type;
using CetPackGround = libnest2d::_PackGroup<libnest2d::PolygonImpl>;
using CetPath = ClipperLib::Path;
using CetPolygonImpl = libnest2d::PolygonImpl;
using CetInpoint = ClipperLib::IntPoint;

constexpr double CET_CLUSTER_PI = 3.14159265358979323846;
constexpr double CET_CLUSTER_HALF_PI = CET_CLUSTER_PI * 0.5;
constexpr double CET_CLUSTER_TWO_PI = CET_CLUSTER_PI * 2.0;
constexpr double CET_CLUSTER_THREE_HALF_PI = CET_CLUSTER_PI * 1.5;
struct TetLib2DItemDataType
{

};
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
    ClipperLib::IntPoint Start;
    ClipperLib::IntPoint End;

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

//cluster使用
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
struct TetShapeBucketKey {
    MetShapeType Type = MetShapeType::Unknown;
    long long ShortSideBucket = 0;
    long long LongSideBucket = 0;
    bool operator<(const TetShapeBucketKey& Other) const {
        const int LeftType = static_cast<int>(Type);
        const int RightType = static_cast<int>(Other.Type);
        if (LeftType != RightType) return LeftType < RightType;
        if (ShortSideBucket != Other.ShortSideBucket) return ShortSideBucket < Other.ShortSideBucket;
        return LongSideBucket < Other.LongSideBucket;
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
    ClipperLib::IntPoint ChordStart;
    ClipperLib::IntPoint ChordEnd;
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

struct TetGapFillCircleCenter
{
    double X = 0.0;
    double Y = 0.0;
    double Size = 0.0;
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

// ---------- 弧形组合与识别 ----------
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

    // 归一化后的原始轮廓。
    // 后面的三角形、旋转矩形和弧形 Builder 都会使用。
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

    // ---------- 旋转矩形 ----------
    bool IsRotatedRectangle = false;

    double OrientedWidth = 0.0;
    double OrientedHeight = 0.0;
    double OrientedAngle = 0.0;
    double OrientedBoxArea = 0.0;
    double OrientedFillRatio = 0.0;

    // ---------- 三角形 ----------
    MetTriangleSideType TriangleSideType =
        MetTriangleSideType::Unknown;

    MetTriangleAngleType TriangleAngleType =
        MetTriangleAngleType::Unknown;

    // 由小到大排序后的三条边
    std::array<double, 3> TriangleSides{};

    // 对应三个顶点的角度，单位弧度
    std::array<double, 3> TriangleAngles{};

    int LongestSideIndex = -1;

    // ---------- 椭圆 ----------
    double EllipseMajorAxis = 0.0;
    double EllipseMinorAxis = 0.0;
    double EllipseAngle = 0.0;
    double EllipseFitError = 1.0;

    // ---------- 弧形 ----------
    MetArcType ArcType = MetArcType::None;

    ClipperLib::IntPoint ArcChordStart{};
    ClipperLib::IntPoint ArcChordEnd{};
    ClipperLib::IntPoint ArcCenter{};

    double ArcChordLength = 0.0;
    double ArcRadius = 0.0;
    double ArcChordAngle = 0.0;
    double ArcSweepAngle = 0.0;
    double ArcFitError = 1.0;

    // 弧线位于有向弦的哪一侧
    int ArcBulgeSign = 0;

    std::size_t ShapeHash = 0;
};
