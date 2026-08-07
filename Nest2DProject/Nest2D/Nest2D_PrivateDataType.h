#pragma once
#include "pch.h"
#include "Nest2D_DataType.h"
#include <vector>
#include <string>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <limits>
#include <array>
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
constexpr std::size_t CET_CUSTOM_SEARCH_MAX_CHILDREN = 16;

constexpr double CET_ARC_SIZE_TOLERANCE = 0.05;
constexpr double CET_ARC_SWEEP_TOLERANCE = CET_CLUSTER_PI / 36.0;
constexpr double CET_ARC_SAFETY_GAP_RATIO = 0.001;
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


// ============================================================================
// 第一层级：毫无依赖的底层枚举 (Enums)
// ============================================================================

enum class MetClusterStrategy { None = 0, RightTrianglePair, AutoPairCluster, TemplateCluster };
enum class MetClusterProxyMode { Unknown = 0, ExactUnion, OffsetUnion, ConvexHull, RectangleFallback };
inline const char* ToString(MetClusterProxyMode AMode)
{
    switch (AMode) {
    case MetClusterProxyMode::ExactUnion: return "ExactUnion";
    case MetClusterProxyMode::OffsetUnion: return "OffsetUnion";
    case MetClusterProxyMode::ConvexHull: return "ConvexHull";
    case MetClusterProxyMode::RectangleFallback: return "RectangleFallback";
    default: return "Unknown";
    }
}
enum class MetENestOrderStrategy { LargeFirst = 0, SmallFirst, LongSideFirst, ThinFirst };

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

enum class MetTriangleSideType
{
    Unknown = 0,
    Equilateral,
    Isosceles,
    Scalene
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

// ============================================================================
// 第二层级：基础结构体 (Base Structs)
// ============================================================================

struct TetItemTransform {
    int OriginalId = -1;
    double RelativeX = 0.0;
    double RelativeY = 0.0;
    double RelativeRotation = 0.0;
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

struct TetRemnantPartBounds
{
    double MinX = 0.0;
    double MinY = 0.0;
    double MaxX = 0.0;
    double MaxY = 0.0;
};

struct TetBaseOffset
{
    double X = 0.0;
    double Y = 0.0;
};

struct TetCircleExportPoint
{
    double X = 0.0;
    double Y = 0.0;
};

struct TetEdgeInfo
{
    CetInpoint Start;
    CetInpoint End;
    double Length = 0.0;
    double Angle = 0.0;
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

struct TetThickArcTestInput {
    double CenterX;
    double CenterY;
    double OuterRadius;
    bool Horizontal;
    int SideSign;
    double ChordAngle;
    CetInpoint ChordStart;
    CetInpoint ChordEnd;
};


// ============================================================================
// 第三层级：复杂结构体 (Complex Structs)
// ============================================================================

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

    MetTriangleSideType TriangleSideType = MetTriangleSideType::Unknown;
    MetTriangleAngleType TriangleAngleType = MetTriangleAngleType::Unknown;

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

struct TetMetaItem {
    int PackedItemIndex = -1;
    bool IsCluster = false;
    std::string ClusterType = "Single";
    std::vector<TetItemTransform> TransformData;
};

struct TetClusterCandidate {
    bool Valid = false; std::string ClusterType; std::string BuilderName;
    std::vector<int> OriginalIndices; std::vector<TetItemTransform> Transforms; CetPath ProxyContour;
    bool ProxyContourNormalized = false; double ClusterWidth = 0.0; double ClusterHeight = 0.0;
    double RealArea = 0.0; double ProxyArea = 0.0; double FillRatio = 0.0; MetClusterProxyMode ProxyMode = MetClusterProxyMode::Unknown;
    double OccupiedArea = 0.0; double ReservedArea = 0.0; double ProxyWasteArea = 0.0; double ProxyWasteRatio = 0.0;
    double BoundingBoxArea = 0.0; double BoundingFillRatio = 0.0; double CompactnessRatio = 0.0; double BoardSpanRatio = 0.0;
    double SheetReuseScore = 0.0; double FragmentationRisk = 1.0; double BaselineArea = 0.0; double AreaSavingRatio = 0.0;
    double Confidence = 1.0; double Score = 0.0;
};


// ============================================================================
// 第四层级：构建器所需参数对象及其他组件
// ============================================================================

struct CustomLayoutCandidateRequest {
    const CetTNestItemVector& Items;
    const std::vector<int>& Indices;
    const TetNestOptions& Options;
    const TetCustomLayoutPattern& Pattern;
    const TetCustomRotationPose& BasePose;
    const TetCustomRotationPose& HalfTurnPose;
    std::size_t RowCount;
    std::size_t ColumnCount;
    double CellWidth;
    double CellHeight;
    double ColumnPitch;
    double RowPitch;
};

struct EllipseBuildRequest {
    const CetTNestItemVector& Items;
    const std::vector<TetShapeFeature>& Features;
    const std::vector<int>& Indices;
    const TetNestOptions& Options;
};

struct TetRectangleFillContext {
    const CetTNestItemVector& OriginalItems;
    const std::vector<TetShapeFeature>& Features;
    const TetNestOptions& Options;
    const std::vector<bool>& Used;
};

struct TetProbeContext {
    const CetTNestItemVector& OriginalItems;
    const std::vector<TetShapeFeature>& Features;
    const TetClusterCandidate& Candidate;
    int FillerIndex;
    const CetPath& RotatedFiller;
    double FillerMinX;
    double FillerMinY;
    double FillerWidth;
    double FillerHeight;
    double RequiredGap;
    double MaxX;
    double MaxY;
};

struct RightTriangleRectangleRequest {
    const CetTNestItemVector& OriginalItems;
    const std::vector<TetShapeFeature>& Features;
    const std::vector<int>& Indices;
    const TetNestOptions& Options;
    int PairCount;
    double CellWidth;
    double CellHeight;
    double AxisGap;
    double CellGap;
    double HalfTurn;
};

struct TriangleEdgePairRequest {
    const CetTNestItemVector& OriginalItems;
    const std::vector<TetShapeFeature>& Features;
    int AIndex;
    int BIndex;
    int AEdgeIndex;
    int BEdgeIndex;
    const TetNestOptions& Options;
    TetClusterCandidate& OutCandidate;
};

struct TetTNestEvalResult {
    int FirstBinCount = 0;
    double FirstBinArea = 0.0;
    int LastBinCount = 0;
    double LastBinArea = 0.0;
    std::size_t Layers = 0;
    bool HasRemnantMetrics = false;
    double ReusableRemnantArea = 0.0;
    double ReusableRemnantShortSide = 0.0;
    double SkylineWasteArea = 0.0;
    double UsedDepth = 0.0;
    bool RemnantIsTopStrip = true;
};

struct TetClusterBuildResult {
    CetTNestItemVector NestItems;
    std::vector<TetMetaItem> MetaItems;
};

struct TetLocalBestResult {
    bool HasBest = false;
    std::size_t Layers = 0;
    TetTNestEvalResult Eval{};
    CetTNestItemVector Items;
    std::vector<TetMetaItem> MetaItems;
    bool HasCluster = false;
};

struct TetAutoPairCandidate {
    bool Valid = false;
    int AIndex = -1;
    int BIndex = -1;
    double RelAX = 0.0;
    double RelAY = 0.0;
    double RelARotation = 0.0;
    double RelBX = 0.0;
    double RelBY = 0.0;
    double RelBRotation = 0.0;
    double RawBOffsetX = 0.0;
    double RawBOffsetY = 0.0;
    double ClusterW = 0.0;
    double ClusterH = 0.0;
    double Score = 0.0;
};

struct TetEdgePairContext {
    const CetTNestItemVector& OriginalItems;
    int AIndex;
    int BIndex;
    const TetNestOptions& Options;
    double RequiredGap;
    double RefLength;
    bool SimilarTrianglePair;
};

struct TetEdgeMatchState {
    double BRotation;
    double LengthMatchRatio;
    double MinLength;
    std::vector<std::pair<double, double>> BaseOffsets;
};

struct TetAutoPairContext {
    const CetTNestItemVector& OriginalItems;
    int AIndex;
    int BIndex;
    const TetNestOptions& Options;
};

struct TetClusterBoundaryResult {
    bool Success = false;
    CetPath Boundary;
    MetClusterProxyMode Mode = MetClusterProxyMode::Unknown;
    double BoundaryArea = 0.0;
};

struct TetCandidateGeometryStats {
    double MinX = (std::numeric_limits<double>::max)();
    double MinY = (std::numeric_limits<double>::max)();
    double MaxX = (std::numeric_limits<double>::lowest)();
    double MaxY = (std::numeric_limits<double>::lowest)();
    double RealArea = 0.0;
    double BaselineArea = 0.0;
};

struct TetEdgePairSearchContext {
    int FirstIndex = -1;
    int SecondIndex = -1;
    CetPath FirstContour;
    CetPath SecondContour;
    std::vector<double> AllowedRotations;
    double RequiredGap = 0.0;
};

struct TetEdgePairRequest {
    const CetTNestItemVector& OriginalItems;
    const std::vector<int>& Indices;
    const TetNestOptions& Options;
    TetClusterCandidate& OutCandidate;
};

struct TetEdgePairPlacement {
    double SecondRotation = 0.0;
    double NormalX = 0.0;
    double NormalY = 0.0;
    double TangentX = 0.0;
    double TangentY = 0.0;
    double BaseOffsetX = 0.0;
    double BaseOffsetY = 0.0;
    double NormalDirection = 0.0;
    double TangentShift = 0.0;
    double LengthRatio = 0.0;
};

struct TetEdgePairSearchResult {
    bool HasCandidate = false;
    TetClusterCandidate BestCandidate;
};

struct TriangleEdgePairGeometry {
    CetInpoint RotatedBThird;
    double ASide = 0.0;
    double RotationB = 0.0;
    double TranslationAX = 0.0;
    double TranslationAY = 0.0;
    double AStartX = 0.0;
    double AStartY = 0.0;
    double EdgeDX = 0.0;
    double EdgeDY = 0.0;
    double UnitX = 0.0;
    double UnitY = 0.0;
    double Gap = 0.0;
    double AMidX = 0.0;
    double AMidY = 0.0;
    double BMidX = 0.0;
    double BMidY = 0.0;
    double LengthMatchRatio = 0.0;
};

struct TetCircleCenter {
    double X = 0.0;
    double Y = 0.0;
    double Radius = 0.0;
};

struct TetNestProgressTracker {
    int totalItems;
    NestProgressCallback callback;
    TetNestProgressTracker(int Atotal, NestProgressCallback Acb)
        : totalItems(Atotal), callback(Acb) {
    }
    void operator()(unsigned Acnt) const {
        if (callback != nullptr) {
            callback(totalItems - static_cast<int>(Acnt), totalItems);
        }
    }
};

struct TetPlacementCandidate {
    TetPlacementCandidate()
        : ItemIndex(0), TargetBin(-1), Translation(libnest2d::Point(0, 0)), Rotation(libnest2d::Radians(0.0)) {
    }
    std::size_t ItemIndex;
    int TargetBin;
    libnest2d::Point Translation;
    libnest2d::Radians Rotation;
};

struct TetLib2DItemDataType
{
    CetTNestItemVector NestItems;
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

struct TetAutoPairItemCache
{
    double W = 0.0;
    double H = 0.0;
    double Area = 0.0;
    double FillRatio = 0.0;
    bool Worth = false;
};

struct TetCustomShapeKey
{
    MetShapeType ShapeType = MetShapeType::Unknown;
    bool HasHoles = false;
    std::vector<long long> OuterSignature;
    std::vector<std::vector<long long>> HoleSignatures;

    bool operator<(const TetCustomShapeKey& AOther) const
    {
        if (ShapeType != AOther.ShapeType) {
            return static_cast<int>(ShapeType) < static_cast<int>(AOther.ShapeType);
        }
        if (HasHoles != AOther.HasHoles) {
            return HasHoles < AOther.HasHoles;
        }
        if (OuterSignature != AOther.OuterSignature) {
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

struct TetTriangleEdgePose
{
    CetInpoint Start;
    CetInpoint End;
    double Length = 0.0;
    double Angle = 0.0;
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

struct TetCircleIndexInfo
{
    int Index = -1;
    double SizeKey = 0.0;
};