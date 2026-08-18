#pragma once
#include "pch.h"
#include "Nest2D_DataType.h"
#include <vector>
#include <string>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <limits>
#include <array>
#include <map>
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
constexpr std::size_t CET_CIRCLE_PERIODIC_MIN_CHILD_COUNT = 5;
constexpr double CET_CIRCLE_FILLER_MIN_SIZE_RATIO = 2.0;
constexpr std::size_t CET_CIRCLE_FILLER_SINGLE_RESERVE_MIN = 8;
constexpr std::size_t CET_CIRCLE_FILLER_SINGLE_RESERVE_MAX = 24;
constexpr std::size_t CET_CIRCLE_FILL_MAX_PAIR_PROBES = 32;
constexpr std::size_t CET_CIRCLE_FILL_SPECIALIZED_PROBE_THRESHOLD = 36;
constexpr std::size_t CET_CIRCLE_GAP_TEMPLATE_MAX_COPIES = 6;
constexpr std::size_t CET_CIRCLE_GAP_FILL_MAX_ACCEPTED_ITEMS = 8;
constexpr std::size_t CET_CIRCLE_GAP_MAX_NEIGHBORS = 8;
constexpr std::size_t CET_CIRCLE_GAP_SEARCH_BEAM_WIDTH = 3;
constexpr std::size_t CET_CIRCLE_GAP_SEARCH_MAX_CANDIDATES = 8;
constexpr std::size_t CET_CIRCLE_GAP_SEARCH_MAX_ATTEMPTS = 64;
constexpr long long CET_CIRCLE_GAP_SEARCH_MAX_TIME_MS = 80;
constexpr long long CET_CIRCLE_GAP_TOTAL_SEARCH_MAX_TIME_MS = 500;
constexpr std::size_t CET_CLUSTER_GLOBAL_REBALANCE_MAX_ATTEMPTS = 48;
constexpr std::size_t CET_CLUSTER_GLOBAL_REBALANCE_MAX_UNASSIGNED_FILLERS = 12;
constexpr std::size_t CET_CLUSTER_GLOBAL_REBALANCE_MAX_TRANSFERS = 6;
constexpr long long CET_CLUSTER_GLOBAL_REBALANCE_MAX_SEARCH_TIME_MS = 600;
constexpr double CET_ELLIPSE_SIZE_TOLERANCE = 0.05;
constexpr double CET_ELLIPSE_HONEYCOMB_ROW_RATIO = 0.90;
constexpr double CET_RECT_SIZE_TOLERANCE = 0.05;
constexpr double CET_RECT_SQUARE_TOLERANCE = 0.01;
constexpr double CET_RECT_MAX_AREA_LOSS_RATIO = 0.10;
constexpr std::size_t CET_RECT_MAX_CLUSTER_CHILDREN = 32;
constexpr std::size_t CET_TRIANGLE_MAX_CLUSTER_CHILDREN = 32;
// Leave small triangle groups adaptable to the actual board remnant instead
// of committing an entire same-size family to a long rectangle grid.
constexpr std::size_t CET_TRIANGLE_REMAINDER_MAX_CLUSTER_CHILDREN = 4;
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
constexpr std::size_t CET_RECTANGLE_FILL_MAX_PROBE_COUNT = 128;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_ORDER_MAX_PROBE_COUNT = 56;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_CANDIDATE_ITEMS = 64;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_BASE_CANDIDATES = 8;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES = 3;
constexpr std::size_t CET_RECTANGLE_FILL_MAX_ACCEPTED_ITEMS_PER_BASE = 8;
constexpr std::size_t CET_RECTANGLE_FILL_MEDIUM_BASE_MAX_ACCEPTED_ITEMS = 4;
constexpr std::size_t CET_RECTANGLE_FILL_LARGE_BASE_MAX_ACCEPTED_ITEMS = 2;
constexpr double CET_RECTANGLE_FILL_POSITION_TOLERANCE = 1.0;

// Template fill search is intentionally bounded so large orders cannot turn
// candidate construction into an exhaustive subset search.
constexpr double CET_CLUSTER_FILL_MAX_PROXY_GROWTH_RATIO = 0.01;
constexpr std::size_t CET_CLUSTER_FILL_BEAM_WIDTH = 6;
constexpr std::size_t CET_CLUSTER_FILL_MAX_DEPTH = 3;
constexpr std::size_t CET_CLUSTER_FILL_MAX_CANDIDATE_FILLERS = 12;
constexpr std::size_t CET_CLUSTER_FILL_MAX_VARIANTS_PER_BASE = 8;
constexpr std::size_t CET_CLUSTER_FILL_MAX_FREE_REGIONS = 16;
constexpr std::size_t CET_CLUSTER_FILL_MAX_PLACEMENT_ATTEMPTS = 768;
constexpr std::size_t CET_CLUSTER_FILL_LARGE_ORDER_BEAM_WIDTH = 3;
constexpr std::size_t CET_CLUSTER_FILL_LARGE_ORDER_MAX_DEPTH = 2;
constexpr std::size_t CET_CLUSTER_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS = 6;
constexpr std::size_t CET_CLUSTER_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS = 192;
constexpr std::size_t CET_CLUSTER_FILL_REDUCED_ORDER_BEAM_WIDTH = 2;
constexpr std::size_t CET_CLUSTER_FILL_REDUCED_ORDER_MAX_DEPTH = 1;
constexpr std::size_t CET_CLUSTER_FILL_REDUCED_ORDER_MAX_CANDIDATE_FILLERS = 4;
constexpr std::size_t CET_CLUSTER_FILL_REDUCED_ORDER_MAX_PLACEMENT_ATTEMPTS = 96;
constexpr double CET_CLUSTER_FILL_VARIANT_POSITION_TOLERANCE = 1.0;
constexpr double CET_CLUSTER_FILL_VARIANT_ROTATION_TOLERANCE = 1e-9;
// An outer-envelope variant reserves a larger rectangle than the skeleton.
// Require a visible density gain before allowing it to compete with the
// original irregular proxy.
constexpr double CET_CLUSTER_ENVELOPE_FILL_MIN_FILL_RATIO_GAIN = 0.01;
constexpr double CET_CLUSTER_ENVELOPE_FILL_SCORE_PER_RATIO = 1000.0;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_BEAM_WIDTH = 4;
// Circle-envelope filling is intentionally restrained: fillers are individual
// small parts, not a second large triangle template inside the circle group.
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MAX_DEPTH = CET_CIRCLE_GAP_TEMPLATE_MAX_COPIES + 1;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MIN_DEPTH_BEFORE_TIMEOUT = 3;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MAX_CANDIDATE_FILLERS = 8;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MAX_PLACEMENT_ATTEMPTS = 128;
constexpr long long CET_CLUSTER_ENVELOPE_FILL_MAX_SEARCH_TIME_MS = 4000;
// Exact contour rebuilding is deferred until the rectangle-envelope beam has
// been deduplicated, avoiding repeated Clipper unions for equivalent states.
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MAX_TRUE_CONTOUR_STATES = 6;
constexpr double CET_CLUSTER_ENVELOPE_FILL_CHILD_SCORE = 60.0;
constexpr double CET_CLUSTER_ENVELOPE_FILL_TRUE_DENSITY_SCORE = 200.0;
// Exterior-envelope searches are intentionally tighter than internal fills.
// A large irregular skeleton already has many contour vertices, so probing all
// skeletons and all fillers would make this optional regularization dominate
// an otherwise bounded nesting run.
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_MAX_BASE_CANDIDATES = 3;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_BASE_CANDIDATES = 2;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_BEAM_WIDTH = 2;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_DEPTH = 1;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MIN_DEPTH_BEFORE_TIMEOUT = 1;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_CANDIDATE_FILLERS = 4;
constexpr std::size_t CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_PLACEMENT_ATTEMPTS = 48;
constexpr long long CET_CLUSTER_ENVELOPE_FILL_LARGE_ORDER_MAX_SEARCH_TIME_MS = 100;

constexpr std::size_t CET_NEST_FULL_STRATEGY_ITEM_LIMIT = 96;
constexpr std::size_t CET_NEST_REDUCED_STRATEGY_ITEM_LIMIT = 256;
constexpr int CET_LAST_BIN_MAX_TARGET_BINS_PER_ITEM = 4;
constexpr int CET_LAST_BIN_MAX_RELOCATION_CANDIDATES = 24;
constexpr int CET_LAST_BIN_MAX_RELOCATED_SMALL_ITEMS = 3;
constexpr int CET_LAST_BIN_MAX_EVACUATION_PASSES = 2;
constexpr double CET_LAST_BIN_SMALL_ITEM_AREA_RATIO = 0.01;
constexpr long long CET_REPAIR_MAX_PLACEMENT_CHECKS_PER_ITEM = 20000;
constexpr long long CET_REPAIR_MAX_TOTAL_PLACEMENT_CHECKS = 120000;
constexpr long long CET_REPAIR_MAX_SEARCH_TIME_MS = 5000;
constexpr std::size_t CET_BOARD_FILL_MAX_FREE_REGIONS = 16;
// Board-local fill uses a bounded beam so irregular edge regions can accept
// several small parts without turning repair into a subset enumeration.
constexpr std::size_t CET_BOARD_LOCAL_FILL_MAX_CANDIDATE_ITEMS = 4;
constexpr std::size_t CET_BOARD_LOCAL_FILL_LARGE_ORDER_MAX_CANDIDATE_ITEMS = 2;
constexpr std::size_t CET_BOARD_LOCAL_FILL_BEAM_WIDTH = 3;
constexpr std::size_t CET_BOARD_LOCAL_FILL_MAX_DEPTH = 3;
constexpr std::size_t CET_BOARD_LOCAL_FILL_MAX_VARIANTS_PER_REGION = 8;
constexpr long long CET_BOARD_LOCAL_FILL_MAX_PLACEMENT_CHECKS_PER_ITEM = 1200;
constexpr long long CET_BOARD_LOCAL_FILL_MAX_PLACEMENT_CHECKS_PER_REGION = 3000;
constexpr std::size_t CET_BOARD_FEEDBACK_NEST_MAX_ITEM_COUNT = 64;
// Board-composite search is deliberately small: it supplements an already
// completed global nest and must never become a second unbounded nester.
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_FREE_REGIONS_PER_BIN = 6;
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_SKELETONS_PER_REGION = 4;
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_FILLERS_PER_STATE = 6;
constexpr std::size_t CET_BOARD_COMPOSITE_BEAM_WIDTH = 3;
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_DEPTH = 2;
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_CANDIDATES_PER_BIN = 8;
// Skeleton attempts bound failed geometry expansions, which otherwise do not
// contribute to the successful-candidate limit.
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_SKELETON_ATTEMPTS_PER_BIN = 8;
constexpr long long CET_BOARD_COMPOSITE_MAX_EXACT_PLACEMENT_CHECKS = 1800;
constexpr long long CET_BOARD_COMPOSITE_MAX_GRID_PLACEMENT_CHECKS_PER_FILLER = 600;
// Reserve most per-filler checks for true-contour boundary contact poses. The
// remaining checks retain a small deterministic grid fallback.
constexpr double CET_BOARD_COMPOSITE_CONTACT_PROBE_BUDGET_RATIO = 0.80;
// Contact vertices are extremal-first and then uniformly sampled. This caps
// irregular polygon pairings independently of the source contour complexity.
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_CONTACT_VERTICES_PER_CONTOUR = 4;
// After a boundary contact, test a few short moves toward the free-region
// center. This captures narrow curved corners without adding a second grid.
constexpr std::size_t CET_BOARD_COMPOSITE_CONTACT_INSET_LEVELS = 2;
constexpr double CET_BOARD_COMPOSITE_CONTACT_INSET_STEP_RATIO = 0.25;
constexpr std::size_t CET_BOARD_COMPOSITE_MAX_ROLLBACKS_PER_BIN = 2;
// Composite repair gets its own bounded time slice because the preceding
// full-placement validation can consume the generic repair deadline.
constexpr long long CET_BOARD_COMPOSITE_MAX_SEARCH_TIME_MS = 7500;
// Each board receives a separate slice inside the total deadline so an early
// fragmented board cannot starve later boards of composite exploration.
constexpr long long CET_BOARD_COMPOSITE_MAX_SEARCH_TIME_PER_BIN_MS = 2000;
// Larger orders use one skeleton/filler expansion and fewer exact probes so
// the repair phase stays a bounded tail of the primary nesting pass.
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SKELETONS_PER_REGION = 2;
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_FILLERS_PER_STATE = 3;
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_BEAM_WIDTH = 2;
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_DEPTH = 1;
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_CANDIDATES_PER_BIN = 3;
constexpr std::size_t CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SKELETON_ATTEMPTS_PER_BIN = 4;
constexpr long long CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_EXACT_PLACEMENT_CHECKS = 600;
constexpr long long CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_GRID_PLACEMENT_CHECKS_PER_FILLER = 240;
constexpr long long CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SEARCH_TIME_MS = 5400;
constexpr long long CET_BOARD_COMPOSITE_LARGE_ORDER_MAX_SEARCH_TIME_PER_BIN_MS = 1800;
constexpr double CET_BOARD_COMPOSITE_ENVELOPE_OPPORTUNITY_WEIGHT = 2.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_FILL_WEIGHT = 1000.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_ASPECT_WEIGHT = 100.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_CONTINUITY_WEIGHT = 100.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_REMNANT_WEIGHT = 0.001;
constexpr double CET_BOARD_COMPOSITE_SCORE_SOURCE_BIN_WEIGHT = 10.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_FRAGMENTATION_WEIGHT = 100.0;
constexpr double CET_BOARD_COMPOSITE_SCORE_COMPARISON_TOLERANCE = 1.0;
constexpr long long CET_LAST_BIN_MAX_PLACEMENT_CHECKS_PER_ITEM = 30000;
constexpr long long CET_LAST_BIN_MAX_TOTAL_PLACEMENT_CHECKS = 240000;
constexpr long long CET_LAST_BIN_MAX_SEARCH_TIME_MS = 15000;


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
enum class MetENestOrderStrategy { LargeFirst = 0, SmallFirst, LongSideFirst, ThinFirst, AreaDensityFirst };

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
    std::size_t SkeletonChildCount = 0;
    bool ProxyContourNormalized = false; double ClusterWidth = 0.0; double ClusterHeight = 0.0;
    double RealArea = 0.0; double ProxyArea = 0.0; double FillRatio = 0.0; MetClusterProxyMode ProxyMode = MetClusterProxyMode::Unknown;
    double OccupiedArea = 0.0; double ReservedArea = 0.0; double ProxyWasteArea = 0.0; double ProxyWasteRatio = 0.0;
    double BoundingBoxArea = 0.0; double BoundingFillRatio = 0.0; double CompactnessRatio = 0.0; double BoardSpanRatio = 0.0;
    double SheetReuseScore = 0.0; double FragmentationRisk = 1.0; double BaselineArea = 0.0; double AreaSavingRatio = 0.0;
    double Confidence = 1.0; double Score = 0.0;
};

struct TetClusterFreeRegion {
    CetPath Contour;
    std::vector<CetPath> Holes;
    double Area = 0.0;
    double MinX = 0.0;
    double MinY = 0.0;
    double MaxX = 0.0;
    double MaxY = 0.0;
    double Width = 0.0;
    double Height = 0.0;
    bool IsClosed = false;
};

struct TetCircleGapTemplateAnchor {
    double CenterX = 0.0;
    double CenterY = 0.0;
    double Angle = 0.0;
    double Distance = 0.0;
    int NeighborCount = 0;
};

struct TetCircleGapWindow {
    double CenterX = 0.0;
    double CenterY = 0.0;
    double Angle = 0.0;
    double HalfWidth = 0.0;
    double HalfHeight = 0.0;
    std::string ClassKey;
};

struct TetCircleGapTemplate {
    TetCircleGapWindow Source;
    std::vector<TetItemTransform> Transforms;
};

using TetCircleGapTemplateCache = std::map<std::string, std::map<std::string, TetCircleGapTemplate>>;


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
    bool HasInternalGapMetric = false;
    double InternalGapArea = 0.0;
    std::size_t InternalGapCount = 0;
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

// Captures the first failed validation pair after cluster proxies have been
// expanded back into their original items.  The engine uses the packed-item
// indices to selectively dissolve only the involved clusters before falling
// back to a fully unclustered nesting pass.
struct TetExpandedSpacingFailure {
    bool Valid = false;
    bool RawContoursIntersect = false;
    int FirstOriginalIndex = -1;
    int SecondOriginalIndex = -1;
    int FirstPackedIndex = -1;
    int SecondPackedIndex = -1;
    int BinId = -1;
};

struct TetLocalBestResult {
    bool HasBest = false;
    std::size_t Layers = 0;
    TetTNestEvalResult Eval{};
    CetTNestItemVector Items;
    std::vector<TetMetaItem> MetaItems;
    bool HasCluster = false;
};

struct TetPairCandidateKey
{
    int First = -1;
    int Second = -1;

    bool operator==(const TetPairCandidateKey& AOther) const
    {
        return First == AOther.First && Second == AOther.Second;
    }
};

struct TetPairCandidateKeyHash
{
    std::size_t operator()(const TetPairCandidateKey& AKey) const noexcept
    {
        const std::size_t FirstHash = std::hash<int>{}(AKey.First);
        const std::size_t SecondHash = std::hash<int>{}(AKey.Second);
        return FirstHash ^ (SecondHash + static_cast<std::size_t>(0x9e3779b9) + (FirstHash << 6) + (FirstHash >> 2));
    }
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
    int totalItems = 0;
    NestProgressCallback callback = nullptr;

    static NestProgressCallback _NormalizeCallback(NestProgressCallback Acb)
    {
        if (Acb == nullptr) {
            return nullptr;
        }
#ifdef _WIN32
        MEMORY_BASIC_INFORMATION MemoryInfo{};
        const SIZE_T QuerySize = VirtualQuery(reinterpret_cast<const void*>(Acb), &MemoryInfo, sizeof MemoryInfo);
        if (QuerySize != sizeof MemoryInfo || MemoryInfo.State != MEM_COMMIT) {
            return nullptr;
        }
        const DWORD ExecuteMask = PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
        if ((MemoryInfo.Protect & ExecuteMask) == 0) {
            return nullptr;
        }
#endif
        return Acb;
    }

    TetNestProgressTracker(int Atotal, NestProgressCallback Acb)
        : totalItems(Atotal), callback(_NormalizeCallback(Acb)) {
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

struct TetBoardLocalFillCandidate
{
    bool Valid = false;
    int TargetBin = -1;
    TetClusterFreeRegion FreeRegion;
    std::vector<TetHoleFillCandidate> Placements;
    double OccupiedAreaGain = 0.0;
    double EnvelopeArea = 0.0;
    double EnvelopeFillRatio = 0.0;
    double Score = 0.0;
};

struct TetBoardCompositeScore
{
    double InternalFillRatio = 0.0;
    double OccupiedAreaGain = 0.0;
    double AspectMatch = 0.0;
    double RemainingShortSide = 0.0;
    double Continuity = 0.0;
    double FragmentationPenalty = 0.0;
    double ReusableRemnantValue = 0.0;
    double SourceBinReduction = 0.0;
    double Total = 0.0;
};

struct TetBoardCompositeCandidate
{
    bool Valid = false;
    bool AnchoredSkeleton = false;
    int SkeletonIndex = -1;
    int TargetBin = -1;
    TetClusterFreeRegion FreeRegion;
    TetClusterCandidate Cluster;
    std::vector<TetHoleFillCandidate> Placements;
    TetBoardCompositeScore Score;
};

struct TetBoardCompositeSearchStats
{
    std::size_t CandidateCount = 0;
    std::size_t AcceptedCount = 0;
    std::size_t RollbackCount = 0;
    std::size_t FillerCount = 0;
    std::size_t RankedAnchoredSkeletons = 0;
    std::size_t RankedMovingSkeletons = 0;
    std::size_t SkeletonBuildFailures = 0;
    std::size_t EmptyFillerSets = 0;
    std::size_t BeamExpansionFailures = 0;
    std::size_t PlacementFailures = 0;
    long long ExactPlacementChecks = 0;
};

struct TetLastBinEvacuationStats
{
    bool Started = false;
    bool Success = false;
    bool ValidationPassed = false;
    bool RolledBack = false;
    int BeforeUsedBins = 0;
    int AfterUsedBins = 0;
    int LastBinId = -1;
    int LastBinItemCount = 0;
    int RemainingItems = 0;
    int DirectMoves = 0;
    int SameBinRelocations = 0;
    int RelocatedExistingSmallItemCount = 0;
    int NoCandidatePosition = 0;
    int RelocationFailed = 0;
    bool InsufficientFreeArea = false;
    bool SearchBudgetReached = false;
    long long PlacementChecks = 0;
    double LastBinArea = 0.0;
    double TimeMs = 0.0;
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
