#pragma once
#include"Nest2D_PrivateDataType.h"
#include<vector>
#include<string>
#include <cstddef>
//
//using CetTNestItemVector = std::vector<libnest2d::Item>;
typedef void(*NestProgressCallback)(int Acurrent_finished, int Atotal);
struct TetNestPoint
{
	double X = 0.0;
	double Y = 0.0;
};
enum class MetNestAlignment {
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
	//int Sotations = 4;
	int Rotations = 4;

	TetNestBoard Board;
	TetNestPlacerOptions Placer;

	bool ExportSvg = false;
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

enum class MetClusterStrategy
{
	None = 0,
	RightTrianglePair = 1,
	AutoPairCluster = 2,

	TemplateCluster = 3
};

enum class MetClusterProxyMode
{
	Unknown = 0,
	ExactUnion,
	OffsetUnion,
	ConvexHull,
	RectangleFallback
};

inline const char* ToString(MetClusterProxyMode AMode)
{
	switch (AMode){
	case MetClusterProxyMode::ExactUnion:
		return "ExactUnion";
	case MetClusterProxyMode::OffsetUnion:
		return "OffsetUnion";
	case MetClusterProxyMode::ConvexHull:
		return "ConvexHull";
	case MetClusterProxyMode::RectangleFallback:
		return "RectangleFallback";
	case MetClusterProxyMode::Unknown:
	default:
		return "Unknown";
	}
}

enum class MetENestOrderStrategy
{
	LargeFirst = 0,
	SmallFirst = 1,
	LongSideFirst = 2,
	ThinFirst = 3
};

struct TetTNestEvalResult
{
	int FirstBinCount = 0;
	double FirstBinArea = 0.0;
	std::size_t Layers = 0;
};

struct TetItemTransform
{
	int OriginalId = -1;
	double RelativeX = 0.0;
	double RelativeY = 0.0;
	double RelativeRotation = 0.0;
};

struct TetMetaItem
{

	int PackedItemIndex = -1;
	bool IsCluster = false;
	std::string ClusterType = "Single";
	std::vector<TetItemTransform> TransformData;
};

struct TetClusterBuildResult
{
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
struct TetAutoPairCandidate
{
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

struct TetClusterBoundaryResult
{
	bool Success = false;
	CetPath Boundary;
	MetClusterProxyMode Mode = MetClusterProxyMode::Unknown;
	double BoundaryArea = 0.0;
};

struct TetClusterCandidate
{
	bool Valid = false;

	std::string ClusterType;
	std::string BuilderName;

	std::vector<int> OriginalIndices;
	std::vector<TetItemTransform> Transforms;

	
	CetPath ProxyContour;

	
	bool ProxyContourNormalized = false;

	double ClusterWidth = 0.0;
	double ClusterHeight = 0.0;

	double RealArea = 0.0;
	double ProxyArea = 0.0;
	double FillRatio = 0.0;
	MetClusterProxyMode ProxyMode = MetClusterProxyMode::Unknown;

	double OccupiedArea = 0.0;
	double ReservedArea = 0.0;
	double ProxyWasteArea = 0.0;
	double ProxyWasteRatio = 0.0;

	double BoundingBoxArea = 0.0;
	double BoundingFillRatio = 0.0;
	double CompactnessRatio = 0.0;
	double BoardSpanRatio = 0.0;
	double SheetReuseScore = 0.0;
	double FragmentationRisk = 1.0;

	
	double BaselineArea = 0.0;

	
	double AreaSavingRatio = 0.0;

	double Confidence = 1.0;
	double Score = 0.0;
};
