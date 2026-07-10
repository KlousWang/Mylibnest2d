#pragma once
#include"Nest2D_PrivateDataType.h"
#include<vector>
#include<string>
#include <cstddef>
//
//using CetTNestItemVector = std::vector<libnest2d::Item>;
typedef void(*NestProgressCallback)(int current_finished, int total);
struct TetNestPoint
{
	double X =0.0;
	double Y =0.0;
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
	std::vector<TetNestPoint> Vertices;//顶点集合（逆时针闭合轮廓）
	// 内孔洞 (一个零件可能包含 0 个或多个孔洞，所以是二维数组)
	std::vector<std::vector<TetNestPoint>> Holes;
	//std::vector<std::string> HoleNames;//孔洞形状名字
	//算法计算输出的结果
	int Out_bin = -1; // 输出的板材编号，-1表示未成功嵌套
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

	NestProgressCallback ProgressCallback = nullptr; // 回调函数，用来看当前的排序进度
};

struct TetNestResult
{
	int Code = 0;
	std::size_t UsedBins = 0;
	std::string Message ="";
};

enum class MetClusterStrategy
{
	None = 0,
	RightTrianglePair =1,
	AutoPairCluster =2
};

enum class MetENestOrderStrategy
{
	LargeFirst=0,
	SmallFirst =1,
	LongSideFirst=2,
	ThinFirst=3
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
	// 用于粗搜后局部细搜
	double RawBOffsetX = 0.0;
	double RawBOffsetY = 0.0;

	double ClusterW = 0.0;
	double ClusterH = 0.0;

	double Score = 0.0;
};