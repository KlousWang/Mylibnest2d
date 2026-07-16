#include "pch.h"
#include "Nest2D_ShapeAnalyzer.h"
#include<cmath>
#include<algorithm>
namespace ET {
	namespace NEST2DMANAGERLIB {
		constexpr double CET_SHAPE_PI = 3.14159265358979323846;
		constexpr double CET_SHAPE_EPSILON = 1e-9;
		CetShapeAnalyzer::CetShapeAnalyzer() :CetCoreObject()
		{
		}
		CetShapeAnalyzer::~CetShapeAnalyzer()
		{
		}
		std::vector<TetShapeFeature> CetShapeAnalyzer::AnalyzeALL(const CetTNestItemVector& AItems)
		{
			std::vector<TetShapeFeature> Features;
			Features.reserve(AItems.size());
			for (int i = 0; i < static_cast<int>(AItems.size()); ++i) {
				TetShapeFeature Feature = _AnalyzeOne(AItems[i], i);
				Features.push_back(std::move(Feature));
			}

			return Features;
		}
        TetShapeFeature CetShapeAnalyzer::_AnalyzeOne(const CetNestItem& AItem, int AOriginalIndex)
        {
            // 初始化特征结构体
            TetShapeFeature Feature;
            Feature.OriginalIndex = AOriginalIndex;
            Feature.ShapeType = MetShapeType::Unknown;
            // 复制排样项并重置变换（平移、旋转、膨胀归零）
            CetNestItem TempItem = AItem;
            TempItem.translation(libnest2d::Point(0, 0));
            TempItem.rotation(libnest2d::Radians(0.0));
            TempItem.inflation(0);
            // 提取变换后的多边形轮廓与孔洞
            const CetPolygonImpl& Polygon = TempItem.transformedShape();
            CetPath Contour = Polygon.Contour;
            ClipperLib::Paths Holes = Polygon.Holes;
            // 归一化轮廓及所有孔洞路径（去重、去共线点）
            _NormalizePath(Contour);
            for (auto& Hole : Holes) _NormalizePath(Hole);
            // 移除顶点数不足3的无效孔洞
            Holes.erase(std::remove_if(Holes.begin(), Holes.end(),
                [](const CetPath& Hole) { return Hole.size() < 3; }), Holes.end());
            // 记录顶点数，不足3则标记无效并提前返回
            Feature.VertexCount = static_cast<int>(Contour.size());
            if (Contour.size() < 3) {
                std::cout << "[SHAPE][INVALID] Index = " << AOriginalIndex << std::endl;
                return Feature;
            }
            // 计算轴对齐包围盒（AABB）
            ClipperLib::cInt MinX = Contour.front().X, MaxX = Contour.front().X;
            ClipperLib::cInt MinY = Contour.front().Y, MaxY = Contour.front().Y;
            for (const auto& Point : Contour) {
                MinX = std::min(MinX, Point.X); MaxX = std::max(MaxX, Point.X);
                MinY = std::min(MinY, Point.Y); MaxY = std::max(MaxY, Point.Y);
            }
            Feature.Width = std::abs(static_cast<double>(MaxX - MinX));
            Feature.Height = std::abs(static_cast<double>(MaxY - MinY));
            // 计算净面积（轮廓面积减去所有孔洞面积）
            double HoleArea = 0.0;
            for (const auto& Hole : Holes) HoleArea += std::abs(static_cast<double>(ClipperLib::Area(Hole)));
            Feature.Area = std::max(0.0, std::abs(static_cast<double>(ClipperLib::Area(Contour))) - HoleArea);
            // 计算填充率（净面积 / 包围盒面积）
            Feature.BoxArea = Feature.Width * Feature.Height;
            Feature.FillRatio = Feature.BoxArea > CET_SHAPE_EPSILON ? Feature.Area / Feature.BoxArea : 0.0;
            // 计算长宽比（长边 / 短边）
            const double MinSide = std::min(Feature.Width, Feature.Height);
            const double MaxSide = std::max(Feature.Width, Feature.Height);
            Feature.AspectRatio = MinSide > CET_SHAPE_EPSILON ? MaxSide / MinSide : 0.0;
            // 判断凸性
            Feature.IsConvex = _IsConvex(Contour);
            // 计算总周长（轮廓 + 所有孔洞）
            double Perimeter = _CalculatePerimeter(Contour);
            for (const auto& Hole : Holes) Perimeter += _CalculatePerimeter(Hole);
            // 计算圆度并钳制到 [0, 1]
            Feature.Circularity = Perimeter > CET_SHAPE_EPSILON ? 4.0 * CET_SHAPE_PI * Feature.Area / (Perimeter * Perimeter) : 0.0;
            Feature.Circularity = std::clamp(Feature.Circularity, 0.0, 1.0);
            // 根据特征分类形状类型
            Feature.ShapeType = _ClassifyShape(Feature, !Holes.empty());
            // 输出形状分析日志
            std::cout << "[SHAPE] Index = " << Feature.OriginalIndex
                << ", Type = " << static_cast<int>(Feature.ShapeType)
                << ", Vertices = " << Feature.VertexCount
                << ", W = " << Feature.Width
                << ", H = " << Feature.Height
                << ", Fill = " << Feature.FillRatio << std::endl;
            return Feature;
        }
	
        double CetShapeAnalyzer::_CalculatePerimeter(const CetPath& AContour)
        {
            const std::size_t Count = AContour.size();
            if (Count < 2) {
                return 0.0;
            }
            long double Perimeter = 0.0L;
            for (std::size_t i = 0; i < Count; ++i) {
                /*
                 * 使用取模让最后一个点连接回第一个点。
                 *
                 * 例如Count=3：
                 * i=0，NextIndex=1
                 * i=1，NextIndex=2
                 * i=2，NextIndex=0
                 */
                const std::size_t NextIndex =(i + 1) % Count;

                /*
                 * 调试阶段使用at()。
                 * 如果以后再次发生越界，能够更容易暴露问题。
                 */
                const ClipperLib::IntPoint& Current =AContour.at(i);
                const ClipperLib::IntPoint& Next =AContour.at(NextIndex);
                const long double DX =static_cast<long double>(Next.X) -static_cast<long double>(Current.X);
                const long double DY =static_cast<long double>(Next.Y) -static_cast<long double>(Current.Y);
                Perimeter +=std::sqrt(DX * DX + DY * DY);
            }

            return static_cast<double>(Perimeter);
        }
		//判断是否是凸多边形
		bool CetShapeAnalyzer::_IsConvex(const CetPath& AContour)
		{
			const std::size_t Count = AContour.size();
			if (Count < 3) {
				return false;
			}
			bool HasPositiveCross = false;
			bool HasNegativeCross = false;
			for (std::size_t i = 0; i < Count; ++i) {
				const ClipperLib::IntPoint& A =AContour[i];
				const ClipperLib::IntPoint& B =AContour[(i + 1) % Count];
				const ClipperLib::IntPoint& C =AContour[(i + 2) % Count];
				const long double ABX =static_cast<long double>(B.X) -static_cast<long double>(A.X);
				const long double ABY =static_cast<long double>(B.Y) -static_cast<long double>(A.Y);
				const long double BCX =static_cast<long double>(C.X) -static_cast<long double>(B.X);
			    const long double BCY =static_cast<long double>(C.Y) -static_cast<long double>(B.Y);
				const long double Cross =ABX * BCY -ABY * BCX;
				//共线点不影响凸性判断
				if (Cross > 0.0l) {
					HasPositiveCross = true;
				}
				else if(Cross<0.0l){
					HasNegativeCross = true;
				}
				//同时出现正交叉和负交叉证明缺陷存在
				if (HasPositiveCross && HasNegativeCross)return false;

			}
			return HasPositiveCross || HasNegativeCross;
		}
        void CetShapeAnalyzer::_NormalizePath(CetPath& APath)
        {
            if (APath.empty()) return;
            // 删除连续重复点
            CetPath Clean;
            Clean.reserve(APath.size());
            for (const auto& Point : APath) {
                if (!Clean.empty() && Clean.back().X == Point.X && Clean.back().Y == Point.Y) continue;
                Clean.push_back(Point);
            }
            // 删除首尾重复闭合点
            if (Clean.size() >= 2 && Clean.front().X == Clean.back().X && Clean.front().Y == Clean.back().Y)
                Clean.pop_back();
            // 删除共线的多余中间点（循环处理直到没有点被删除）
            bool Changed = true;
            while (Changed && Clean.size() > 3) {
                Changed = false;
                CetPath Result;
                const std::size_t Count = Clean.size();
                for (std::size_t i = 0; i < Count; ++i) {
                    const auto& A = Clean[(i + Count - 1) % Count];
                    const auto& B = Clean[i];
                    const auto& C = Clean[(i + 1) % Count];
                    const long double Cross = static_cast<long double>(B.X - A.X) * static_cast<long double>(C.Y - B.Y) - static_cast<long double>(B.Y - A.Y) 
                        * static_cast<long double>(C.X - B.X);
                    if (Cross == 0.0L) { Changed = true; continue; }
                    Result.push_back(B);
                }
                if (Result.size() < 3) break;
                Clean = std::move(Result);
            }
            APath = std::move(Clean);
        }
        MetShapeType CetShapeAnalyzer::_ClassifyShape(const TetShapeFeature& AFeature, bool AHasHoles)
        {
            if (AFeature.VertexCount == 3) return MetShapeType::TriangleLike;
            if (AFeature.VertexCount == 4 && AFeature.IsConvex)
                return AFeature.FillRatio >= 0.95 ? MetShapeType::RectangleLike : MetShapeType::QuadrilateralLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.90 && AFeature.AspectRatio <= 1.10)
                return MetShapeType::CircleLike;
            if (!AHasHoles && AFeature.IsConvex && AFeature.Circularity >= 0.65 && AFeature.AspectRatio > 1.10)
                return MetShapeType::EllipseLike;
            return AFeature.IsConvex ? MetShapeType::ConvexPolygon : MetShapeType::ConcavePolygon;
        }
	}
}
