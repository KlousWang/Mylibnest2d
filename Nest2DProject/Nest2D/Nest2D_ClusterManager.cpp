#include "pch.h"
#include "Nest2D_ClusterManager.h"
#include"Nest2D_DataType.h"
#include"NestUtils.h"
#include <algorithm>
#include <cmath>
#include <set>
using namespace ClipperLib;
namespace ET {
        namespace NEST2DMANAGERLIB {
            constexpr double CET_CLUSTER_PI = 3.14159265358979323846;
           
            CetClusterManager::CetClusterManager() :CetCoreObject()
            {
            }

            CetClusterManager::~CetClusterManager()
            {
            }        
            TetClusterBuildResult CetClusterManager::BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy)
            {
                TetClusterBuildResult Result;
                Result.NestItems.reserve(AOriginalItems.size());
                Result.MetaItems.reserve(AOriginalItems.size());
                if (AOriginalItems.empty()){
                    return Result;
                }
                //zanshibuzuhe
                if (AStrategy == MetClusterStrategy::None) {
                    for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i)
                    {
                        AddSingleItem(AOriginalItems, i, Result);
                    }
                    return Result;
                }
                // 第二版入口：直角三角形组合
                // 现在 TryMakeRightTrianglePair 暂时返回 false，
                // 所以当前效果仍然等价于单件排样。
                if (AStrategy == MetClusterStrategy::RightTrianglePair){
                    std::vector<bool> Used(AOriginalItems.size(), false);
                    for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
                        if (Used[i]){
                            continue;
                        }
                        bool Paired = false;
                        for (int j = i + 1; j < static_cast<int>(AOriginalItems.size()); ++j){
                            if (Used[j]){
                                continue;
                            }
                            if (TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)){
                                Used[i] = true;
                                Used[j] = true;
                                Paired = true;
                                std::cout << "[CLUSTER] Pair accepted: "
                                    << i << " + " << j
                                    << ", PackedCount = " << Result.NestItems.size()
                                    << std::endl;
                                break;
                            }
                        }

                        if (!Paired){
                            Used[i] = true;
                            AddSingleItem(AOriginalItems, i, Result);
                        }
                    }
                    return Result;
                }
                // 兜底：未知策略时，全部按单件处理
                for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
                    AddSingleItem(AOriginalItems, i, Result);
                }
                return Result;
            }

            void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems)
            {
                AOutOriginalItems = AOriginalItems;

                if(APackedItems.size() != AMetaItems.size())
                {
                    std::cout << "[CLUSTER][ERROR] PackedItems size != MetaItems size. "
                        << "PackedItems = " << APackedItems.size()
                        << ", MetaItems = " << AMetaItems.size()
                        << std::endl;
                    return;
				}
                for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex) {
					const auto& PackedItem = APackedItems[PackedIndex];
                    const auto& Meta = AMetaItems[PackedIndex];

					auto PackedTranslation = PackedItem.translation();

					double PackedX = static_cast<double>(PackedTranslation.X);
					double PackedY = static_cast<double>(PackedTranslation.Y);
					double PackedRotation = PackedItem.rotation();

					double CosR = std::cos(PackedRotation);
					double SinR = std::sin(PackedRotation);
                    std::cout << "[CLUSTER][EXPAND PACKED] PackedIndex = "
                        << PackedIndex
                        << ", IsCluster = " << Meta.IsCluster
                        << ", PackedBin = " << PackedItem.binId()
                        << ", PackedX = " << PackedX
                        << ", PackedY = " << PackedY
                        << ", PackedRotation = " << PackedRotation
                        << ", Children = " << Meta.TransformData.size()
                        << std::endl;
                    for (const auto& Transform : Meta.TransformData) {
						int originalId = Transform.OriginalId;
                        if (originalId < 0 || originalId >= static_cast<int>(AOutOriginalItems.size())) {
							std::cout << "[ClusTer][WARM]Invalid originalId in TransformData: " << originalId << std::endl;
                            continue;
                        }
						auto& OriginalItem = AOutOriginalItems[originalId];

						double LocalX = Transform.RelativeX;
						double LocalY = Transform.RelativeY;

                        //把组合件内部的相对坐标，跟随组合件旋转
						double RotatedLocalX = LocalX * CosR - LocalY * SinR;
						double RotatedLocalY = LocalX * SinR + LocalY * CosR;

						double FinalX = PackedX + RotatedLocalX;
						double FinalY = PackedY + RotatedLocalY;

						double FinalRotation = PackedRotation + Transform.RelativeRotation;

                        //继承组合件所在的板号
						OriginalItem.binId(PackedItem.binId());

                        //回填原始零件最终位置
						OriginalItem.translation(ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(FinalX), static_cast<ClipperLib::cInt>(FinalY)));

						OriginalItem.rotation(FinalRotation);
                        std::cout << "[CLUSTER][EXPAND ITEM] OriginalId = "
                            << originalId
                            << ", Local = (" << LocalX << ", " << LocalY << ")"
                            << ", Final = (" << FinalX << ", " << FinalY << ")"
                            << ", FinalRotation = " << FinalRotation
                            << ", Bin = " << PackedItem.binId()
                            << std::endl;
                    }
                }
                std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. "<< "Original count = " << AOutOriginalItems.size()<< std::endl;
            }

            void CetClusterManager::AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult)
            {
                const int PackedIndex = static_cast<int>(AResult.NestItems.size());
                AResult.NestItems.push_back(AOriginalItems[AOriginalIndex]);

                TetMetaItem Meta;
                Meta.PackedItemIndex = PackedIndex;
                Meta.IsCluster = false;
                Meta.ClusterType = "Single";

                TetItemTransform Transform;
                Transform.OriginalId = AOriginalIndex;
                Transform.RelativeX = 0.0;
                Transform.RelativeY = 0.0;
                Transform.RelativeRotation = 0.0;

                Meta.TransformData.push_back(Transform);

                AResult.MetaItems.push_back(Meta);
            }
          
            bool CetClusterManager::TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult)
            {
                const auto& ItemA = AOriginalItems[AIndex];
                const auto& ItemB = AOriginalItems[BIndex];

                double WA = GetItemWidth(ItemA);
                double HA = GetItemHeight(ItemA);
                double WB = GetItemWidth(ItemB);
                double HB = GetItemHeight(ItemB);

                double AreaA = std::abs(static_cast<double>(ItemA.area()));
                double AreaB = std::abs(static_cast<double>(ItemB.area()));

                bool RightA = IsRightTriangleLike(ItemA);
                bool RightB = IsRightTriangleLike(ItemB);

                if (!RightA || !RightB){
                    std::cout << "[CLUSTER][REJECT] not right triangle: "
                        << AIndex << " + " << BIndex
                        << ", A(W,H,Area)=(" << WA << "," << HA << "," << AreaA << ")"
                        << ", B(W,H,Area)=(" << WB << "," << HB << "," << AreaB << ")"
                        << ", A ratio=" << (WA * HA > 0.0 ? AreaA * 2.0 / (WA * HA) : 0.0)
                        << ", B ratio=" << (WB * HB > 0.0 ? AreaB * 2.0 / (WB * HB) : 0.0)
                        << std::endl;

                    return false;
                }

                if (!IsSameSizeTrianglePair(ItemA, ItemB)){
                    std::cout << "[CLUSTER][REJECT] size mismatch: "
                        << AIndex << " + " << BIndex
                        << ", A(W,H)=(" << WA << "," << HA << ")"
                        << ", B(W,H)=(" << WB << "," << HB << ")"
                        << std::endl;

                    return false;
                }
                double W = WA;
                double H = HA;
                if (W <= 0.0 || H <= 0.0){
                    return false;
                }

                double InternalSpacing = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
                double AxisGap = CalcTrianglePairAxisGap(W, H, InternalSpacing);
                double ClusterW = W + AxisGap;
                double ClusterH = H + AxisGap;

                double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
                if (ClusterW > BinW || ClusterH > BinH){
                    std::cout << "[CLUSTER][REJECT] cluster bigger than bin: "
                        << AIndex << " + " << BIndex
                        << ", W = " << W
                        << ", H = " << H
                        << ", InternalSpacing = " << InternalSpacing
                        << ", AxisGap = " << AxisGap
                        << ", ClusterW = " << ClusterW
                        << ", ClusterH = " << ClusterH
                        << ", BinW = " << BinW
                        << ", BinH = " << BinH
                        << std::endl;

                    return false;
                }
                auto ClusterItem = MakeRectangleNestItemByNestCoord(ClusterW, ClusterH);
                const int PackedIndex = static_cast<int>(AResult.NestItems.size());
                AResult.NestItems.push_back(std::move(ClusterItem));

                TetMetaItem Meta;
                Meta.PackedItemIndex = PackedIndex;
                Meta.IsCluster = true;
                Meta.ClusterType = "RightTrianglePair";

                TetItemTransform TransformA;
                TransformA.OriginalId = AIndex;
                TransformA.RelativeX = 0.0;
                TransformA.RelativeY = 0.0;
                TransformA.RelativeRotation = 0.0;
                Meta.TransformData.push_back(TransformA);

                TetItemTransform TransformB;
                TransformB.OriginalId = BIndex;
                TransformB.RelativeX = ClusterW;
                TransformB.RelativeY = ClusterH;
                TransformB.RelativeRotation = CET_CLUSTER_PI;
                Meta.TransformData.push_back(TransformB);

                AResult.MetaItems.push_back(Meta);

                std::cout << "[CLUSTER] RightTrianglePair created: "
                    << AIndex << " + " << BIndex
                    << ", W = " << W
                    << ", H = " << H
                    << ", InternalSpacing = " << InternalSpacing
                    << ", AxisGap = " << AxisGap
                    << ", ClusterW = " << ClusterW
                    << ", ClusterH = " << ClusterH
                    << ", PackedIndex = " << PackedIndex
                    << std::endl;

                return true;
            }

            bool CetClusterManager::NearlyEqual(double A, double B, double RelTol)
            {
                double Den = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Den * RelTol;
            }

            double CetClusterManager::GetItemWidth(const CetNestItem& AItem)
            {
                return static_cast<double>(AItem.boundingBox().width());
            }

            double CetClusterManager::GetItemHeight(const CetNestItem& AItem)
            {
                return static_cast<double>(AItem.boundingBox().height());
            }

            bool CetClusterManager::IsRightTriangleLike(const CetNestItem& AItem)
            {
                double W = GetItemWidth(AItem);
				double H = GetItemHeight(AItem);

				if (W <= 0.0 || H <= 0.0) {
					return false;
				}

				double BoxArea = std::abs(W * H);
				double ItemArea = std::abs(static_cast<double>(AItem.area()));

				if (ItemArea <= 0.0 || BoxArea <= 0.0) {
					return false;
				}

				double Ratio = ItemArea * 2.0 / BoxArea;

				// 直角三角形：Area ≈ W * H / 2，所以 Ratio ≈ 1
				return std::abs(Ratio - 1.0) <= 0.08;
            }

            bool CetClusterManager::IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& BItem)
            {
                double WA = GetItemWidth(AItem);
                double HA = GetItemHeight(AItem);
                double WB = GetItemWidth(BItem); 
                    double HB = GetItemHeight(BItem);

                bool SameDirection = NearlyEqual(WA, WB, 0.05) && NearlyEqual(HA, HB, 0.05);
                bool SwappedDirection = NearlyEqual(WA, HB, 0.05) && NearlyEqual(HA, WB, 0.05);

                return SameDirection || SwappedDirection;
            }

			CetNestItem CetClusterManager::MakeRectangleNestItemByNestCoord(double W, double H)
			{
				using namespace libnest2d;
				Path outerPoints;
				outerPoints.reserve(4);
				outerPoints.push_back(Point(0, 0));
				outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(W), 0));
				outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(W), static_cast<ClipperLib::cInt>(H)));
				outerPoints.push_back(Point(0, static_cast<ClipperLib::cInt>(H)));
				if (ClipperLib::Orientation(outerPoints) == false) {
					std::reverse(outerPoints.begin(), outerPoints.end());
				}
				Paths holes;

				PolygonImpl poly(std::move(outerPoints), std::move(holes));

				return CetTNestItemVector::value_type(std::move(poly));

			}

            double CetClusterManager::CalcTrianglePairAxisGap(double AW, double AH, double ASpacing)
            {
                if (AW <= 0.0 || AH <= 0.0 || ASpacing <= 0.0)
                {
                    return 0.0;
                }

                // Spacing 是希望两个三角形斜边之间保留的真实间隙
                // AxisGap 是 X/Y 方向各自需要增加的距离
                double AxisGap = ASpacing * std::sqrt(AW * AW + AH * AH) / (AW + AH);

                // 因为后面会转成整数坐标，向上取整，避免实际间隙被截断变小
                return std::ceil(AxisGap);
            }

		}
}
