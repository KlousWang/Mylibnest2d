#include "pch.h"
#include "Nest2D_ClusterManager.h"
#include"Nest2D_DataType.h"
#include"NestUtils.h"
#include <algorithm>
#include <cmath>
#include <set>
using namespace ClipperLib;
using namespace libnest2d;
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
                        _AddSingleItem(AOriginalItems, i, Result);
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
                            if (_TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)){
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
                            _AddSingleItem(AOriginalItems, i, Result);
                        }
                    }
                    return Result;
                }

                if (AStrategy == MetClusterStrategy::AutoPairCluster) {
                    return _BuildAutoPairClusters(AOriginalItems, AOptions);
                }
                // 兜底：未知策略时，全部按单件处理
                for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
                    _AddSingleItem(AOriginalItems, i, Result);
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

					_ExpandClusterChildren(PackedItem, Meta, AOutOriginalItems);
                }
                std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. "<< "Original count = " << AOutOriginalItems.size()<< std::endl;
            }

            void CetClusterManager::_AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult)
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
          
            bool CetClusterManager::_TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult)
            {
                const auto& ItemA = AOriginalItems[AIndex];
                const auto& ItemB = AOriginalItems[BIndex];

                double WA = _GetItemWidth(ItemA);
                double HA = _GetItemHeight(ItemA);
                double WB = _GetItemWidth(ItemB);
                double HB = _GetItemHeight(ItemB);

                double AreaA = std::abs(static_cast<double>(ItemA.area()));
                double AreaB = std::abs(static_cast<double>(ItemB.area()));

                bool RightA = _IsRightTriangleLike(ItemA);
                bool RightB = _IsRightTriangleLike(ItemB);

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

                if (!_IsSameSizeTrianglePair(ItemA, ItemB)){
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
                double AxisGap = _CalcTrianglePairAxisGap(W, H, InternalSpacing);
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
                auto ClusterItem = _MakeRectangleNestItemByNestCoord(ClusterW, ClusterH);
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

            bool CetClusterManager::_NearlyEqual(double A, double B, double RelTol)
            {
                double Den = std::max(1.0, std::max(std::abs(A), std::abs(B)));
                return std::abs(A - B) <= Den * RelTol;
            }

            double CetClusterManager::_GetItemWidth(const CetNestItem& AItem)
            {
                return static_cast<double>(AItem.boundingBox().width());
            }

            double CetClusterManager::_GetItemHeight(const CetNestItem& AItem)
            {
                return static_cast<double>(AItem.boundingBox().height());
            }

            bool CetClusterManager::_IsRightTriangleLike(const CetNestItem& AItem)
            {
                double W = _GetItemWidth(AItem);
				double H = _GetItemHeight(AItem);

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

            bool CetClusterManager::_IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& BItem)
            {
                double WA = _GetItemWidth(AItem);
                double HA = _GetItemHeight(AItem);
                double WB = _GetItemWidth(BItem); 
                    double HB = _GetItemHeight(BItem);

                bool SameDirection = _NearlyEqual(WA, WB, 0.05) && _NearlyEqual(HA, HB, 0.05);
                bool SwappedDirection = _NearlyEqual(WA, HB, 0.05) && _NearlyEqual(HA, WB, 0.05);

                return SameDirection || SwappedDirection;
            }

			CetNestItem CetClusterManager::_MakeRectangleNestItemByNestCoord(double AW, double AH)
			{
				using namespace libnest2d;
				Path outerPoints;
				outerPoints.reserve(4);
				outerPoints.push_back(Point(0, 0));
				outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AW), 0));
				outerPoints.push_back(Point(static_cast<ClipperLib::cInt>(AW), static_cast<ClipperLib::cInt>(AH)));
				outerPoints.push_back(Point(0, static_cast<ClipperLib::cInt>(AH)));
				if (ClipperLib::Orientation(outerPoints) == false) {
					std::reverse(outerPoints.begin(), outerPoints.end());
				}
				Paths holes;

				PolygonImpl poly(std::move(outerPoints), std::move(holes));

				return CetTNestItemVector::value_type(std::move(poly));

			}

            double CetClusterManager::_CalcTrianglePairAxisGap(double AW, double AH, double ASpacing)
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

            void CetClusterManager::_ExpandClusterChildren(const CetNestItem& APackedItem, const TetMetaItem& AMeta, CetTNestItemVector& AOutOriginalItems)
            {
                auto PackedTranslation = APackedItem.translation();
                double PackedX = static_cast<double>(PackedTranslation.X);
                double PackedY = static_cast<double>(PackedTranslation.Y);
                double PackedRotation = APackedItem.rotation();

                // 在处理子零件前计算一次三角函数，避免在内部循环中重复计算
                double CosR = std::cos(PackedRotation);
                double SinR = std::sin(PackedRotation);

                std::cout << "[CLUSTER][EXPAND PACKED] IsCluster = " << AMeta.IsCluster
                    << ", PackedBin = " << APackedItem.binId()
                    << ", PackedX = " << PackedX
                    << ", PackedY = " << PackedY
                    << ", PackedRotation = " << PackedRotation
                    << ", Children = " << AMeta.TransformData.size()
                    << std::endl;

                for (const auto& Transform : AMeta.TransformData) {
                    int originalId = Transform.OriginalId;
                    if (originalId < 0 || originalId >= static_cast<int>(AOutOriginalItems.size())) {
                        std::cout << "[ClusTer][WARN] Invalid originalId in TransformData: " << originalId << std::endl;
                        continue;
                    }

                    auto& OriginalItem = AOutOriginalItems[originalId];

                    double LocalX = Transform.RelativeX;
                    double LocalY = Transform.RelativeY;

                    // 把组合件内部的相对坐标，跟随组合件旋转
                    double RotatedLocalX = LocalX * CosR - LocalY * SinR;
                    double RotatedLocalY = LocalX * SinR + LocalY * CosR;

                    double FinalX = PackedX + RotatedLocalX;
                    double FinalY = PackedY + RotatedLocalY;
                    double FinalRotation = PackedRotation + Transform.RelativeRotation;

                    // 继承组合件所在的板号并回填最终位置
                    OriginalItem.binId(APackedItem.binId());
                    OriginalItem.translation(ClipperLib::IntPoint(
                        static_cast<ClipperLib::cInt>(FinalX),
                        static_cast<ClipperLib::cInt>(FinalY)
                    ));
                    OriginalItem.rotation(FinalRotation);

                    std::cout << "[CLUSTER][EXPAND ITEM] OriginalId = " << originalId
                        << ", Local = (" << LocalX << ", " << LocalY << ")"
                        << ", Final = (" << FinalX << ", " << FinalY << ")"
                        << ", FinalRotation = " << FinalRotation
                        << ", Bin = " << APackedItem.binId()
                        << std::endl;
                }
            }

			TetClusterBuildResult CetClusterManager::_BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions)
			{
				TetClusterBuildResult Result;
				Result.NestItems.reserve(AOriginalItems.size());
				Result.MetaItems.reserve(AOriginalItems.size());

				const int Count = static_cast<int>(AOriginalItems.size());
				std::vector<bool> Used(Count, false);
				std::vector<TetAutoPairCandidate> AllCandidates;

				auto IsWorthAutoPair = [&](const CetNestItem& Item) -> bool {
					double W = _GetItemWidth(Item);
					double H = _GetItemHeight(Item);

					if (W <= 0.0 || H <= 0.0) {
						return false;
					}
					double BoxArea = W * H;
					double RealArea = std::abs(static_cast<double>(Item.area()));
					if (BoxArea <= 0.0 || RealArea <= 0.0) {
						return false;
					}
					double FillRatio = RealArea / BoxArea;
					return FillRatio < 0.92;
					};
				// 1. 先收集所有可行组合候选
				for (int i = 0; i < Count; ++i) {
					for (int j = i + 1; j < Count; ++j) {
						// 两个都是接近矩形的零件，先跳过
						if (!IsWorthAutoPair(AOriginalItems[i]) &&!IsWorthAutoPair(AOriginalItems[j])) {
							continue;
						}
						TetAutoPairCandidate Candidate;
						if (_TryFindBestAutoPairCandidate(AOriginalItems,i,j,AOptions,Candidate)){
							if (Candidate.Valid) {
								AllCandidates.push_back(Candidate);
							}
						}
					}
				}
				// 2. 按分数从高到低排序
				std::sort(AllCandidates.begin(),AllCandidates.end(),[](const TetAutoPairCandidate& A, const TetAutoPairCandidate& B) {
						return A.Score > B.Score;
					}
				);
				std::cout << "[AUTO_PAIR][GLOBAL] CandidateCount = "
					<< AllCandidates.size()
					<< std::endl;

				// 3. 全局选择：谁分数高谁先用，但一个零件只能被用一次
				for (const auto& Candidate : AllCandidates) {
					if (!Candidate.Valid) {
						continue;
					}

					if (Used[Candidate.AIndex] || Used[Candidate.BIndex]) {
						continue;
					}

					_AddAutoPairCluster(AOriginalItems, Candidate, Result);

					Used[Candidate.AIndex] = true;
					Used[Candidate.BIndex] = true;

					std::cout << "[AUTO_PAIR][GLOBAL ACCEPT] "
						<< Candidate.AIndex << " + " << Candidate.BIndex
						<< ", Score = " << Candidate.Score
						<< ", ClusterW = " << Candidate.ClusterW
						<< ", ClusterH = " << Candidate.ClusterH
						<< std::endl;
				}

				// 4. 没组合成功的零件，作为单件加入
				for (int i = 0; i < Count; ++i) {
					if (!Used[i]) {
						_AddSingleItem(AOriginalItems, i, Result);
						Used[i] = true;
					}
				}

				return Result;
			}

            bool CetClusterManager::_TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
            {
                if (AIndex < 0 || BIndex < 0 ||AIndex >= static_cast<int>(AOriginalItems.size()) ||BIndex >= static_cast<int>(AOriginalItems.size())){
                    return false;
                }
                const auto& ItemA = AOriginalItems[AIndex];
                const auto& ItemB = AOriginalItems[BIndex];
                double OriginWA = _GetItemWidth(ItemA);
                double OriginHA = _GetItemHeight(ItemA);
                double OriginWB = _GetItemWidth(ItemB);
                double OriginHB = _GetItemHeight(ItemB);
                if (OriginWA <= 0.0 || OriginHA <= 0.0 || OriginWB <= 0.0 || OriginHB <= 0.0) {
                    return false;
                }
                std::vector<double> Rotations;
                if (AOptions.Rotations > 0) {
                    double AngleStep = 2.0 * CET_CLUSTER_PI / AOptions.Rotations;
                    for (int r = 0; r < AOptions.Rotations; ++r) {
                        Rotations.push_back(r * AngleStep);
                    }
                }
                else {
                    Rotations.push_back(0.0);
                }

                double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));

                auto GetRotatedBBoxSize = [&](const CetNestItem& SrcItem, double Rotation, double& OutW, double& OutH) {
                    CetNestItem Tmp = SrcItem;
                    Tmp.translation(libnest2d::Point(0, 0));
                    Tmp.rotation(libnest2d::Radians(Rotation));
                    Tmp.inflation(0);
                    auto BB = Tmp.boundingBox();
                    OutW = static_cast<double>(BB.width());
                    OutH = static_cast<double>(BB.height());
                    };

                bool Found = false;

                for (double ARot : Rotations) {
                    for (double BRot : Rotations) {
                        double RotWA = 0.0, RotHA = 0.0, RotWB = 0.0, RotHB = 0.0;
                        GetRotatedBBoxSize(ItemA, ARot, RotWA, RotHA);
                        GetRotatedBBoxSize(ItemB, BRot, RotWB, RotHB);

                        if (RotWA <= 0.0 || RotHA <= 0.0 || RotWB <= 0.0 || RotHB <= 0.0) {
                            continue;
                        }

                        double BaseSize = std::min(std::min(RotWA, RotHA), std::min(RotWB, RotHB));
                        if (BaseSize <= 0.0) continue;

                        double CoarseStep = std::max(SpacingCoord > 0.0 ? SpacingCoord : 1.0, BaseSize / 4.0);
                        double FineStep = std::max(SpacingCoord > 0.0 ? SpacingCoord / 2.0 : 1.0, BaseSize / 16.0);

                        // 构造粗搜配置
                        TetAutoPairGridConfig CoarseConfig;
                        CoarseConfig.ARot = ARot;
                        CoarseConfig.BRot = BRot;
                        CoarseConfig.RotWA = RotWA;
                        CoarseConfig.RotHA = RotHA;
                        CoarseConfig.RotWB = RotWB;
                        CoarseConfig.RotHB = RotHB;
                        CoarseConfig.MinOffsetX = -RotWB - SpacingCoord;
                        CoarseConfig.MaxOffsetX = RotWA + SpacingCoord;
                        CoarseConfig.MinOffsetY = -RotHB - SpacingCoord;
                        CoarseConfig.MaxOffsetY = RotHA + SpacingCoord;
                        CoarseConfig.Step = CoarseStep;
                        CoarseConfig.MaxCheckedCount = 5000;

                        TetAutoPairCandidate CoarseBest;
                        bool CoarseFound = _RunAutoPairGridSearch(AOriginalItems, AIndex, BIndex, AOptions, CoarseConfig, CoarseBest);

                        if (!CoarseFound) {
                            continue;
                        }
                        // 复用粗搜配置，仅修改边界和步长进行精搜
                        TetAutoPairGridConfig FineConfig = CoarseConfig;
                        FineConfig.MinOffsetX = CoarseBest.RawBOffsetX - CoarseStep;
                        FineConfig.MaxOffsetX = CoarseBest.RawBOffsetX + CoarseStep;
                        FineConfig.MinOffsetY = CoarseBest.RawBOffsetY - CoarseStep;
                        FineConfig.MaxOffsetY = CoarseBest.RawBOffsetY + CoarseStep;
                        FineConfig.Step = FineStep;
                        FineConfig.MaxCheckedCount = 3000;
                        TetAutoPairCandidate FineBest;
                        bool FineFound = _RunAutoPairGridSearch(AOriginalItems, AIndex, BIndex, AOptions, FineConfig, FineBest);
                        const TetAutoPairCandidate& CurrentBest = FineFound ? FineBest : CoarseBest;
                        if (!Found || CurrentBest.Score > ABestCandidate.Score) {
                            ABestCandidate = CurrentBest;
                            Found = true;
                        }
                    }
                }

                return Found;
            }

            bool CetClusterManager::_TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems,const TetNestOptions& AOptions,const TetAutoPairBuildInput& AInput,TetAutoPairCandidate& ACandidate)
            {
                using NestItemType = CetTNestItemVector::value_type;

                if (AInput.AIndex < 0 ||
                    AInput.BIndex < 0 ||
                    AInput.AIndex >= static_cast<int>(AOriginalItems.size()) ||
                    AInput.BIndex >= static_cast<int>(AOriginalItems.size()))
                {
                    return false;
                }
                const auto& AItem = AOriginalItems[AInput.AIndex];
                const auto& BItem = AOriginalItems[AInput.BIndex];

                CetNestItem A = AItem;
                CetNestItem B = BItem;

                A.translation(libnest2d::Point(0, 0));
                A.rotation(libnest2d::Radians(AInput.ARotation));
                A.inflation(0);

                B.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(AInput.BOffsetX),static_cast<ClipperLib::cInt>(AInput.BOffsetY)));
                B.rotation(libnest2d::Radians(AInput.BRotation));
                B.inflation(0);

                double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));

                // 组合件内部 spacing 检查。
                if (SpacingCoord > 0.0) {
                    auto OldInflation = A.inflation();
                    A.inflation(static_cast<decltype(OldInflation)>(SpacingCoord));

                    if (NestItemType::intersects(A, B)) {
                        return false;
                    }

                    A.inflation(OldInflation);
                }
                else {
                    if (NestItemType::intersects(A, B)) {
                        return false;
                    }
                }

                auto BBA = A.boundingBox();
                auto BBB = B.boundingBox();
                double MinX = std::min(static_cast<double>(getX(BBA.minCorner())),static_cast<double>(getX(BBB.minCorner())));
                double MinY = std::min(static_cast<double>(getY(BBA.minCorner())),static_cast<double>(getY(BBB.minCorner())));
                double MaxX = std::max(static_cast<double>(getX(BBA.maxCorner())),static_cast<double>(getX(BBB.maxCorner())));
                double MaxY = std::max(static_cast<double>(getY(BBA.maxCorner())),static_cast<double>(getY(BBB.maxCorner())));
                double ClusterW = MaxX - MinX;
                double ClusterH = MaxY - MinY;

                if (ClusterW <= 0.0 || ClusterH <= 0.0) {
                    return false;
                }

                double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
                double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));

                if (ClusterW > BinW || ClusterH > BinH) {
                    return false;
                }

                double WA = _GetItemWidth(AItem);
                double HA = _GetItemHeight(AItem);
                double WB = _GetItemWidth(BItem);
                double HB = _GetItemHeight(BItem);

                double BeforeBBoxArea = WA * HA + WB * HB;
                double AfterBBoxArea = ClusterW * ClusterH;

                if (BeforeBBoxArea <= 0.0 || AfterBBoxArea <= 0.0) {
                    return false;
                }

                double SaveArea = BeforeBBoxArea - AfterBBoxArea;
                double SaveRatio = SaveArea / BeforeBBoxArea;
                // 组合后至少节省一点空间才接受。
                if (SaveRatio < 0.03) {
                    return false;
                }
                double RealArea =std::abs(static_cast<double>(AItem.area())) +std::abs(static_cast<double>(BItem.area()));

                double Score = _CalcAutoPairScore(BeforeBBoxArea,AfterBBoxArea,RealArea,ClusterW,ClusterH);
                ACandidate.Valid = true;
                ACandidate.AIndex = AInput.AIndex;
                ACandidate.BIndex = AInput.BIndex;

                // 归一化到组合件局部坐标。
                ACandidate.RelAX = -MinX;
                ACandidate.RelAY = -MinY;
                ACandidate.RelARotation = AInput.ARotation;
                ACandidate.RelBX = AInput.BOffsetX - MinX;
                ACandidate.RelBY = AInput.BOffsetY - MinY;
                ACandidate.RelBRotation = AInput.BRotation;

                // 保留原始搜索 offset，给细搜用
                ACandidate.RawBOffsetX = AInput.BOffsetX;
                ACandidate.RawBOffsetY = AInput.BOffsetY;

                ACandidate.ClusterW = ClusterW;
                ACandidate.ClusterH = ClusterH;
                ACandidate.Score = Score;
                return true;
            }

            void CetClusterManager::_AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult)
            {
                if (!ACandidate.Valid) {
                    return;
                }
                auto ClusterItem = _MakeUnionNestItemFromCandidate(AOriginalItems,ACandidate);
                //auto ClusterItem = _MakeRectangleNestItemByNestCoord(ACandidate.ClusterW,ACandidate.ClusterH);
               const int PackedIndex = static_cast<int>(AResult.NestItems.size());
                AResult.NestItems.push_back(std::move(ClusterItem));

                TetMetaItem Meta;
                Meta.PackedItemIndex = PackedIndex;
                Meta.IsCluster = true;
                Meta.ClusterType = "AutoPairCluster";

                TetItemTransform TransformA;
                TransformA.OriginalId = ACandidate.AIndex;
                TransformA.RelativeX = ACandidate.RelAX;
                TransformA.RelativeY = ACandidate.RelAY;
                TransformA.RelativeRotation = ACandidate.RelARotation;
                Meta.TransformData.push_back(TransformA);

                TetItemTransform TransformB;
                TransformB.OriginalId = ACandidate.BIndex;
                TransformB.RelativeX = ACandidate.RelBX;
                TransformB.RelativeY = ACandidate.RelBY;
                TransformB.RelativeRotation = ACandidate.RelBRotation;
                Meta.TransformData.push_back(TransformB);

                AResult.MetaItems.push_back(Meta);
            }

            double CetClusterManager::_CalcAutoPairScore(double ABeforeBBoxArea, double AAfterBBoxArea, double ARealArea, double AClusterW, double AClusterH)
            {
                if (ABeforeBBoxArea <= 0.0 || AAfterBBoxArea <= 0.0) {
                    return -1.0;
                }
                double SaveArea = ABeforeBBoxArea - AAfterBBoxArea;
                double SaveRatio = SaveArea / ABeforeBBoxArea;
                double FillRatio = 0.0;
                if (AAfterBBoxArea > 0.0) {
                    FillRatio = ARealArea / AAfterBBoxArea;
                }
                // 越省外包面积越好，真实面积填充率越高越好，外包周长略微惩罚。
                double Score =SaveRatio * 1000.0+ FillRatio * 100.0- (AClusterW + AClusterH) * 0.000001;
                return Score;
            }

            bool CetClusterManager::_RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& OutBest)
            {
                if (AConfig.Step <= 0.0) {
                    return false;
                }

                bool Found = false;
                int CheckedCount = 0;

                double BeforeBBoxArea = AConfig.RotWA * AConfig.RotHA + AConfig.RotWB * AConfig.RotHB;

                if (BeforeBBoxArea <= 0.0) {
                    return false;
                }

                for (double OffsetY = AConfig.MinOffsetY; OffsetY <= AConfig.MaxOffsetY; OffsetY += AConfig.Step)
                {
                    for (double OffsetX = AConfig.MinOffsetX; OffsetX <= AConfig.MaxOffsetX; OffsetX += AConfig.Step)
                    {
                        ++CheckedCount;
                        if (CheckedCount > AConfig.MaxCheckedCount) {
                            return Found;
                        }

                        // BBox 快速过滤
                        double QuickMinX = std::min(0.0, OffsetX);
                        double QuickMinY = std::min(0.0, OffsetY);

                        double QuickMaxX = std::max(AConfig.RotWA, OffsetX + AConfig.RotWB);
                        double QuickMaxY = std::max(AConfig.RotHA, OffsetY + AConfig.RotHB);

                        double QuickW = QuickMaxX - QuickMinX;
                        double QuickH = QuickMaxY - QuickMinY;

                        if (QuickW <= 0.0 || QuickH <= 0.0) {
                            continue;
                        }

                        double QuickAfterArea = QuickW * QuickH;

                        // 如果理论上都没有节省 3%，就没必要做昂贵的 intersects。
                        if (QuickAfterArea >= BeforeBBoxArea * 0.97) {
                            continue;
                        }

                        TetAutoPairBuildInput Input;
                        Input.AIndex = AIndex;
                        Input.BIndex = BIndex;
                        Input.ARotation = AConfig.ARot;
                        Input.BRotation = AConfig.BRot;
                        Input.BOffsetX = OffsetX;
                        Input.BOffsetY = OffsetY;

                        TetAutoPairCandidate Candidate;

                        if (!_TryBuildAutoPairAt(AOriginalItems, AOptions, Input, Candidate)) {
                            continue;
                        }

                        Candidate.RawBOffsetX = OffsetX;
                        Candidate.RawBOffsetY = OffsetY;

                        if (!Found || Candidate.Score > OutBest.Score) {
                            OutBest = Candidate;
                            Found = true;
                        }
                    }
                }
                return Found;
            }

			CetNestItem CetClusterManager::_MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate)
			{
				CetNestItem A = AOriginalItems[ACandidate.AIndex];
				CetNestItem B = AOriginalItems[ACandidate.BIndex];

				A.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(ACandidate.RelAX),static_cast<ClipperLib::cInt>(ACandidate.RelAY)));
				A.rotation(libnest2d::Radians(ACandidate.RelARotation));
				A.inflation(0);

				B.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(ACandidate.RelBX),static_cast<ClipperLib::cInt>(ACandidate.RelBY)));
				B.rotation(libnest2d::Radians(ACandidate.RelBRotation));
				B.inflation(0);

				// 这里需要把 A / B 的 polygon 取出来，转成 Clipper Paths
				// 然后做 union。
				ClipperLib::Paths Subject;
                ClipperLib::Paths Solution;
                _AddTransformedItemPathToSubject(AOriginalItems[ACandidate.AIndex],ACandidate.RelAX,ACandidate.RelAY,ACandidate.RelARotation,Subject);
                _AddTransformedItemPathToSubject(AOriginalItems[ACandidate.BIndex],ACandidate.RelBX,ACandidate.RelBY,ACandidate.RelBRotation,Subject);
				
				// TODO:
				// Subject.push_back(A 的外轮廓);
				// Subject.push_back(B 的外轮廓);
				ClipperLib::Clipper Clipper;
				Clipper.AddPaths(Subject, ClipperLib::ptSubject, true);
				Clipper.Execute(ClipperLib::ctUnion,Solution,ClipperLib::pftNonZero,ClipperLib::pftNonZero);

				if (Solution.empty()) {
					return _MakeRectangleNestItemByNestCoord(ACandidate.ClusterW,ACandidate.ClusterH);
				}
				// 第一阶段先取面积最大的 union 外轮廓
				auto BestIt = std::max_element(Solution.begin(),Solution.end(),[](const ClipperLib::Path& A, const ClipperLib::Path& B) {
						return std::abs(ClipperLib::Area(A)) < std::abs(ClipperLib::Area(B));
					}
				);
				ClipperLib::Path Outer = *BestIt;
				if (ClipperLib::Orientation(Outer) == false) {
					std::reverse(Outer.begin(), Outer.end());
				}
				ClipperLib::Paths Holes;
				PolygonImpl Poly(std::move(Outer), std::move(Holes));
				return CetTNestItemVector::value_type(std::move(Poly));
			}

            void CetClusterManager::_AddTransformedItemPathToSubject(const CetNestItem& AItem, double AOffsetX, double AOffsetY, double ARotation, ClipperLib::Paths& ASubject)
            {
                ClipperLib::Path Outer;

                // TODO: 替换成你项目里真实的取轮廓接口
                // Outer = AItem.transformedShape().contour();
                // 或 Outer = AItem.rawShape().contour();

                if (Outer.empty()) {
                    return;
                }

                double CosR = std::cos(ARotation);
                double SinR = std::sin(ARotation);

                ClipperLib::Path Transformed;
                Transformed.reserve(Outer.size());

                for (const auto& P : Outer) {
                    double X = static_cast<double>(P.X);
                    double Y = static_cast<double>(P.Y);

                    double RX = X * CosR - Y * SinR + AOffsetX;
                    double RY = X * SinR + Y * CosR + AOffsetY;

                    Transformed.push_back(ClipperLib::IntPoint(
                        static_cast<ClipperLib::cInt>(std::llround(RX)),
                        static_cast<ClipperLib::cInt>(std::llround(RY))
                    ));
                }

                if (ClipperLib::Orientation(Transformed) == false) {
                    std::reverse(Transformed.begin(), Transformed.end());
                }

                ASubject.push_back(std::move(Transformed));
            }

		}
}
