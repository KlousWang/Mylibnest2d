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
            if (AOriginalItems.empty()) {
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
            if (AStrategy == MetClusterStrategy::RightTrianglePair) {
                std::vector<bool> Used(AOriginalItems.size(), false);
                for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
                    if (Used[i]) {
                        continue;
                    }
                    bool Paired = false;
                    for (int j = i + 1; j < static_cast<int>(AOriginalItems.size()); ++j) {
                        if (Used[j]) {
                            continue;
                        }
                        if (_TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)) {
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
                    if (!Paired) {
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
            for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i) {
                _AddSingleItem(AOriginalItems, i, Result);
            }
            return Result;
        }

        void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems)
        {
            AOutOriginalItems = AOriginalItems;

            if (APackedItems.size() != AMetaItems.size())
            {
                std::cout << "[CLUSTER][ERROR] PackedItems size != MetaItems size. "
                    << "PackedItems = " << APackedItems.size()<< ", MetaItems = " << AMetaItems.size()<< std::endl;
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
                    << PackedIndex<< ", IsCluster = " << Meta.IsCluster<< ", PackedBin = " << PackedItem.binId()
                    << ", PackedX = " << PackedX<< ", PackedY = " << PackedY<< ", PackedRotation = " << PackedRotation<< ", Children = " << Meta.TransformData.size()<< std::endl;

                _ExpandClusterChildren(PackedItem, Meta, AOutOriginalItems);
            }
            std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. " << "Original count = " << AOutOriginalItems.size() << std::endl;
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

            if (!RightA || !RightB) {
                std::cout << "[CLUSTER][REJECT] not right triangle: "
                    << AIndex << " + " << BIndex
                    << ", A(W,H,Area)=(" << WA << "," << HA << "," << AreaA << ")"
                    << ", B(W,H,Area)=(" << WB << "," << HB << "," << AreaB << ")"
                    << ", A ratio=" << (WA * HA > 0.0 ? AreaA * 2.0 / (WA * HA) : 0.0)
                    << ", B ratio=" << (WB * HB > 0.0 ? AreaB * 2.0 / (WB * HB) : 0.0)
                    << std::endl;

                return false;
            }

            if (!_IsSameSizeTrianglePair(ItemA, ItemB)) {
                std::cout << "[CLUSTER][REJECT] size mismatch: "
                    << AIndex << " + " << BIndex
                    << ", A(W,H)=(" << WA << "," << HA << ")"
                    << ", B(W,H)=(" << WB << "," << HB << ")"
                    << std::endl;

                return false;
            }
            double W = WA;
            double H = HA;
            if (W <= 0.0 || H <= 0.0) {
                return false;
            }

            double InternalSpacing = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
            double AxisGap = _CalcTrianglePairAxisGap(W, H, InternalSpacing);
            double ClusterW = W + AxisGap;
            double ClusterH = H + AxisGap;

            double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
            if (ClusterW > BinW || ClusterH > BinH) {
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
                    if (!IsWorthAutoPair(AOriginalItems[i]) && !IsWorthAutoPair(AOriginalItems[j])) {
                        continue;
                    }
                    TetAutoPairCandidate Candidate;
                    if (_TryFindBestAutoPairCandidate(AOriginalItems, i, j, AOptions, Candidate)) {
                        if (Candidate.Valid) {
                            AllCandidates.push_back(Candidate);
                        }
                    }
                }
            }
            // 2. 按分数从高到低排序
            std::sort(AllCandidates.begin(), AllCandidates.end(), [](const TetAutoPairCandidate& A, const TetAutoPairCandidate& B) {
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

                std::cout << "[AUTO_PAIR][GLOBAL ACCEPT] "<< Candidate.AIndex << " + " << Candidate.BIndex<< ", Score = " << Candidate.Score<< ", ClusterW = " << Candidate.ClusterW<< ", ClusterH = " << Candidate.ClusterH<< std::endl;
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

        CetPath CetClusterManager::_GetItemIdentityContour(const CetNestItem& AItem)
        {
            CetNestItem TempItem = AItem;
            TempItem.translation(libnest2d::Point(0, 0));
            TempItem.rotation(libnest2d::Radians(0.0));
            TempItem.inflation(0);

            const CetPolygonImpl& Polygon = TempItem.transformedShape();
            return Polygon.Contour;
        }

        bool CetClusterManager::_TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems,int AIndex,int BIndex,const TetNestOptions& AOptions,TetAutoPairCandidate& ABestCandidate)
        {
            if (AIndex < 0 || BIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || BIndex >= static_cast<int>(AOriginalItems.size()) || AIndex == BIndex) {
                return false;
            }
            const ClipperLib::Path ContourA = _GetItemIdentityContour(AOriginalItems[AIndex]);
            const ClipperLib::Path ContourB = _GetItemIdentityContour(AOriginalItems[BIndex]);
            const std::vector<TetEdgeInfo> EdgesA = _CollectEdges(ContourA);
            const std::vector<TetEdgeInfo> EdgesB = _CollectEdges(ContourB);
            if (EdgesA.empty() || EdgesB.empty()) return false;

            double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
            TetEdgePairContext ctx = {
                AOriginalItems, AIndex, BIndex, AOptions,
                std::max(0.0, SpacingCoord) + std::max(2.0, SpacingCoord * 0.001),
                std::max(1.0, std::min(std::max(_GetItemWidth(AOriginalItems[AIndex]), _GetItemHeight(AOriginalItems[AIndex])),
                                       std::max(_GetItemWidth(AOriginalItems[BIndex]), _GetItemHeight(AOriginalItems[BIndex])))),
                _IsSimilarTriangleByEdges(EdgesA, EdgesB)
            };

            bool Found = false;
            for (const TetEdgeInfo& EdgeA : EdgesA) {
                for (const TetEdgeInfo& EdgeB : EdgesB) {
                    if (_EvaluateEdgePair(ctx, EdgeA, EdgeB, ABestCandidate)) {
                        Found = true;
                    }
                }
            }
            return Found;
        }

        bool CetClusterManager::_TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
        {
            if (AIndex < 0 || BIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || BIndex >= static_cast<int>(AOriginalItems.size())) {
                return false;
            }
            if (_GetItemWidth(AOriginalItems[AIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[AIndex]) <= 0.0 ||_GetItemWidth(AOriginalItems[BIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[BIndex]) <= 0.0) {
                return false;
            }
            // 优先执行边缘对齐逻辑
            if (_TryFindBestEdgePairCandidate(AOriginalItems, AIndex, BIndex, AOptions, ABestCandidate)) {
                return true;
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
            TetAutoPairContext ctx = { AOriginalItems, AIndex, BIndex, AOptions };
            return _RunGridSearchAllAngles(ctx, Rotations, ABestCandidate);
        }

        bool CetClusterManager::_TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate)
        {
            using NestItemType = CetTNestItemVector::value_type;
            if (AInput.AIndex < 0 ||AInput.BIndex < 0 ||
                AInput.AIndex >= static_cast<int>(AOriginalItems.size()) ||
                AInput.BIndex >= static_cast<int>(AOriginalItems.size())){
                return false;
            }
            const auto& AItem = AOriginalItems[AInput.AIndex];
            const auto& BItem = AOriginalItems[AInput.BIndex];

            CetNestItem A = AItem;
            CetNestItem B = BItem;

            A.translation(libnest2d::Point(0, 0));
            A.rotation(libnest2d::Radians(AInput.ARotation));
            A.inflation(0);

            const ClipperLib::cInt QuantizedOffsetX =static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetX));
            const ClipperLib::cInt QuantizedOffsetY =static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetY));

            B.translation(libnest2d::Point(QuantizedOffsetX, QuantizedOffsetY));
            B.rotation(libnest2d::Radians(AInput.BRotation));
            B.inflation(0);

            const double SpacingCoord =static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));

            if (SpacingCoord > 0.0) {
                const auto OldInflation = A.inflation();
                A.inflation(static_cast<decltype(OldInflation)>(SpacingCoord));
                if (NestItemType::intersects(A, B)) {
                    return false;
                }
                A.inflation(OldInflation);
            }
            else if (NestItemType::intersects(A, B)) {
                return false;
            }

            const auto BBA = A.boundingBox();
            const auto BBB = B.boundingBox();

            const double AMinX = static_cast<double>(getX(BBA.minCorner()));
            const double AMinY = static_cast<double>(getY(BBA.minCorner()));
            const double AMaxX = static_cast<double>(getX(BBA.maxCorner()));
            const double AMaxY = static_cast<double>(getY(BBA.maxCorner()));
            const double BMinX = static_cast<double>(getX(BBB.minCorner()));
            const double BMinY = static_cast<double>(getY(BBB.minCorner()));
            const double BMaxX = static_cast<double>(getX(BBB.maxCorner()));
            const double BMaxY = static_cast<double>(getY(BBB.maxCorner()));

            const double MinX = std::min(AMinX, BMinX);
            const double MinY = std::min(AMinY, BMinY);
            const double MaxX = std::max(AMaxX, BMaxX);
            const double MaxY = std::max(AMaxY, BMaxY);

            const double ClusterW = MaxX - MinX;
            const double ClusterH = MaxY - MinY;

            if (ClusterW <= 0.0 || ClusterH <= 0.0) {
                return false;
            }

            const double BinW =static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
            const double BinH =static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));

            const bool FitsNormally = ClusterW <= BinW && ClusterH <= BinH;
            const bool FitsAfter90DegreeRotation =AOptions.Rotations > 1 && ClusterH <= BinW && ClusterW <= BinH;

            if (!FitsNormally && !FitsAfter90DegreeRotation) {
                return false;
            }

            const double RotatedBBoxAreaA =std::abs((AMaxX - AMinX) * (AMaxY - AMinY));
            const double RotatedBBoxAreaB =std::abs((BMaxX - BMinX) * (BMaxY - BMinY));

            const double BeforeBBoxArea =RotatedBBoxAreaA + RotatedBBoxAreaB;
            const double AfterBBoxArea = ClusterW * ClusterH;

            if (BeforeBBoxArea <= 0.0 || AfterBBoxArea <= 0.0) {
                return false;
            }
            const double SaveArea = BeforeBBoxArea - AfterBBoxArea;
            const double SaveRatio = SaveArea / BeforeBBoxArea;
            if (SaveRatio < 0.03) {
                return false;
            }

            const double RealArea =std::abs(static_cast<double>(AItem.area())) +std::abs(static_cast<double>(BItem.area()));

            const double Score = _CalcAutoPairScore(BeforeBBoxArea,AfterBBoxArea,RealArea,ClusterW,ClusterH);

            ACandidate.Valid = true;
            ACandidate.AIndex = AInput.AIndex;
            ACandidate.BIndex = AInput.BIndex;
            ACandidate.RelAX = -MinX;
            ACandidate.RelAY = -MinY;
            ACandidate.RelARotation = AInput.ARotation;
            ACandidate.RelBX = static_cast<double>(QuantizedOffsetX) - MinX;
            ACandidate.RelBY = static_cast<double>(QuantizedOffsetY) - MinY;
            ACandidate.RelBRotation = AInput.BRotation;
            ACandidate.RawBOffsetX = static_cast<double>(QuantizedOffsetX);
            ACandidate.RawBOffsetY = static_cast<double>(QuantizedOffsetY);
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
            auto ClusterItem = _MakeUnionNestItemFromCandidate(AOriginalItems, ACandidate);
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
            double Score = SaveRatio * 1000.0 + FillRatio * 100.0 - (AClusterW + AClusterH) * 0.000001;
            return Score;
        }

        bool CetClusterManager::_RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& OutBest)
        {
            if (AConfig.Step <= 0.0) {
                return false;
            }

            const double AWidth = AConfig.RotAMaxX - AConfig.RotAMinX;
            const double AHeight = AConfig.RotAMaxY - AConfig.RotAMinY;
            const double BWidth = AConfig.RotBMaxX - AConfig.RotBMinX;
            const double BHeight = AConfig.RotBMaxY - AConfig.RotBMinY;

            const double BeforeBBoxArea =
                AWidth * AHeight + BWidth * BHeight;

            if (BeforeBBoxArea <= 0.0) {
                return false;
            }

            bool Found = false;
            int CheckedCount = 0;

            for (double OffsetY = AConfig.MinOffsetY;
                OffsetY <= AConfig.MaxOffsetY;
                OffsetY += AConfig.Step)
            {
                for (double OffsetX = AConfig.MinOffsetX;
                    OffsetX <= AConfig.MaxOffsetX;
                    OffsetX += AConfig.Step)
                {
                    ++CheckedCount;
                    if (CheckedCount > AConfig.MaxCheckedCount) {
                        return Found;
                    }

                    const double QuickMinX = std::min(
                        AConfig.RotAMinX,
                        OffsetX + AConfig.RotBMinX);
                    const double QuickMinY = std::min(
                        AConfig.RotAMinY,
                        OffsetY + AConfig.RotBMinY);
                    const double QuickMaxX = std::max(
                        AConfig.RotAMaxX,
                        OffsetX + AConfig.RotBMaxX);
                    const double QuickMaxY = std::max(
                        AConfig.RotAMaxY,
                        OffsetY + AConfig.RotBMaxY);

                    const double QuickW = QuickMaxX - QuickMinX;
                    const double QuickH = QuickMaxY - QuickMinY;

                    if (QuickW <= 0.0 || QuickH <= 0.0) {
                        continue;
                    }

                    const double QuickAfterArea = QuickW * QuickH;
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
                    if (!_TryBuildAutoPairAt(
                        AOriginalItems,
                        AOptions,
                        Input,
                        Candidate))
                    {
                        continue;
                    }

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
            // 参数安全检查
            if (!ACandidate.Valid ||ACandidate.AIndex < 0 ||ACandidate.BIndex < 0 ||ACandidate.AIndex >= static_cast<int>(AOriginalItems.size()) ||ACandidate.BIndex >= static_cast<int>(AOriginalItems.size())){
                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }
            ClipperLib::Paths Subject;
            ClipperLib::Paths Solution;
            _AddTransformedItemPathToSubject(AOriginalItems[ACandidate.AIndex],ACandidate.RelAX,ACandidate.RelAY,ACandidate.RelARotation,Subject);
            _AddTransformedItemPathToSubject(AOriginalItems[ACandidate.BIndex],ACandidate.RelBX,ACandidate.RelBY,ACandidate.RelBRotation,Subject);

            std::cout<< "[AUTO_PAIR][UNION] SubjectPathCount = "<< Subject.size()<< std::endl;

            if (Subject.empty()) {
                std::cout<< "[AUTO_PAIR][UNION][WARN] Subject is empty."<< std::endl;

                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }

            ClipperLib::Clipper Clipper;

            if (!Clipper.AddPaths(Subject,ClipperLib::ptSubject,true))
            {
                std::cout<< "[AUTO_PAIR][UNION][WARN] Clipper.AddPaths failed."<< std::endl;
                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }

            const bool ExecuteSuccess =Clipper.Execute(ClipperLib::ctUnion,Solution,ClipperLib::pftNonZero,ClipperLib::pftNonZero);

            std::cout<< "[AUTO_PAIR][UNION] ExecuteSuccess = "<< ExecuteSuccess<< ", SolutionPathCount = "<< Solution.size()<< std::endl;

            if (!ExecuteSuccess || Solution.empty()) {
                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }

            if (Solution.size() != 1) {
                std::cout<< "[AUTO_PAIR][UNION] Multiple disconnected contours, "<< "fallback to rectangle."<< std::endl;
                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }
            ClipperLib::Path Outer =std::move(Solution.front());
            if (Outer.size() < 3) {
                return _MakeRectangleNestItemByNestCoord(std::ceil(ACandidate.ClusterW),std::ceil(ACandidate.ClusterH));
            }

            if (!ClipperLib::Orientation(Outer)) {
                std::reverse(Outer.begin(),Outer.end());
            }
            ClipperLib::Paths Holes;
            PolygonImpl Poly(std::move(Outer),std::move(Holes));
            return CetNestItem(std::move(Poly));
            return CetTNestItemVector::value_type(std::move(Poly));
        }

        void CetClusterManager::_AddTransformedItemPathToSubject(const CetNestItem& AItem, double AOffsetX, double AOffsetY, double ARotation, ClipperLib::Paths& ASubject)
        {
            // 复制一份，避免修改原始零件
            CetNestItem TempItem = AItem;

            TempItem.translation(libnest2d::Point(static_cast<ClipperLib::cInt>(std::llround(AOffsetX)),static_cast<ClipperLib::cInt>(std::llround(AOffsetY))));

            TempItem.rotation(libnest2d::Radians(ARotation));

            TempItem.inflation(0);
            // 让 libnest2d 自己完成旋转和平移
            const CetPolygonImpl& TransformedPolygon =TempItem.transformedShape();
            const CetPath& Outer =TransformedPolygon.Contour;
            if (Outer.size() < 3) {
                std::cout<< "[AUTO_PAIR][UNION][WARN] "<< "Transformed contour is empty."<< std::endl;
                return;
            }
            ASubject.push_back(Outer);

            for (const auto& Hole : TransformedPolygon.Holes) {
                if (Hole.size() >= 3) {
                    ASubject.push_back(Hole);
                }
            }
        }

        double CetClusterManager::_CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& B)
        {
            const double DX = static_cast<double>(B.X - A.X);
            const double DY = static_cast<double>(B.Y - A.Y);
            return std::sqrt(DX * DX + DY * DY);
        }

        std::vector<TetEdgeInfo> CetClusterManager::_CollectEdges(const ClipperLib::Path& AContour)
        {
            std::vector<TetEdgeInfo> Result;
            if (AContour.size() < 3) {
                return Result;
            }
            Result.reserve(AContour.size());
            for (std::size_t i = 0; i < AContour.size(); ++i) {
                const auto& Start = AContour[i];
                const auto& End = AContour[(i + 1) % AContour.size()];
                const double Length = _CalcEdgeLength(Start, End);

                if (Length <= 1.0) {
                    continue;
                }
                const double DX = static_cast<double>(End.X - Start.X);
                const double DY = static_cast<double>(End.Y - Start.Y);

                TetEdgeInfo Edge;
                Edge.Start = Start;
                Edge.End = End;
                Edge.Length = Length;
                Edge.Angle = std::atan2(DY, DX);
                Result.push_back(Edge);
            }

            std::sort(
                Result.begin(),
                Result.end(),
                [](const TetEdgeInfo& A, const TetEdgeInfo& B) {
                    return A.Length > B.Length;
                });

            constexpr std::size_t MAX_EDGE_COUNT = 24;
            if (Result.size() > MAX_EDGE_COUNT) {
                Result.resize(MAX_EDGE_COUNT);
            }

            return Result;
        }

        bool CetClusterManager::_IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> BEdges)
        {
            if (AEdges.size() != 3 || BEdges.size() != 3) {
                return false;
            }
            auto LongerFirst = [](const TetEdgeInfo& A, const TetEdgeInfo& B) {
                return A.Length > B.Length;
                };

            std::sort(AEdges.begin(), AEdges.end(), LongerFirst);
            std::sort(BEdges.begin(), BEdges.end(), LongerFirst);
            if (AEdges.front().Length <= 0.0 || BEdges.front().Length <= 0.0) {
                return false;
            }
            const double Scale =AEdges.front().Length / BEdges.front().Length;
            constexpr double SHAPE_TOLERANCE = 0.08;
            for (std::size_t i = 0; i < 3; ++i) {
                const double ScaledBLength = BEdges[i].Length * Scale;
                const double Denominator = std::max(1.0,std::max(AEdges[i].Length, ScaledBLength));
                const double RelativeError =std::abs(AEdges[i].Length - ScaledBLength) /Denominator;

                if (RelativeError > SHAPE_TOLERANCE) {
                    return false;
                }
            }

            return true;
        }

        bool CetClusterManager::_SnapToAllowedRotation(double ATarget, int ARotations, double& AOutRotation)
        {
            constexpr double MAX_ANGLE_ERROR = 0.0523598775598299; // 3 degrees
            auto NormalizeAngle = [](double AAngle) -> double {
                const double FullTurn = 2.0 * CET_CLUSTER_PI;
                AAngle = std::fmod(AAngle, FullTurn);
                return AAngle < 0.0 ? AAngle + FullTurn : AAngle;
                };
            auto AngleDistance = [&](double ALeft, double ARight) -> double {
                double Delta = std::abs(NormalizeAngle(ALeft) - NormalizeAngle(ARight));
                return std::min(Delta, 2.0 * CET_CLUSTER_PI - Delta);
                };

            ATarget = NormalizeAngle(ATarget);
            if (ARotations <= 0) {
                AOutRotation = 0.0;
                return AngleDistance(ATarget, AOutRotation) <= MAX_ANGLE_ERROR;
            }

            const double Step = 2.0 * CET_CLUSTER_PI / static_cast<double>(ARotations);
            const long long RotationIndex = std::llround(ATarget / Step);
            AOutRotation = NormalizeAngle(static_cast<double>(RotationIndex) * Step);
            return AngleDistance(ATarget, AOutRotation) <= std::min(MAX_ANGLE_ERROR, Step * 0.15);
        }

        bool CetClusterManager::_EvaluateEdgePair(const TetEdgePairContext& ctx, const TetEdgeInfo& EdgeA, const TetEdgeInfo& EdgeB, TetAutoPairCandidate& ABestCandidate)
        {
            constexpr double MIN_EDGE_MATCH_RATIO = 0.80;
            const double MaxLength = std::max(EdgeA.Length, EdgeB.Length);
            const double MinLength = std::min(EdgeA.Length, EdgeB.Length);
            if (MaxLength <= 0.0 || (MinLength / MaxLength) < MIN_EDGE_MATCH_RATIO) return false;

            const double TargetBRotation = EdgeA.Angle + CET_CLUSTER_PI - EdgeB.Angle;
            double BRotation = 0.0;
            if (!_SnapToAllowedRotation(TargetBRotation, ctx.Options.Rotations, BRotation)) return false;

            const double CosR = std::cos(BRotation), SinR = std::sin(BRotation);
            auto RotatePt = [&](const ClipperLib::IntPoint& Pt, double& OutX, double& OutY) {
                OutX = static_cast<double>(Pt.X) * CosR - static_cast<double>(Pt.Y) * SinR;
                OutY = static_cast<double>(Pt.X) * SinR + static_cast<double>(Pt.Y) * CosR;
                };

            double RotBStartX = 0.0, RotBStartY = 0.0, RotBEndX = 0.0, RotBEndY = 0.0;
            RotatePt(EdgeB.Start, RotBStartX, RotBStartY);
            RotatePt(EdgeB.End, RotBEndX, RotBEndY);

            const double AMidX = (static_cast<double>(EdgeA.Start.X) + static_cast<double>(EdgeA.End.X)) * 0.5;
            const double AMidY = (static_cast<double>(EdgeA.Start.Y) + static_cast<double>(EdgeA.End.Y)) * 0.5;
            const double BMidX = (RotBStartX + RotBEndX) * 0.5;
            const double BMidY = (RotBStartY + RotBEndY) * 0.5;

            TetEdgeMatchState state;
            state.BRotation = BRotation;
            state.LengthMatchRatio = MinLength / MaxLength;
            state.MinLength = MinLength;
            state.BaseOffsets = {
                { AMidX - BMidX, AMidY - BMidY },
                { static_cast<double>(EdgeA.Start.X) - RotBEndX, static_cast<double>(EdgeA.Start.Y) - RotBEndY },
                { static_cast<double>(EdgeA.End.X) - RotBStartX, static_cast<double>(EdgeA.End.Y) - RotBStartY }
            };

            return _TestEdgeOffsets(ctx, state, EdgeA, ABestCandidate);
        }

        bool CetClusterManager::_TestEdgeOffsets(const TetEdgePairContext& ctx, const TetEdgeMatchState& state, const TetEdgeInfo& EdgeA, TetAutoPairCandidate& ABestCandidate)
        {
            const double EdgeDX = static_cast<double>(EdgeA.End.X - EdgeA.Start.X);
            const double EdgeDY = static_cast<double>(EdgeA.End.Y - EdgeA.Start.Y);
            const double EdgeLength = std::sqrt(EdgeDX * EdgeDX + EdgeDY * EdgeDY);
            if (EdgeLength <= 0.0) return false;

            const double NormalX = -EdgeDY / EdgeLength, NormalY = EdgeDX / EdgeLength;
            bool Found = false;

            for (double Direction : { -1.0, 1.0 }) {
                for (const auto& BaseOffset : state.BaseOffsets) {
                    TetAutoPairBuildInput Input;
                    Input.AIndex = ctx.AIndex; Input.BIndex = ctx.BIndex;
                    Input.ARotation = 0.0;     Input.BRotation = state.BRotation;
                    Input.BOffsetX = BaseOffset.first + NormalX * ctx.RequiredGap * Direction;
                    Input.BOffsetY = BaseOffset.second + NormalY * ctx.RequiredGap * Direction;

                    TetAutoPairCandidate Candidate;
                    if (!_TryBuildAutoPairAt(ctx.OriginalItems, ctx.Options, Input, Candidate)) continue;

                    double EdgeCoverage = std::max(0.0, std::min(1.0, state.MinLength / ctx.RefLength));
                    Candidate.Score += state.LengthMatchRatio * 60.0 + EdgeCoverage * 40.0 + (ctx.SimilarTrianglePair ? 50.0 : 0.0);

                    if (!Found || Candidate.Score > ABestCandidate.Score) {
                        ABestCandidate = Candidate;
                        Found = true;
                    }
                }
            }
            return Found;
        }

        bool CetClusterManager::_RunGridSearchAllAngles(const TetAutoPairContext& ctx, const std::vector<double>& rotations, TetAutoPairCandidate& ABestCandidate)
        {
            bool Found = false;
            for (double ARot : rotations) {
                for (double BRot : rotations) {
                    if (_EvaluateRotationPair(ctx, ARot, BRot, ABestCandidate)) {
                        Found = true;
                    }
                }
            }
            return Found;
        }

        bool CetClusterManager::_EvaluateRotationPair(const TetAutoPairContext& ctx, double ARot, double BRot, TetAutoPairCandidate& ABestCandidate)
        {
            auto GetRotatedBBox = [&](const CetNestItem& SrcItem, double Rotation, double& OutMinX, double& OutMinY, double& OutMaxX, double& OutMaxY, double& OutW, double& OutH) {
                CetNestItem Tmp = SrcItem;
                Tmp.translation(libnest2d::Point(0, 0)); Tmp.rotation(libnest2d::Radians(Rotation)); Tmp.inflation(0);
                const auto BB = Tmp.boundingBox();
                OutMinX = static_cast<double>(getX(BB.minCorner())); OutMinY = static_cast<double>(getY(BB.minCorner()));
                OutMaxX = static_cast<double>(getX(BB.maxCorner())); OutMaxY = static_cast<double>(getY(BB.maxCorner()));
                OutW = OutMaxX - OutMinX; OutH = OutMaxY - OutMinY;
                };

            double RotAMinX = 0, RotAMinY = 0, RotAMaxX = 0, RotAMaxY = 0, RotWA = 0, RotHA = 0;
            double RotBMinX = 0, RotBMinY = 0, RotBMaxX = 0, RotBMaxY = 0, RotWB = 0, RotHB = 0;
            GetRotatedBBox(ctx.OriginalItems[ctx.AIndex], ARot, RotAMinX, RotAMinY, RotAMaxX, RotAMaxY, RotWA, RotHA);
            GetRotatedBBox(ctx.OriginalItems[ctx.BIndex], BRot, RotBMinX, RotBMinY, RotBMaxX, RotBMaxY, RotWB, RotHB);

            if (RotWA <= 0.0 || RotHA <= 0.0 || RotWB <= 0.0 || RotHB <= 0.0) return false;
            double BaseSize = std::min(std::min(RotWA, RotHA), std::min(RotWB, RotHB));
            if (BaseSize <= 0.0) return false;

            double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(ctx.Options.Spacing));
            double CoarseStep = std::max(SpacingCoord > 0.0 ? SpacingCoord : 1.0, BaseSize / 4.0);
            double FineStep = std::max(SpacingCoord > 0.0 ? SpacingCoord / 2.0 : 1.0, BaseSize / 16.0);

            TetAutoPairGridConfig CoarseConfig;
            CoarseConfig.ARot = ARot; CoarseConfig.BRot = BRot;
            CoarseConfig.RotWA = RotWA; CoarseConfig.RotHA = RotHA; CoarseConfig.RotWB = RotWB; CoarseConfig.RotHB = RotHB;
            CoarseConfig.RotAMinX = RotAMinX; CoarseConfig.RotAMinY = RotAMinY; CoarseConfig.RotAMaxX = RotAMaxX; CoarseConfig.RotAMaxY = RotAMaxY;
            CoarseConfig.RotBMinX = RotBMinX; CoarseConfig.RotBMinY = RotBMinY; CoarseConfig.RotBMaxX = RotBMaxX; CoarseConfig.RotBMaxY = RotBMaxY;
            CoarseConfig.MinOffsetX = RotAMinX - RotBMaxX - SpacingCoord; CoarseConfig.MaxOffsetX = RotAMaxX - RotBMinX + SpacingCoord;
            CoarseConfig.MinOffsetY = RotAMinY - RotBMaxY - SpacingCoord; CoarseConfig.MaxOffsetY = RotAMaxY - RotBMinY + SpacingCoord;
            CoarseConfig.Step = CoarseStep; CoarseConfig.MaxCheckedCount = 5000;

            TetAutoPairCandidate CoarseBest;
            if (!_RunAutoPairGridSearch(ctx.OriginalItems, ctx.AIndex, ctx.BIndex, ctx.Options, CoarseConfig, CoarseBest)) return false;

            TetAutoPairGridConfig FineConfig = CoarseConfig;
            FineConfig.MinOffsetX = CoarseBest.RawBOffsetX - CoarseStep; FineConfig.MaxOffsetX = CoarseBest.RawBOffsetX + CoarseStep;
            FineConfig.MinOffsetY = CoarseBest.RawBOffsetY - CoarseStep; FineConfig.MaxOffsetY = CoarseBest.RawBOffsetY + CoarseStep;
            FineConfig.Step = FineStep; FineConfig.MaxCheckedCount = 3000;

            TetAutoPairCandidate FineBest;
            bool FineFound = _RunAutoPairGridSearch(ctx.OriginalItems, ctx.AIndex, ctx.BIndex, ctx.Options, FineConfig, FineBest);
            const TetAutoPairCandidate& CurrentBest = FineFound ? FineBest : CoarseBest;

            if (CurrentBest.Score > ABestCandidate.Score) {
                ABestCandidate = CurrentBest;
                return true;
            }
            return false;
        }

   

}
}
