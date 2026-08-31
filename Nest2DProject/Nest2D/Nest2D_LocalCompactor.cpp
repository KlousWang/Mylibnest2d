#include "pch.h"
#include "Nest2D_LocalCompactor.h"
#include "Nest2D_BoardUtils.h"
#include "Nest2D_RotationUtils.h"
#include "Nest2D_SelfFunction.h"
#include "NestUtils.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetLocalCompactor::CetLocalCompactor() : CetCoreObject() {}
        CetLocalCompactor::~CetLocalCompactor() {}
        bool CetLocalCompactor::_ValidatePlacedItemsSpacing(const CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            const auto SpacingCoord = NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing));
            for (std::size_t FirstIndex = 0; FirstIndex < AItems.size(); ++FirstIndex) {
                const CetNestItem &SourceItem = AItems[FirstIndex];
                if (SourceItem.binId() < 0) {
                    std::cout << "[NEST][SPACING][REJECT] Item " << FirstIndex << " was not placed on a bin." << std::endl;
                    return false;
                }
                CetNestItem FirstItem = SourceItem;
                FirstItem.inflation(0);
                if (SpacingCoord > 0) {
                    FirstItem.inflation(static_cast<decltype(FirstItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
                }
                for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < AItems.size(); ++SecondIndex) {
                    if (AItems[SecondIndex].binId() != SourceItem.binId()) {
                        continue;
                    }
                    CetNestItem SecondItem = AItems[SecondIndex];
                    SecondItem.inflation(0);
                    if (SpacingCoord > 0) {
                        SecondItem.inflation(static_cast<decltype(SecondItem.inflation())>(std::ceil(static_cast<double>(SpacingCoord) * 0.5)));
                    }
                    // libnest2d packs every item with half of the requested spacing.
                    // At exactly the requested clearance those expanded outlines touch;
                    // touching is legal, while an interior intersection is not.
                    if (CetNestItem::intersects(FirstItem, SecondItem) && !CetNestItem::touches(FirstItem, SecondItem)) {
                        std::cout << "[NEST][SPACING][REJECT] " << (SpacingCoord > 0 ? "Spacing violation" : "Overlap") << " between items " << FirstIndex << " and " << SecondIndex << " on bin " << SourceItem.binId() << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }
        bool CetLocalCompactor::_BuildBoardPath(const std::vector<TetNestPoint> &AVertices, bool AOuter, CetPath &AOutPath)
        {
            return Nest2DUtils->Nest2DBord->BuildBoardPath(AVertices, AOuter, AOutPath);
        }
        bool CetLocalCompactor::_BuildBoardSubjectContours(const TetNestOptions &AOptions, ClipperLib::Paths &AOutContours)
        {
            return Nest2DUtils->Nest2DBord->BuildBoardSubjectContours(AOptions, AOutContours);
        }
        bool CetLocalCompactor::_BuildPlacedReservedContours(const CetTNestItemVector &AItems, int ABinId, double ASpacing, ClipperLib::Paths &AOutContours, libnest2d::Coord AExtraInflation)
        {
            return Nest2DUtils->Nest2DBord->BuildPlacedReservedContours(AItems, ABinId, ASpacing, AOutContours, AExtraInflation);
        }
        double CetLocalCompactor::_LocalCompactAngleDistance(double ALeft, double ARight)
        {
            return Nest2DUtils->Nest2dRotationUtils->AngleDistance(ALeft, ARight);
        }
        bool CetLocalCompactor::_LocalCompactGetBounds(const CetNestItem &AItem, double &AOutMinX, double &AOutMinY, double &AOutMaxX, double &AOutMaxY)
        {
            CetNestItem Item = AItem;
            Item.inflation(0);
            const auto Bounds = Item.boundingBox();
            AOutMinX = static_cast<double>(getX(Bounds.minCorner()));
            AOutMinY = static_cast<double>(getY(Bounds.minCorner()));
            AOutMaxX = static_cast<double>(getX(Bounds.maxCorner()));
            AOutMaxY = static_cast<double>(getY(Bounds.maxCorner()));
            return std::isfinite(AOutMinX) && std::isfinite(AOutMinY) && std::isfinite(AOutMaxX) && std::isfinite(AOutMaxY) && AOutMaxX >= AOutMinX && AOutMaxY >= AOutMinY;
        }
        TetLocalCompactEnvelope CetLocalCompactor::CalculateEnvelope(const CetTNestItemVector &AItems, int ABinId, const std::vector<bool> *AExcluded)
        {
            TetLocalCompactEnvelope Result;
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                if (AItems[Index].binId() != ABinId || (AExcluded != nullptr && Index < AExcluded->size() && (*AExcluded)[Index]))
                    continue;
                double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
                if (!_LocalCompactGetBounds(AItems[Index], MinX, MinY, MaxX, MaxY))
                    continue;
                if (!Result.Valid) {
                    Result.Valid = true;
                    Result.MinX = MinX;
                    Result.MinY = MinY;
                    Result.MaxX = MaxX;
                    Result.MaxY = MaxY;
                } else {
                    Result.MinX = std::min(Result.MinX, MinX);
                    Result.MinY = std::min(Result.MinY, MinY);
                    Result.MaxX = std::max(Result.MaxX, MaxX);
                    Result.MaxY = std::max(Result.MaxY, MaxY);
                }
            }
            if (Result.Valid) {
                Result.Width = Result.MaxX - Result.MinX;
                Result.Height = Result.MaxY - Result.MinY;
                Result.Area = Result.Width * Result.Height;
                Result.LongSide = std::max(Result.Width, Result.Height);
            }
            return Result;
        }
        void CetLocalCompactor::_LocalCompactAppendFreeRegions(const ClipperLib::PolyNode &ANode, std::vector<TetClusterFreeRegion> &AOutRegions)
        {
            if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
                TetClusterFreeRegion Region;
                Region.Contour = ANode.Contour;
                Region.IsClosed = true;
                Region.Area = std::abs(static_cast<double>(ClipperLib::Area(Region.Contour)));
                if (Region.Area > 0.0 && std::isfinite(Region.Area)) {
                    Region.MinX = Region.MaxX = static_cast<double>(Region.Contour.front().X);
                    Region.MinY = Region.MaxY = static_cast<double>(Region.Contour.front().Y);
                    for (const ClipperLib::IntPoint &Point : Region.Contour) {
                        Region.MinX = std::min(Region.MinX, static_cast<double>(Point.X));
                        Region.MinY = std::min(Region.MinY, static_cast<double>(Point.Y));
                        Region.MaxX = std::max(Region.MaxX, static_cast<double>(Point.X));
                        Region.MaxY = std::max(Region.MaxY, static_cast<double>(Point.Y));
                    }
                    for (const ClipperLib::PolyNode *Child : ANode.Childs) {
                        if (Child != nullptr && Child->IsHole())
                            Region.Holes.push_back(Child->Contour);
                    }
                    Region.Width = Region.MaxX - Region.MinX;
                    Region.Height = Region.MaxY - Region.MinY;
                    if (Region.Width > 0.0 && Region.Height > 0.0)
                        AOutRegions.push_back(std::move(Region));
                }
            }
            for (const ClipperLib::PolyNode *Child : ANode.Childs)
                if (Child != nullptr) {
                    _LocalCompactAppendFreeRegions(*Child, AOutRegions);
                }
        }
        bool CetLocalCompactor::_BuildLocalCompactFreeRegions(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, int ABinId, const std::vector<bool> &ATargetMask, std::vector<TetClusterFreeRegion> &AOutRegions)
        {
            AOutRegions.clear();
            ClipperLib::Paths Board;
            if (!_BuildBoardSubjectContours(AOptions, Board))
                return false;
            CetTNestItemVector FrozenItems = AItems;
            for (std::size_t Index = 0; Index < FrozenItems.size() && Index < ATargetMask.size(); ++Index) {
                if (ATargetMask[Index])
                    FrozenItems[Index].binId(-1);
            }
            const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
            ClipperLib::Paths Reserved;
            if (!_BuildPlacedReservedContours(FrozenItems, ABinId, AOptions.Spacing, Reserved, HalfSpacing))
                return false;
            ClipperLib::Clipper Difference;
            if (!Difference.AddPaths(Board, ClipperLib::ptSubject, true) || (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true)))
                return false;
            ClipperLib::PolyTree Tree;
            if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero))
                return false;
            for (const ClipperLib::PolyNode *Node : Tree.Childs)
                if (Node != nullptr)
                    _LocalCompactAppendFreeRegions(*Node, AOutRegions);
            std::stable_sort(AOutRegions.begin(), AOutRegions.end(), [](const TetClusterFreeRegion &ALeft, const TetClusterFreeRegion &ARight) { return ALeft.Area != ARight.Area ? ALeft.Area > ARight.Area : ALeft.MinY < ARight.MinY; });
            if (AOutRegions.size() > CET_LOCAL_COMPACT_MAX_FREE_REGIONS)
                AOutRegions.resize(CET_LOCAL_COMPACT_MAX_FREE_REGIONS);
            return !AOutRegions.empty();
        }
        TetLocalCompactFreeSpaceMetric CetLocalCompactor::_LocalCompactCalculateFreeSpaceMetric(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, int ABinId)
        {
            TetLocalCompactFreeSpaceMetric Result;
            ClipperLib::Paths Board;
            if (!_BuildBoardSubjectContours(AOptions, Board))
                return Result;
            const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
            ClipperLib::Paths Reserved;
            if (!_BuildPlacedReservedContours(AItems, ABinId, AOptions.Spacing, Reserved, HalfSpacing))
                return Result;
            ClipperLib::Clipper Difference;
            if (!Difference.AddPaths(Board, ClipperLib::ptSubject, true) || (!Reserved.empty() && !Difference.AddPaths(Reserved, ClipperLib::ptClip, true)))
                return Result;
            ClipperLib::PolyTree Tree;
            if (!Difference.Execute(ClipperLib::ctDifference, Tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero))
                return Result;
            double TotalArea = 0.0;
            auto Accumulate = [&](const auto &Self, const ClipperLib::PolyNode &ANode) -> void {
                if (!ANode.IsHole() && ANode.Contour.size() >= 3) {
                    const double Area = std::abs(static_cast<double>(ClipperLib::Area(ANode.Contour)));
                    if (Area > 0.0 && std::isfinite(Area)) {
                        ++Result.RegionCount;
                        TotalArea += Area;
                        Result.LargestArea = std::max(Result.LargestArea, Area);
                    }
                }
                for (const ClipperLib::PolyNode *Child : ANode.Childs)
                    if (Child != nullptr)
                        Self(Self, *Child);
            };
            for (const ClipperLib::PolyNode *Node : Tree.Childs)
                if (Node != nullptr)
                    Accumulate(Accumulate, *Node);
            Result.Valid = Result.RegionCount > 0;
            Result.FragmentedArea = std::max(0.0, TotalArea - Result.LargestArea);
            return Result;
        }
        double CetLocalCompactor::_LocalCompactCalculateBoardArea(const TetNestOptions &AOptions)
        {
            ClipperLib::Paths Board;
            if (!_BuildBoardSubjectContours(AOptions, Board))
                return 0.0;
            double Area = 0.0;
            for (const CetPath &Contour : Board) {
                const double ContourArea = std::abs(static_cast<double>(ClipperLib::Area(Contour)));
                if (ContourArea <= 0.0 || !std::isfinite(ContourArea))
                    continue;
                Area += ClipperLib::Orientation(Contour) ? ContourArea : -ContourArea;
            }
            return std::max(0.0, Area);
        }
        std::map<int, std::string> CetLocalCompactor::_LocalCompactBuildSkippedBins(const CetTNestItemVector &AItems, const TetNestOptions &AOptions)
        {
            std::map<int, std::string> SkippedBins;
            std::map<int, std::size_t> ItemCounts;
            for (const CetNestItem &Item : AItems)
                if (Item.binId() >= 0)
                    ++ItemCounts[static_cast<int>(Item.binId())];
            const double BoardArea = _LocalCompactCalculateBoardArea(AOptions);
            for (const auto &Entry : ItemCounts) {
                const int BinId = Entry.first;
                const std::size_t ItemCount = Entry.second;
                if (ItemCount <= 1) {
                    SkippedBins[BinId] = "INSUFFICIENT_ITEMS";
                    continue;
                }
                const TetLocalCompactFreeSpaceMetric FreeSpace = _LocalCompactCalculateFreeSpaceMetric(AItems, AOptions, BinId);
                if (!FreeSpace.Valid) {
                    SkippedBins[BinId] = "NO_FREE_REGION";
                    continue;
                }
                const double DominantGapRatio = BoardArea > 0.0 ? FreeSpace.LargestArea / BoardArea : 0.0;
                // A single region is not by itself a reason to skip: useful compacting
                // moves often happen inside one connected free-space component. Skip
                // only when that component is genuinely dominant (nearly empty board).
                if (FreeSpace.RegionCount <= 1 && DominantGapRatio >= 0.80) {
                    SkippedBins[BinId] = "SINGLE_CONTIGUOUS_GAP";
                    continue;
                }
                if (DominantGapRatio >= 0.80) {
                    SkippedBins[BinId] = "DOMINANT_EMPTY_BOARD";
                    continue;
                }
            }
            return SkippedBins;
        }
        std::vector<std::size_t> CetLocalCompactor::_LocalCompactSelectContactVertices(const CetPath &AContour)
        {
            std::vector<std::size_t> Result;
            if (AContour.empty())
                return Result;
            std::array<std::size_t, 4> Extremes{0, 0, 0, 0};
            for (std::size_t Index = 1; Index < AContour.size(); ++Index) {
                if (AContour[Index].X < AContour[Extremes[0]].X)
                    Extremes[0] = Index;
                if (AContour[Index].X > AContour[Extremes[1]].X)
                    Extremes[1] = Index;
                if (AContour[Index].Y < AContour[Extremes[2]].Y)
                    Extremes[2] = Index;
                if (AContour[Index].Y > AContour[Extremes[3]].Y)
                    Extremes[3] = Index;
            }
            for (const std::size_t Index : Extremes) {
                if (std::find(Result.begin(), Result.end(), Index) == Result.end())
                    Result.push_back(Index);
            }
            for (std::size_t Slot = 0; Result.size() < CET_LOCAL_COMPACT_MAX_CONTACT_VERTICES && Slot < AContour.size(); ++Slot) {
                const std::size_t Index = Slot * AContour.size() / CET_LOCAL_COMPACT_MAX_CONTACT_VERTICES;
                if (std::find(Result.begin(), Result.end(), Index) == Result.end())
                    Result.push_back(Index);
            }
            return Result;
        }
        void CetLocalCompactor::_LocalCompactApplyPose(CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, double ARotation, double AAnchorX, double AAnchorY)
        {
            const double CosRotation = std::cos(ARotation);
            const double SinRotation = std::sin(ARotation);
            for (std::size_t Slot = 0; Slot < ATarget.Indices.size() && Slot < ATarget.Transforms.size(); ++Slot) {
                CetNestItem &Item = AItems[ATarget.Indices[Slot]];
                const TetItemTransform &Transform = ATarget.Transforms[Slot];
                const double X = AAnchorX + Transform.RelativeX * CosRotation - Transform.RelativeY * SinRotation;
                const double Y = AAnchorY + Transform.RelativeX * SinRotation + Transform.RelativeY * CosRotation;
                Item.translation(ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(std::llround(X)), static_cast<ClipperLib::cInt>(std::llround(Y))));
                Item.rotation(ARotation + Transform.RelativeRotation);
                Item.inflation(0);
            }
        }
        std::vector<TetLocalCompactFixedItem> CetLocalCompactor::_LocalCompactBuildFixedItemCache(const CetTNestItemVector &AItems, const std::vector<bool> &ATargetMask, const TetNestOptions &AOptions, int ABinId)
        {
            std::vector<TetLocalCompactFixedItem> Result;
            const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                if ((Index < ATargetMask.size() && ATargetMask[Index]) || AItems[Index].binId() != ABinId)
                    continue;
                CetNestItem Raw = AItems[Index];
                Raw.inflation(0);
                CetNestItem Spaced = Raw;
                if (HalfSpacing > 0)
                    Spaced.inflation(HalfSpacing);
                const auto RawBounds = Raw.boundingBox();
                const auto SpacedBounds = Spaced.boundingBox();
                Result.push_back({Raw, Spaced, static_cast<double>(getX(RawBounds.minCorner())), static_cast<double>(getY(RawBounds.minCorner())), static_cast<double>(getX(RawBounds.maxCorner())), static_cast<double>(getY(RawBounds.maxCorner())), static_cast<double>(getX(SpacedBounds.minCorner())), static_cast<double>(getY(SpacedBounds.minCorner())), static_cast<double>(getX(SpacedBounds.maxCorner())), static_cast<double>(getY(SpacedBounds.maxCorner()))});
            }
            return Result;
        }
        bool CetLocalCompactor::_LocalCompactIsTargetPoseValid(const TetLocalCompactValidationRequest &ARequest, const char *&AOutReason)
        {
            const auto &AItems = ARequest.Items;
            const auto &ATarget = ARequest.Target;
            const auto &ATargetMask = ARequest.TargetMask;
            const auto &ABinPoly = ARequest.BinPolygon;
            const auto &AOptions = ARequest.Options;
            const int ABinId = ARequest.BinId;
            const auto &AFixedItems = ARequest.FixedItems;
            AOutReason = "OUT_OF_BIN";
            const auto HalfSpacing = static_cast<libnest2d::Coord>(std::ceil(static_cast<double>(NestUtils::ToNestCoord(std::max(0.0, AOptions.Spacing))) * 0.5));
            for (const std::size_t TargetIndex : ATarget.Indices) {
                CetNestItem Target = AItems[TargetIndex];
                Target.inflation(0);
                if (Target.binId() != ABinId || !Target.isInside(ABinPoly))
                    return false;
                const auto RawBounds = Target.boundingBox();
                CetNestItem SpacedTarget = Target;
                if (HalfSpacing > 0)
                    SpacedTarget.inflation(HalfSpacing);
                const auto SpacedTargetBounds = SpacedTarget.boundingBox();
                for (const TetLocalCompactFixedItem &Other : AFixedItems) {
                    const bool RawBoundsOverlap = !(getX(RawBounds.maxCorner()) < Other.RawMinX || getX(RawBounds.minCorner()) > Other.RawMaxX || getY(RawBounds.maxCorner()) < Other.RawMinY || getY(RawBounds.minCorner()) > Other.RawMaxY);
                    if (RawBoundsOverlap && CetNestItem::intersects(Target, Other.Raw) && !CetNestItem::touches(Target, Other.Raw)) {
                        AOutReason = "COLLISION";
                        return false;
                    }
                    if (HalfSpacing > 0) {
                        if (!(getX(SpacedTargetBounds.maxCorner()) < Other.SpacedMinX || getX(SpacedTargetBounds.minCorner()) > Other.SpacedMaxX || getY(SpacedTargetBounds.maxCorner()) < Other.SpacedMinY || getY(SpacedTargetBounds.minCorner()) > Other.SpacedMaxY) && CetNestItem::intersects(SpacedTarget, Other.Spaced) && !CetNestItem::touches(SpacedTarget, Other.Spaced)) {
                            AOutReason = "SPACING";
                            return false;
                        }
                    }
                }
            }
            AOutReason = "VALID";
            return true;
        }
        TetLocalCompactEnvelope CetLocalCompactor::_LocalCompactCalculateTargetEnvelope(const CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget)
        {
            TetLocalCompactEnvelope Result;
            for (const std::size_t Index : ATarget.Indices) {
                if (Index >= AItems.size())
                    continue;
                double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
                if (!_LocalCompactGetBounds(AItems[Index], MinX, MinY, MaxX, MaxY))
                    continue;
                if (!Result.Valid) {
                    Result.Valid = true;
                    Result.MinX = MinX;
                    Result.MinY = MinY;
                    Result.MaxX = MaxX;
                    Result.MaxY = MaxY;
                } else {
                    Result.MinX = std::min(Result.MinX, MinX);
                    Result.MinY = std::min(Result.MinY, MinY);
                    Result.MaxX = std::max(Result.MaxX, MaxX);
                    Result.MaxY = std::max(Result.MaxY, MaxY);
                }
            }
            if (Result.Valid) {
                Result.Width = Result.MaxX - Result.MinX;
                Result.Height = Result.MaxY - Result.MinY;
                Result.Area = Result.Width * Result.Height;
                Result.LongSide = std::max(Result.Width, Result.Height);
            }
            return Result;
        }
        TetLocalCompactEnvelope CetLocalCompactor::_LocalCompactMergeEnvelopes(const TetLocalCompactEnvelope &AFixed, const TetLocalCompactEnvelope &ATarget)
        {
            if (!AFixed.Valid)
                return ATarget;
            if (!ATarget.Valid)
                return AFixed;
            TetLocalCompactEnvelope Result;
            Result.Valid = true;
            Result.MinX = std::min(AFixed.MinX, ATarget.MinX);
            Result.MinY = std::min(AFixed.MinY, ATarget.MinY);
            Result.MaxX = std::max(AFixed.MaxX, ATarget.MaxX);
            Result.MaxY = std::max(AFixed.MaxY, ATarget.MaxY);
            Result.Width = Result.MaxX - Result.MinX;
            Result.Height = Result.MaxY - Result.MinY;
            Result.Area = Result.Width * Result.Height;
            Result.LongSide = std::max(Result.Width, Result.Height);
            return Result;
        }
        bool CetLocalCompactor::_LocalCompactTryBuildClusterTarget(const CetTNestItemVector &AItems, const TetMetaItem &AMeta, TetLocalCompactTarget &AOutTarget)
        {
            AOutTarget = TetLocalCompactTarget{};
            if (!AMeta.IsCluster || AMeta.TransformData.size() < 2)
                return false;
            for (const TetItemTransform &Transform : AMeta.TransformData) {
                if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AItems.size()))
                    return false;
            }
            const TetItemTransform &FirstTransform = AMeta.TransformData.front();
            const CetNestItem &FirstItem = AItems[FirstTransform.OriginalId];
            if (FirstItem.binId() < 0)
                return false;
            const double Rotation = CetRotationUtils::NormalizeAngle(static_cast<double>(FirstItem.rotation()) - FirstTransform.RelativeRotation);
            const double CosRotation = std::cos(Rotation);
            const double SinRotation = std::sin(Rotation);
            const Point FirstTranslation = FirstItem.translation();
            const double AnchorX = static_cast<double>(FirstTranslation.X) - FirstTransform.RelativeX * CosRotation + FirstTransform.RelativeY * SinRotation;
            const double AnchorY = static_cast<double>(FirstTranslation.Y) - FirstTransform.RelativeX * SinRotation - FirstTransform.RelativeY * CosRotation;
            for (const TetItemTransform &Transform : AMeta.TransformData) {
                const CetNestItem &Item = AItems[Transform.OriginalId];
                if (Item.binId() != FirstItem.binId() || _LocalCompactAngleDistance(static_cast<double>(Item.rotation()), Rotation + Transform.RelativeRotation) > 1e-8)
                    return false;
                const Point Translation = Item.translation();
                const double ExpectedX = AnchorX + Transform.RelativeX * CosRotation - Transform.RelativeY * SinRotation;
                const double ExpectedY = AnchorY + Transform.RelativeX * SinRotation + Transform.RelativeY * CosRotation;
                if (std::abs(static_cast<double>(Translation.X) - ExpectedX) > 1.0 || std::abs(static_cast<double>(Translation.Y) - ExpectedY) > 1.0)
                    return false;
            }
            AOutTarget.Type = AMeta.ClusterType.empty() ? "Cluster" : AMeta.ClusterType;
            AOutTarget.IsCluster = true;
            AOutTarget.CurrentRotation = Rotation;
            AOutTarget.CurrentAnchorX = AnchorX;
            AOutTarget.CurrentAnchorY = AnchorY;
            AOutTarget.Transforms = AMeta.TransformData;
            for (const TetItemTransform &Transform : AMeta.TransformData)
                AOutTarget.Indices.push_back(static_cast<std::size_t>(Transform.OriginalId));
            return true;
        }
        bool CetLocalCompactor::_LocalCompactTryRecoverCurrentClusterTarget(const CetTNestItemVector &AItems, const TetMetaItem &AMeta, TetLocalCompactTarget &AOutTarget)
        {
            AOutTarget = TetLocalCompactTarget{};
            if (!AMeta.IsCluster || AMeta.TransformData.size() < 2)
                return false;
            const int FirstIndex = AMeta.TransformData.front().OriginalId;
            if (FirstIndex < 0 || FirstIndex >= static_cast<int>(AItems.size()) || AItems[FirstIndex].binId() < 0)
                return false;
            const int BinId = static_cast<int>(AItems[FirstIndex].binId());
            const Point Anchor = AItems[FirstIndex].translation();
            AOutTarget.Type = AMeta.ClusterType.empty() ? "RecoveredCluster" : AMeta.ClusterType;
            AOutTarget.IsCluster = true;
            AOutTarget.CurrentRotation = 0.0;
            AOutTarget.CurrentAnchorX = static_cast<double>(Anchor.X);
            AOutTarget.CurrentAnchorY = static_cast<double>(Anchor.Y);
            for (const TetItemTransform &Source : AMeta.TransformData) {
                if (Source.OriginalId < 0 || Source.OriginalId >= static_cast<int>(AItems.size()) || AItems[Source.OriginalId].binId() != BinId)
                    return false;
                const Point Translation = AItems[Source.OriginalId].translation();
                AOutTarget.Indices.push_back(static_cast<std::size_t>(Source.OriginalId));
                AOutTarget.Transforms.push_back({Source.OriginalId, static_cast<double>(Translation.X - Anchor.X), static_cast<double>(Translation.Y - Anchor.Y), static_cast<double>(AItems[Source.OriginalId].rotation())});
            }
            return true;
        }
        std::vector<TetLocalCompactTarget> CetLocalCompactor::_BuildLocalCompactTargets(const CetTNestItemVector &AItems, const std::vector<TetMetaItem> *AMetaItems)
        {
            std::vector<TetLocalCompactTarget> Targets;
            std::vector<bool> ClusterMembers(AItems.size(), false);
            if (AMetaItems != nullptr)
                for (const TetMetaItem &Meta : *AMetaItems) {
                    if (!Meta.IsCluster || Meta.TransformData.size() < 2)
                        continue;
                    bool ValidIndices = true;
                    for (const TetItemTransform &Transform : Meta.TransformData) {
                        if (Transform.OriginalId < 0 || Transform.OriginalId >= static_cast<int>(AItems.size())) {
                            ValidIndices = false;
                            break;
                        }
                    }
                    if (!ValidIndices) {
                        std::cout << "[LOCAL COMPACT][SKIP] type=" << Meta.ClusterType << " reason=INVALID_CLUSTER_INDEX" << std::endl;
                        continue;
                    }
                    TetLocalCompactTarget Target;
                    if (_LocalCompactTryBuildClusterTarget(AItems, Meta, Target) || _LocalCompactTryRecoverCurrentClusterTarget(AItems, Meta, Target)) {
                        for (const TetItemTransform &Transform : Meta.TransformData)
                            ClusterMembers[Transform.OriginalId] = true;
                        Targets.push_back(std::move(Target));
                    } else
                        std::cout << "[LOCAL COMPACT][SKIP] type=" << Meta.ClusterType << " reason=CLUSTER_RECOVERY_FAILED" << std::endl;
                }
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                if (ClusterMembers[Index] || AItems[Index].binId() < 0)
                    continue;
                TetLocalCompactTarget Target;
                Target.Indices.push_back(Index);
                Target.Transforms.push_back({static_cast<int>(Index), 0.0, 0.0, 0.0});
                Target.CurrentRotation = CetRotationUtils::NormalizeAngle(static_cast<double>(AItems[Index].rotation()));
                const Point Translation = AItems[Index].translation();
                Target.CurrentAnchorX = static_cast<double>(Translation.X);
                Target.CurrentAnchorY = static_cast<double>(Translation.Y);
                Targets.push_back(std::move(Target));
            }
            std::stable_sort(Targets.begin(), Targets.end(), [](const TetLocalCompactTarget &ALeft, const TetLocalCompactTarget &ARight) {
                if (ALeft.IsCluster != ARight.IsCluster)
                    return ALeft.IsCluster;
                return ALeft.Indices.size() > ARight.Indices.size();
            });
            return Targets;
        }
        bool CetLocalCompactor::_LocalCompactIsStrictImprovement(const TetLocalCompactEnvelope &ACandidate, const TetLocalCompactEnvelope &ABaseline)
        {
            if (!ACandidate.Valid || !ABaseline.Valid)
                return false;
            const double AreaEpsilon = std::max({1.0, std::abs(ACandidate.Area), std::abs(ABaseline.Area)}) * 1e-9;
            if (ACandidate.Area < ABaseline.Area - AreaEpsilon)
                return true;
            if (std::abs(ACandidate.Area - ABaseline.Area) > AreaEpsilon)
                return false;
            const double LongSideEpsilon = std::max({1.0, std::abs(ACandidate.LongSide), std::abs(ABaseline.LongSide)}) * 1e-9;
            return ACandidate.LongSide < ABaseline.LongSide - LongSideEpsilon;
        }
        bool CetLocalCompactor::_LocalCompactIsNonWorsening(const TetLocalCompactEnvelope &ACandidate, const TetLocalCompactEnvelope &ABaseline)
        {
            if (!ACandidate.Valid || !ABaseline.Valid)
                return false;
            const double AreaEpsilon = std::max({1.0, std::abs(ACandidate.Area), std::abs(ABaseline.Area)}) * 1e-9;
            if (ACandidate.Area > ABaseline.Area + AreaEpsilon)
                return false;
            const double LongSideEpsilon = std::max({1.0, std::abs(ACandidate.LongSide), std::abs(ABaseline.LongSide)}) * 1e-9;
            return ACandidate.LongSide <= ABaseline.LongSide + LongSideEpsilon;
        }
        bool CetLocalCompactor::_LocalCompactIsFreeSpaceBetter(const TetLocalCompactFreeSpaceMetric &ACandidate, const TetLocalCompactFreeSpaceMetric &ABaseline)
        {
            if (!ACandidate.Valid || !ABaseline.Valid)
                return false;
            if (ACandidate.RegionCount != ABaseline.RegionCount)
                return ACandidate.RegionCount < ABaseline.RegionCount;
            const double AreaEpsilon = std::max({1.0, std::abs(ACandidate.LargestArea), std::abs(ABaseline.LargestArea)}) * 1e-9;
            if (ACandidate.LargestArea > ABaseline.LargestArea + AreaEpsilon)
                return true;
            if (std::abs(ACandidate.LargestArea - ABaseline.LargestArea) > AreaEpsilon)
                return false;
            const double FragmentedEpsilon = std::max({1.0, std::abs(ACandidate.FragmentedArea), std::abs(ABaseline.FragmentedArea)}) * 1e-9;
            return ACandidate.FragmentedArea < ABaseline.FragmentedArea - FragmentedEpsilon;
        }
        bool CetLocalCompactor::_LocalCompactIsCandidateBetter(const TetLocalCompactCandidate &ACandidate, const TetLocalCompactCandidate &ABest)
        {
            const double AreaEpsilon = std::max({1.0, std::abs(ACandidate.Envelope.Area), std::abs(ABest.Envelope.Area)}) * 1e-9;
            if (std::abs(ACandidate.Envelope.Area - ABest.Envelope.Area) > AreaEpsilon)
                return ACandidate.Envelope.Area < ABest.Envelope.Area;
            const double LongSideEpsilon = std::max({1.0, std::abs(ACandidate.Envelope.LongSide), std::abs(ABest.Envelope.LongSide)}) * 1e-9;
            if (std::abs(ACandidate.Envelope.LongSide - ABest.Envelope.LongSide) > LongSideEpsilon)
                return ACandidate.Envelope.LongSide < ABest.Envelope.LongSide;
            if (ACandidate.ContactScore != ABest.ContactScore)
                return ACandidate.ContactScore > ABest.ContactScore;
            if (std::abs(ACandidate.TranslationDistance - ABest.TranslationDistance) > 1e-9)
                return ACandidate.TranslationDistance < ABest.TranslationDistance;
            return ACandidate.RotationDelta < ABest.RotationDelta - 1e-12;
        }
        void CetLocalCompactor::_ApplyLocalCompactBestCandidate(CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, const TetLocalCompactCandidate &ABest) { _LocalCompactApplyPose(AItems, ATarget, ABest.Rotation, ABest.AnchorX, ABest.AnchorY); }
        void CetLocalCompactor::_AppendLocalCompactContactAnchors(const TetLocalCompactContactAnchorRequest &ARequest)
        {
            const std::vector<std::size_t> ContactVertices = _LocalCompactSelectContactVertices(ARequest.ContactContour);
            for (const std::size_t TargetVertex : ARequest.TargetContacts)
                for (const std::size_t ContactVertex : ContactVertices) {
                    const ClipperLib::cInt X = ARequest.ContactContour[ContactVertex].X - ARequest.TargetVertices[TargetVertex].X;
                    const ClipperLib::cInt Y = ARequest.ContactContour[ContactVertex].Y - ARequest.TargetVertices[TargetVertex].Y;
                    for (ClipperLib::cInt DX = -1; DX <= 1; ++DX)
                        for (ClipperLib::cInt DY = -1; DY <= 1; ++DY) {
                            const auto Key = std::make_pair(X + DX, Y + DY);
                            auto Existing = ARequest.Anchors.find(Key);
                            if (Existing == ARequest.Anchors.end())
                                ARequest.Anchors.emplace(Key, ARequest.ContactScore);
                            else
                                Existing->second = std::max(Existing->second, ARequest.ContactScore);
                        }
                }
        }
        std::vector<const CetPath *> CetLocalCompactor::_SelectLocalCompactHoleContacts(const TetLocalCompactAnchorBuildRequest &ARequest)
        {
            std::vector<std::pair<double, const CetPath *>> Contacts;
            for (const TetClusterFreeRegion &Region : ARequest.FreeRegions)
                for (const CetPath &Hole : Region.Holes) {
                    if (Hole.empty())
                        continue;
                    double Distance = std::numeric_limits<double>::infinity();
                    for (const std::size_t Vertex : _LocalCompactSelectContactVertices(Hole)) {
                        const double DX = static_cast<double>(Hole[Vertex].X) - ARequest.Target.CurrentAnchorX;
                        const double DY = static_cast<double>(Hole[Vertex].Y) - ARequest.Target.CurrentAnchorY;
                        Distance = std::min(Distance, DX * DX + DY * DY);
                    }
                    Contacts.emplace_back(Distance, &Hole);
                }
            std::stable_sort(Contacts.begin(), Contacts.end(), [](const auto &ALeft, const auto &ARight) { return ALeft.first < ARight.first; });
            std::vector<const CetPath *> Result;
            const auto Add = [&Result](const CetPath *APath) {
                if (APath != nullptr && std::find(Result.begin(), Result.end(), APath) == Result.end())
                    Result.push_back(APath);
            };
            const auto SelectExtreme = [&](auto ACoordinate, bool AMinimum) {
                if (Contacts.empty())
                    return;
                const auto Extreme = std::min_element(Contacts.begin(), Contacts.end(), [&](const auto &ALeft, const auto &ARight) { return AMinimum ? ACoordinate(*ALeft.second) < ACoordinate(*ARight.second) : ACoordinate(*ALeft.second) > ACoordinate(*ARight.second); });
                Add(Extreme->second);
            };
            SelectExtreme([](const CetPath &APath) { return static_cast<double>(std::min_element(APath.begin(), APath.end(), [](const auto &ALeft, const auto &ARight) { return ALeft.X < ARight.X; })->X); }, true);
            SelectExtreme([](const CetPath &APath) { return static_cast<double>(std::max_element(APath.begin(), APath.end(), [](const auto &ALeft, const auto &ARight) { return ALeft.X < ARight.X; })->X); }, false);
            SelectExtreme([](const CetPath &APath) { return static_cast<double>(std::min_element(APath.begin(), APath.end(), [](const auto &ALeft, const auto &ARight) { return ALeft.Y < ARight.Y; })->Y); }, true);
            SelectExtreme([](const CetPath &APath) { return static_cast<double>(std::max_element(APath.begin(), APath.end(), [](const auto &ALeft, const auto &ARight) { return ALeft.Y < ARight.Y; })->Y); }, false);
            for (const auto &Contact : Contacts) {
                if (Result.size() >= CET_LOCAL_COMPACT_MAX_HOLE_CONTACTS)
                    break;
                Add(Contact.second);
            }
            return Result;
        }
        std::vector<TetLocalCompactAnchor> CetLocalCompactor::_BuildLocalCompactCandidateAnchors(const TetLocalCompactAnchorBuildRequest &ARequest)
        {
            CetPath TargetVertices;
            for (const std::size_t Index : ARequest.Target.Indices) {
                const CetPolygonImpl Shape = ARequest.CandidateItems[Index].transformedShape();
                TargetVertices.insert(TargetVertices.end(), Shape.Contour.begin(), Shape.Contour.end());
            }
            const std::vector<std::size_t> TargetContacts = _LocalCompactSelectContactVertices(TargetVertices);
            std::map<std::pair<ClipperLib::cInt, ClipperLib::cInt>, int> Anchors;
            const std::vector<const CetPath *> Holes = _SelectLocalCompactHoleContacts(ARequest);
            for (const TetClusterFreeRegion &Region : ARequest.FreeRegions) {
                const std::array<double, 2> Xs{Region.MinX - ARequest.TemplateEnvelope.MinX, Region.MaxX - ARequest.TemplateEnvelope.MaxX};
                const std::array<double, 2> Ys{Region.MinY - ARequest.TemplateEnvelope.MinY, Region.MaxY - ARequest.TemplateEnvelope.MaxY};
                for (const double X : Xs)
                    for (const double Y : Ys)
                        Anchors[std::make_pair(static_cast<ClipperLib::cInt>(std::llround(X)), static_cast<ClipperLib::cInt>(std::llround(Y)))] = 0;
                _AppendLocalCompactContactAnchors({TargetVertices, TargetContacts, Region.Contour, 1, Anchors});
            }
            for (const CetPath *Hole : Holes)
                _AppendLocalCompactContactAnchors({TargetVertices, TargetContacts, *Hole, 2, Anchors});
            std::vector<TetLocalCompactAnchor> Result;
            Result.reserve(Anchors.size());
            for (const auto &Anchor : Anchors)
                Result.push_back({Anchor.first.first, Anchor.first.second, Anchor.second});
            return Result;
        }
        std::vector<TetLocalCompactAnchor> CetLocalCompactor::_SelectLocalCompactCandidateAnchors(const TetLocalCompactAnchorSelectionRequest &ARequest)
        {
            std::vector<TetLocalCompactAnchor> Result = ARequest.Anchors;
            if (Result.size() > CET_LOCAL_COMPACT_MAX_ANCHORS_PER_ROTATION) {
                const std::vector<TetLocalCompactAnchor> AllAnchors = Result;
                Result.clear();
                const auto Add = [&Result](const TetLocalCompactAnchor &AAnchor) {
                    const auto Existing = std::find_if(Result.begin(), Result.end(), [&](const TetLocalCompactAnchor &ACandidate) { return ACandidate.X == AAnchor.X && ACandidate.Y == AAnchor.Y; });
                    if (Existing == Result.end())
                        Result.push_back(AAnchor);
                };
                const int MaxScore = std::max_element(AllAnchors.begin(), AllAnchors.end(), [](const TetLocalCompactAnchor &ALeft, const TetLocalCompactAnchor &ARight) { return ALeft.ContactScore < ARight.ContactScore; })->ContactScore;
                std::vector<TetLocalCompactAnchor> BestContacts;
                for (const TetLocalCompactAnchor &Anchor : AllAnchors)
                    if (Anchor.ContactScore == MaxScore)
                        BestContacts.push_back(Anchor);
                if (!BestContacts.empty()) {
                    Add(BestContacts.front());
                    Add(BestContacts.back());
                    Add(*std::min_element(BestContacts.begin(), BestContacts.end(), [](const TetLocalCompactAnchor &ALeft, const TetLocalCompactAnchor &ARight) { return ALeft.Y < ARight.Y; }));
                    Add(*std::max_element(BestContacts.begin(), BestContacts.end(), [](const TetLocalCompactAnchor &ALeft, const TetLocalCompactAnchor &ARight) { return ALeft.Y < ARight.Y; }));
                }
                const std::size_t Remaining = CET_LOCAL_COMPACT_MAX_ANCHORS_PER_ROTATION - Result.size();
                for (std::size_t Slot = 0; Slot < Remaining; ++Slot)
                    Add(AllAnchors[Remaining > 1 ? Slot * (AllAnchors.size() - 1) / (Remaining - 1) : 0]);
            }
            std::stable_sort(Result.begin(), Result.end(), [&](const TetLocalCompactAnchor &ALeft, const TetLocalCompactAnchor &ARight) {
                const auto Estimate = [&](const TetLocalCompactAnchor &AAnchor) {
                    TetLocalCompactEnvelope Placed = ARequest.TemplateEnvelope;
                    Placed.MinX += AAnchor.X;
                    Placed.MaxX += AAnchor.X;
                    Placed.MinY += AAnchor.Y;
                    Placed.MaxY += AAnchor.Y;
                    return _LocalCompactMergeEnvelopes(ARequest.FixedEnvelope, Placed);
                };
                const TetLocalCompactEnvelope Left = Estimate(ALeft), Right = Estimate(ARight);
                if (Left.Area != Right.Area)
                    return Left.Area < Right.Area;
                if (Left.LongSide != Right.LongSide)
                    return Left.LongSide < Right.LongSide;
                return ALeft.ContactScore > ARight.ContactScore;
            });
            return Result;
        }
        TetLocalCompactTargetPreparation CetLocalCompactor::_PrepareLocalCompactTarget(const CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, const TetNestOptions &AOptions)
        {
            TetLocalCompactTargetPreparation Result;
            if (ATarget.Indices.empty() || ATarget.Indices.front() >= AItems.size())
                return Result;
            Result.BinId = static_cast<int>(AItems[ATarget.Indices.front()].binId());
            if (Result.BinId < 0)
                return Result;
            Result.TargetMask.assign(AItems.size(), false);
            for (const std::size_t Index : ATarget.Indices)
                if (Index < Result.TargetMask.size())
                    Result.TargetMask[Index] = true;
            Result.OldEnvelope = CalculateEnvelope(AItems, Result.BinId);
            Result.FixedEnvelope = CalculateEnvelope(AItems, Result.BinId, &Result.TargetMask);
            const TetLocalCompactEnvelope TargetEnvelope = _LocalCompactCalculateTargetEnvelope(AItems, ATarget);
            constexpr double BoundaryTolerance = 1.0;
            Result.IsEnvelopeBoundaryTarget = Result.OldEnvelope.Valid && TargetEnvelope.Valid && (TargetEnvelope.MinX <= Result.OldEnvelope.MinX + BoundaryTolerance || TargetEnvelope.MinY <= Result.OldEnvelope.MinY + BoundaryTolerance || TargetEnvelope.MaxX >= Result.OldEnvelope.MaxX - BoundaryTolerance || TargetEnvelope.MaxY >= Result.OldEnvelope.MaxY - BoundaryTolerance);
            if (!Result.IsEnvelopeBoundaryTarget)
                return Result;
            Result.FixedItems = _LocalCompactBuildFixedItemCache(AItems, Result.TargetMask, AOptions, Result.BinId);
            Result.HasFreeRegions = _BuildLocalCompactFreeRegions(AItems, AOptions, Result.BinId, Result.TargetMask, Result.FreeRegions);
            return Result;
        }
        void CetLocalCompactor::_EvaluateLocalCompactRotation(const TetLocalCompactRotationSearchRequest &ARequest)
        {
            const auto TimeBudgetReached = [&ARequest]() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - ARequest.PassStart).count() >= CET_LOCAL_COMPACT_MAX_TIME_MS; };
            CetTNestItemVector CandidateItems = ARequest.Items;
            _LocalCompactApplyPose(CandidateItems, ARequest.Target, ARequest.Rotation, 0.0, 0.0);
            const TetLocalCompactEnvelope TemplateEnvelope = _LocalCompactCalculateTargetEnvelope(CandidateItems, ARequest.Target);
            if (!TemplateEnvelope.Valid)
                return;
            const std::vector<TetLocalCompactAnchor> AllAnchors = _BuildLocalCompactCandidateAnchors({CandidateItems, ARequest.Target, ARequest.Preparation.FreeRegions, TemplateEnvelope});
            const std::vector<TetLocalCompactAnchor> Anchors = _SelectLocalCompactCandidateAnchors({AllAnchors, TemplateEnvelope, ARequest.Preparation.FixedEnvelope});
            for (const TetLocalCompactAnchor &Anchor : Anchors) {
                if (TimeBudgetReached()) {
                    ARequest.Result.TimedOut = true;
                    return;
                }
                ++ARequest.Result.CandidateCount;
                const double AnchorX = static_cast<double>(Anchor.X), AnchorY = static_cast<double>(Anchor.Y);
                _LocalCompactApplyPose(CandidateItems, ARequest.Target, ARequest.Rotation, AnchorX, AnchorY);
                const char *Reason = "OUT_OF_BIN";
                const bool Valid = _LocalCompactIsTargetPoseValid({CandidateItems, ARequest.Target, ARequest.Preparation.TargetMask, ARequest.BinPolygon, ARequest.Options, ARequest.Preparation.BinId, ARequest.Preparation.FixedItems}, Reason);
                TetLocalCompactCandidate Candidate;
                Candidate.Valid = Valid;
                Candidate.Rotation = ARequest.Rotation;
                Candidate.AnchorX = AnchorX;
                Candidate.AnchorY = AnchorY;
                Candidate.ContactScore = Anchor.ContactScore;
                Candidate.TranslationDistance = std::hypot(AnchorX - ARequest.Target.CurrentAnchorX, AnchorY - ARequest.Target.CurrentAnchorY);
                Candidate.RotationDelta = _LocalCompactAngleDistance(ARequest.Rotation, ARequest.Target.CurrentRotation);
                if (Valid)
                    Candidate.Envelope = _LocalCompactMergeEnvelopes(ARequest.Preparation.FixedEnvelope, _LocalCompactCalculateTargetEnvelope(CandidateItems, ARequest.Target));
                const bool StrictImprovement = Valid && _LocalCompactIsStrictImprovement(Candidate.Envelope, ARequest.Preparation.OldEnvelope);
                const bool ContactImprovement = Valid && Candidate.ContactScore > 0 && _LocalCompactIsNonWorsening(Candidate.Envelope, ARequest.Preparation.OldEnvelope);
                if (Valid)
                    ++ARequest.Result.ValidCandidateCount;
                else if (std::string(Reason) == "OUT_OF_BIN")
                    ++ARequest.Result.OutOfBinCount;
                else if (std::string(Reason) == "COLLISION")
                    ++ARequest.Result.CollisionCount;
                else if (std::string(Reason) == "SPACING")
                    ++ARequest.Result.SpacingCount;
                if (Valid && _LocalCompactIsNonWorsening(Candidate.Envelope, ARequest.Preparation.OldEnvelope))
                    ARequest.Result.FreeSpaceCandidates.push_back(Candidate);
                if ((StrictImprovement || ContactImprovement) && (!ARequest.Result.HasBest || _LocalCompactIsCandidateBetter(Candidate, ARequest.Result.Best))) {
                    std::cout << "[LOCAL COMPACT][CANDIDATE] rotation=" << ARequest.Rotation << " translation=(" << AnchorX << "," << AnchorY << ") contactScore=" << Candidate.ContactScore << " newEnvelope=(" << Candidate.Envelope.Width << "," << Candidate.Envelope.Height << "," << Candidate.Envelope.Area << ") reason=" << (StrictImprovement ? "STRICT_IMPROVEMENT" : "CONTACT_IMPROVEMENT") << std::endl;
                    ARequest.Result.HasBest = true;
                    ARequest.Result.Best = Candidate;
                }
            }
        }
        void CetLocalCompactor::_ReevaluateLocalCompactFreeSpace(const TetLocalCompactFreeSpaceReviewRequest &ARequest)
        {
            if (!ARequest.Baseline.Valid || ARequest.Candidates.empty())
                return;
            std::vector<TetLocalCompactCandidate> Candidates = ARequest.Candidates;
            std::stable_sort(Candidates.begin(), Candidates.end(), [this](const TetLocalCompactCandidate &ALeft, const TetLocalCompactCandidate &ARight) { return _LocalCompactIsCandidateBetter(ALeft, ARight); });
            std::vector<std::size_t> Indices;
            const auto AddIndex = [&Indices](std::size_t AIndex) {
                if (std::find(Indices.begin(), Indices.end(), AIndex) == Indices.end())
                    Indices.push_back(AIndex);
            };
            for (std::size_t Index = 0; Index < Candidates.size() && Indices.size() < CET_LOCAL_COMPACT_MAX_FREE_SPACE_EVALUATIONS / 2; ++Index)
                AddIndex(Index);
            const auto AddExtreme = [&](auto ACoordinate, bool AMinimum) {
                const auto Extreme = std::min_element(Candidates.begin(), Candidates.end(), [&](const auto &ALeft, const auto &ARight) { return AMinimum ? ACoordinate(ALeft) < ACoordinate(ARight) : ACoordinate(ALeft) > ACoordinate(ARight); });
                AddIndex(static_cast<std::size_t>(std::distance(Candidates.begin(), Extreme)));
            };
            AddExtreme([](const TetLocalCompactCandidate &ACandidate) { return ACandidate.AnchorX; }, true);
            AddExtreme([](const TetLocalCompactCandidate &ACandidate) { return ACandidate.AnchorX; }, false);
            AddExtreme([](const TetLocalCompactCandidate &ACandidate) { return ACandidate.AnchorY; }, true);
            AddExtreme([](const TetLocalCompactCandidate &ACandidate) { return ACandidate.AnchorY; }, false);
            for (const std::size_t Index : Indices) {
                const TetLocalCompactCandidate &Candidate = Candidates[Index];
                CetTNestItemVector CandidateItems = ARequest.Items;
                _LocalCompactApplyPose(CandidateItems, ARequest.Target, Candidate.Rotation, Candidate.AnchorX, Candidate.AnchorY);
                const TetLocalCompactFreeSpaceMetric FreeSpace = _LocalCompactCalculateFreeSpaceMetric(CandidateItems, ARequest.Options, ARequest.BinId);
                if (!_LocalCompactIsFreeSpaceBetter(FreeSpace, ARequest.Baseline) || (ARequest.Result.HasBest && !_LocalCompactIsCandidateBetter(Candidate, ARequest.Result.Best)))
                    continue;
                std::cout << "[LOCAL COMPACT][RECOMPOSE] translation=(" << Candidate.AnchorX << "," << Candidate.AnchorY << ") regions=" << ARequest.Baseline.RegionCount << "->" << FreeSpace.RegionCount << " largestFree=" << ARequest.Baseline.LargestArea << "->" << FreeSpace.LargestArea << std::endl;
                ARequest.Result.HasBest = true;
                ARequest.Result.Best = Candidate;
            }
        }
        TetLocalCompactSearchResult CetLocalCompactor::_SearchLocalCompactTarget(const TetLocalCompactTargetSearchRequest &ARequest)
        {
            TetLocalCompactSearchResult Result;
            std::vector<double> Rotations = ARequest.AllowedRotations;
            if (ARequest.Target.IsCluster)
                std::stable_sort(Rotations.begin(), Rotations.end(), [&](double ALeft, double ARight) { return std::abs(_LocalCompactAngleDistance(ALeft, ARequest.Target.CurrentRotation) - CET_CLUSTER_HALF_PI) < std::abs(_LocalCompactAngleDistance(ARight, ARequest.Target.CurrentRotation) - CET_CLUSTER_HALF_PI); });
            for (const double Rotation : Rotations) {
                if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - ARequest.PassStart).count() >= CET_LOCAL_COMPACT_MAX_TIME_MS) {
                    Result.TimedOut = true;
                    break;
                }
                _EvaluateLocalCompactRotation({ARequest.Items, ARequest.Options, ARequest.BinPolygon, ARequest.Target, ARequest.Preparation, Rotation, ARequest.PassStart, Result});
                if (Result.TimedOut)
                    break;
            }
            const TetLocalCompactFreeSpaceMetric Baseline = _LocalCompactCalculateFreeSpaceMetric(ARequest.Items, ARequest.Options, ARequest.Preparation.BinId);
            _ReevaluateLocalCompactFreeSpace({ARequest.Items, ARequest.Options, ARequest.Target, ARequest.Preparation.BinId, Baseline, Result.FreeSpaceCandidates, Result});
            return Result;
        }
        void CetLocalCompactor::RunLocalCompactPass(CetTNestItemVector &AItems, const TetNestOptions &AOptions, const std::vector<TetMetaItem> *AMetaItems)
        {
            if (!AOptions.EnableLocalCompactPass) {
                std::cout << "[LOCAL COMPACT][SKIP] reason=DISABLED" << std::endl;
                return;
            }
            if (AItems.empty()) {
                std::cout << "[LOCAL COMPACT][SKIP] reason=EMPTY_ITEMS" << std::endl;
                return;
            }
            double BoardWidth = AOptions.BinWidth;
            double BoardHeight = AOptions.BinHeight;
            const CetPolygonImpl BinPoly = Nest2DUtils->Nest2DBord->BuildBinPolygonFromOptions(AOptions, BoardWidth, BoardHeight);
            if (BinPoly.Contour.size() < 3) {
                std::cout << "[LOCAL COMPACT][SKIP] reason=INVALID_BIN" << std::endl;
                return;
            }
            const CetTNestItemVector BaselineItems = AItems;
            const auto PassStart = std::chrono::steady_clock::now();
            const std::vector<TetLocalCompactTarget> Targets = _BuildLocalCompactTargets(AItems, AMetaItems);
            const std::map<int, std::string> SkippedBins = _LocalCompactBuildSkippedBins(AItems, AOptions);
            auto TimeBudgetReached = [&PassStart]() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - PassStart).count() >= CET_LOCAL_COMPACT_MAX_TIME_MS; };
            for (const auto &Entry : SkippedBins)
                std::cout << "[LOCAL COMPACT][SKIP BIN] bin=" << Entry.first << " reason=" << Entry.second << std::endl;
            const std::vector<double> AllowedRotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
            std::cout << "[LOCAL COMPACT][PASS] targets=" << Targets.size() << " rotations=" << AllowedRotations.size() << std::endl;
            std::size_t ProcessedTargets = 0;
            bool TimedOut = false;
            for (std::size_t TargetNumber = 0; TargetNumber < Targets.size(); ++TargetNumber) {
                if (TimeBudgetReached() || ProcessedTargets >= CET_LOCAL_COMPACT_MAX_TARGETS) {
                    TimedOut = TimeBudgetReached();
                    break;
                }
                const TetLocalCompactTarget &Target = Targets[TargetNumber];
                const TetLocalCompactTargetPreparation Preparation = _PrepareLocalCompactTarget(AItems, Target, AOptions);
                const int BinId = Preparation.BinId;
                if (BinId < 0)
                    continue;
                if (SkippedBins.find(BinId) != SkippedBins.end())
                    continue;
                if (!Preparation.IsEnvelopeBoundaryTarget)
                    continue;
                ++ProcessedTargets;
                const TetLocalCompactEnvelope &OldEnvelope = Preparation.OldEnvelope;
                const std::vector<TetClusterFreeRegion> &FreeRegions = Preparation.FreeRegions;
                const bool HasFreeRegions = Preparation.HasFreeRegions;
                std::cout << "[LOCAL COMPACT][BEGIN] index=" << TargetNumber << " type=" << Target.Type << " isCluster=" << Target.IsCluster << " oldRotation=" << Target.CurrentRotation << " allowedRotations=" << AllowedRotations.size() << " oldEnvelope=(" << OldEnvelope.Width << "," << OldEnvelope.Height << "," << OldEnvelope.Area << "," << OldEnvelope.LongSide << ") freeRegions=" << FreeRegions.size() << std::endl;
                if (!HasFreeRegions || !OldEnvelope.Valid) {
                    std::cout << "[LOCAL COMPACT][BEST] accepted=false reason=NO_FREE_REGION" << std::endl;
                    continue;
                }
                const TetLocalCompactSearchResult Search = _SearchLocalCompactTarget({AItems, AOptions, BinPoly, Target, Preparation, AllowedRotations, PassStart});
                TimedOut = TimedOut || Search.TimedOut;
                std::cout << "[LOCAL COMPACT][SUMMARY] candidates=" << Search.CandidateCount << " valid=" << Search.ValidCandidateCount << " outOfBin=" << Search.OutOfBinCount << " collision=" << Search.CollisionCount << " spacing=" << Search.SpacingCount << std::endl;
                if (Search.HasBest)
                    _ApplyLocalCompactBestCandidate(AItems, Target, Search.Best);
                std::cout << "[LOCAL COMPACT][BEST] accepted=" << Search.HasBest << " rotation=" << (Search.HasBest ? Search.Best.Rotation : Target.CurrentRotation) << " translation=(" << (Search.HasBest ? Search.Best.AnchorX : Target.CurrentAnchorX) << "," << (Search.HasBest ? Search.Best.AnchorY : Target.CurrentAnchorY) << ") oldArea=" << OldEnvelope.Area << " newArea=" << (Search.HasBest ? Search.Best.Envelope.Area : OldEnvelope.Area) << " oldLongSide=" << OldEnvelope.LongSide << " newLongSide=" << (Search.HasBest ? Search.Best.Envelope.LongSide : OldEnvelope.LongSide) << std::endl;
            }
            if (TimedOut)
                std::cout << "[LOCAL COMPACT][BUDGET] elapsedMs=" << CET_LOCAL_COMPACT_MAX_TIME_MS << " processedTargets=" << ProcessedTargets << std::endl;
            if (!_ValidatePlacedItemsSpacing(AItems, AOptions)) {
                AItems = BaselineItems;
                std::cout << "[LOCAL COMPACT][ROLLBACK] reason=FINAL_SPACING_VALIDATION" << std::endl;
            }
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
