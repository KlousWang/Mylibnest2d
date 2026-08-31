#include "pch.h"
#include "Nest2D_QuarterTurnOptimizer.h"
#include "Nest2D_LocalCompactor.h"
#include "Nest2D_SelfFunction.h"
#include <algorithm>
#include <cmath>
namespace ET {
    namespace NEST2DMANAGERLIB {
        CetQuarterTurnOptimizer::CetQuarterTurnOptimizer() : CetCoreObject() {}
        CetQuarterTurnOptimizer::~CetQuarterTurnOptimizer() {}
        std::vector<TetQuarterTurnTarget> CetQuarterTurnOptimizer::CollectQuarterTurnTargets(const CetTNestItemVector &AItems, const std::vector<TetMetaItem> &AMetaItems, libnest2d::Coord ASpacing)
        {
            std::vector<TetQuarterTurnTarget> Targets;
            for (std::size_t Index = 0; Index < AItems.size() && Index < AMetaItems.size(); ++Index) {
                if (!AMetaItems[Index].IsCluster || AMetaItems[Index].TransformData.size() < 2)
                    continue;
                const auto Bounds = AItems[Index].boundingBox();
                const double Width = std::abs(static_cast<double>(Bounds.width()));
                const double Height = std::abs(static_cast<double>(Bounds.height()));
                if (Width <= 0.0 || Height <= 0.0)
                    continue;
                const double Aspect = std::max(Width, Height) / std::min(Width, Height);
                if (Aspect <= 1.1)
                    continue;
                const TetLocalCompactEnvelope Envelope = Nest2DUtils->Nest2dLocalCompactor->CalculateEnvelope(AItems, AItems[Index].binId());
                const double Tolerance = std::max(1.0, static_cast<double>(ASpacing));
                const bool OnEnvelope = Envelope.Valid && (std::abs(static_cast<double>(getX(Bounds.minCorner())) - Envelope.MinX) <= Tolerance || std::abs(static_cast<double>(getX(Bounds.maxCorner())) - Envelope.MaxX) <= Tolerance || std::abs(static_cast<double>(getY(Bounds.minCorner())) - Envelope.MinY) <= Tolerance || std::abs(static_cast<double>(getY(Bounds.maxCorner())) - Envelope.MaxY) <= Tolerance);
                if (OnEnvelope)
                    Targets.push_back({Aspect, Index});
            }
            std::stable_sort(Targets.begin(), Targets.end(), [](const TetQuarterTurnTarget &ALeft, const TetQuarterTurnTarget &ARight) { return ALeft.Score > ARight.Score; });
            if (Targets.size() > 8)
                Targets.resize(8);
            return Targets;
        }
        namespace {
            void AddQuarterTurnCoordinate(std::vector<ClipperLib::cInt> &ACoordinates, double AValue, double AMaximum)
            {
                if (AValue >= -1.0 && AValue <= AMaximum + 1.0)
                    ACoordinates.push_back(static_cast<ClipperLib::cInt>(std::llround(std::clamp(AValue, 0.0, AMaximum))));
            }
        } // namespace
        TetQuarterTurnCoordinates CetQuarterTurnOptimizer::BuildQuarterTurnCoordinates(const TetQuarterTurnCoordinateRequest &ARequest)
        {
            TetQuarterTurnCoordinates Result;
            const double MaxX = static_cast<double>(ARequest.BinWidth) - ARequest.RotatedWidth;
            const double MaxY = static_cast<double>(ARequest.BinHeight) - ARequest.RotatedHeight;
            AddQuarterTurnCoordinate(Result.X, 0.0, MaxX);
            AddQuarterTurnCoordinate(Result.X, MaxX, MaxX);
            AddQuarterTurnCoordinate(Result.Y, 0.0, MaxY);
            AddQuarterTurnCoordinate(Result.Y, MaxY, MaxY);
            for (std::size_t Index = 0; Index < ARequest.Items.size(); ++Index) {
                if (Index == ARequest.TargetIndex || ARequest.Items[Index].binId() != ARequest.BinId)
                    continue;
                CetNestItem Other = ARequest.Items[Index];
                Other.inflation(0);
                const auto Bounds = Other.boundingBox();
                AddQuarterTurnCoordinate(Result.X, static_cast<double>(getX(Bounds.maxCorner())) + ARequest.Spacing, MaxX);
                AddQuarterTurnCoordinate(Result.X, static_cast<double>(getX(Bounds.minCorner())) - ARequest.Spacing - ARequest.RotatedWidth, MaxX);
                AddQuarterTurnCoordinate(Result.Y, static_cast<double>(getY(Bounds.maxCorner())) + ARequest.Spacing, MaxY);
                AddQuarterTurnCoordinate(Result.Y, static_cast<double>(getY(Bounds.minCorner())) - ARequest.Spacing - ARequest.RotatedHeight, MaxY);
            }
            std::sort(Result.X.begin(), Result.X.end());
            Result.X.erase(std::unique(Result.X.begin(), Result.X.end()), Result.X.end());
            std::sort(Result.Y.begin(), Result.Y.end());
            Result.Y.erase(std::unique(Result.Y.begin(), Result.Y.end()), Result.Y.end());
            return Result;
        }
        bool CetQuarterTurnOptimizer::CanPlaceQuarterTurnTarget(const CetTNestItemVector &AItems, std::size_t ATargetIndex, libnest2d::Coord ABinWidth, libnest2d::Coord ABinHeight, libnest2d::Coord AHalfSpacing)
        {
            if (ATargetIndex >= AItems.size())
                return false;
            CetNestItem Target = AItems[ATargetIndex];
            Target.inflation(0);
            const auto RawBounds = Target.boundingBox();
            if (getX(RawBounds.minCorner()) < 0 || getY(RawBounds.minCorner()) < 0 || getX(RawBounds.maxCorner()) > ABinWidth || getY(RawBounds.maxCorner()) > ABinHeight)
                return false;
            Target.inflation(AHalfSpacing);
            const auto TargetBounds = Target.boundingBox();
            for (std::size_t Index = 0; Index < AItems.size(); ++Index) {
                if (Index == ATargetIndex || AItems[Index].binId() != Target.binId())
                    continue;
                CetNestItem Other = AItems[Index];
                Other.inflation(AHalfSpacing);
                const auto OtherBounds = Other.boundingBox();
                if (getX(TargetBounds.maxCorner()) < getX(OtherBounds.minCorner()) || getX(TargetBounds.minCorner()) > getX(OtherBounds.maxCorner()) || getY(TargetBounds.maxCorner()) < getY(OtherBounds.minCorner()) || getY(TargetBounds.minCorner()) > getY(OtherBounds.maxCorner()))
                    continue;
                if (CetNestItem::intersects(Target, Other) && !CetNestItem::touches(Target, Other))
                    return false;
            }
            return true;
        }
        void CetQuarterTurnOptimizer::ApplyQuarterTurnPassBest(TetLocalBestResult &AInOutBest, TetTNestEvalResult &&AEval, std::size_t ALayers, CetTNestItemVector &&AItems, std::vector<TetMetaItem> &&AMetaItems, bool AHasCluster)
        {
            AInOutBest.HasBest = true;
            AInOutBest.Eval = std::move(AEval);
            AInOutBest.Layers = ALayers;
            AInOutBest.Items = std::move(AItems);
            AInOutBest.MetaItems = std::move(AMetaItems);
            AInOutBest.HasCluster = AHasCluster;
        }
    } // namespace NEST2DMANAGERLIB
} // namespace ET
