#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"
#include <vector>
namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetRectangleFillClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetRectangleFillClusterBuilder) friend class CetClusterManager;

        protected:
            int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("BuildCandidateForBase", Type_Class_Func(BuildCandidateForBase));
                _WrapFunc("TryAppendFiller", Type_Class_Func(TryAppendFiller));
            }

        public:
            CetRectangleFillClusterBuilder();
            ~CetRectangleFillClusterBuilder();
            bool BuildCandidateForBase(const CetTNestItemVector &AOriginalItems, const std::vector<TetShapeFeature> &AFeatures, const TetClusterCandidate &ABaseCandidate, const TetNestOptions &AOptions, const std::vector<bool> &AUsed, TetClusterCandidate &AOutCandidate);
            bool TryAppendFiller(const TetRectangleFillRequest &ARequest, TetClusterCandidate &AOutCandidate);
            bool TryAppendFillerInFreeRegions(const TetRectangleFillRequest &ARequest, TetClusterCandidate &AOutCandidate);
            bool TryAppendFillerInRectangleEnvelope(const TetRectangleFillRequest &ARequest, TetClusterCandidate &AOutCandidate);
            bool TryAppendFillerTemplateInRectangleEnvelope(const TetRectangleFillRequest &ARequest, const TetItemTransform &ATemplateTransform, TetClusterCandidate &AOutCandidate);
            void BuildFillerVariantsInFreeRegions(const TetRectangleFillRequest &ARequest, std::size_t AMaxCount, std::vector<TetClusterCandidate> &AOutCandidates);
        protected:
            void _CollectFillerTransforms(const TetRectangleFillRequest &ARequest, std::size_t AMaxCount, std::vector<TetScoredItemTransform> &AOutTransforms) const;
            void _CollectRotationPlacements(const TetRectangleFillRequest &ARequest, double ARotation, std::vector<TetScoredItemTransform> &AOutTransforms) const;
            bool _TryAddFiller(const TetRectangleFillRequest &ARequest, TetClusterCandidate &AOutCandidate);
            bool _TryFindFillerTransform(const TetRectangleFillRequest &ARequest, TetItemTransform &AOutTransform) const;
            bool _IsContourInsideFreeRegions(const CetPath &AContour, const std::vector<TetClusterFreeRegion> &AFreeRegions) const;
            // void _BuildProbePositions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate, int AFillerIndex, const CetPath& ARotatedFiller, double AFillerMinX, double AFillerMinY, double AFillerMaxX, double AFillerMaxY, double ARequiredGap, std::vector<std::pair<double, double>>& AOutPositions);
            bool _ContainsOriginalIndex(const TetClusterCandidate &ACandidate, int AOriginalIndex) const;
            double _GetFeatureArea(const CetNestItem &AItem, const TetShapeFeature &AFeature) const;
            double _CalculatePlacementScore(const TetPlacementScoreRequest &ARequest) const;
            double _CalculateFreeRegionBoundaryContactScore(const CetPath &AFillerContour, const std::vector<TetClusterFreeRegion> &AFreeRegions, double ARequiredGap) const;
            void _AppendProbePosition(std::vector<std::pair<double, double>> &APositions, double AX, double AY, double AMaxX, double AMaxY) const;
            std::vector<int> _PrepareFillerIndices(const TetRectangleFillContext &ACtx, const TetClusterCandidate &ACandidate, double ABaseAvailableArea) const;
            std::uint64_t _BuildFillerShapeSignature(const TetShapeFeature &AFeature, int AIndex) const;
            // 注意：把原本 11 个参数的 _BuildProbePositions 更新为只接收 Context
            void _BuildProbePositions(const TetProbeContext &ACtx, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _BuildCircleProbePositions(const TetProbeContext &ACtx, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _AppendFourCircleCenterProbes(const TetProbeContext &ACtx, const std::vector<TetCircleCenter> &ACircleCenters, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _BuildEllipseProbePositions(const TetProbeContext &ACtx, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _BuildEllipseNeighborLists(const std::vector<TetEllipseCenter> &ACenters, std::vector<std::vector<std::size_t>> &AOutNeighbors) const;
            void _BuildFreeRegionProbePositions(const TetProbeContext &ACtx, const std::vector<TetClusterFreeRegion> &AFreeRegions, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _AppendFreeRegionContourProbes(const TetProbeContext &ACtx, const CetPath &AContour, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _AppendFreeRegionCenterProbes(const TetProbeContext &ACtx, const std::vector<TetClusterFreeRegion> &AFreeRegions, std::vector<std::pair<double, double>> &AOutPositions) const;
            void _PrioritizeFreeRegionProbes(const TetProbeContext &ACtx, const std::vector<TetClusterFreeRegion> &AFreeRegions, bool AHasDenseEllipseSkeleton, std::vector<std::pair<double, double>> &AInOutPositions) const;
            void _BuildDenseCircleFreeRegionProbePositions(const TetProbeContext &ACtx, const std::vector<TetClusterFreeRegion> &AFreeRegions, std::vector<std::pair<double, double>> &AOutPositions) const;
            bool _TryFindRotationPlacement(const TetRectangleFillRequest &ARequest, double ARotation, TetItemTransform &AOutTransform, double &AOutScore) const;
            bool _TryFindBoundaryRotationPlacement(const TetRectangleFillRequest &ARequest, double ARotation, TetItemTransform &AOutTransform, double &AOutScore) const;
            void _BuildChildContourProbePositions(const TetProbeContext &ACtx, std::vector<std::pair<double, double>> &AOutPositions) const;
        };
    } // namespace NEST2DMANAGERLIB
} // namespace ET
