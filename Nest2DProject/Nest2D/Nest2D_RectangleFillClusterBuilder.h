#pragma once

#include "EtTechCore_Object.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {

        class CetRectangleFillClusterBuilder : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetRectangleFillClusterBuilder)

            friend class CetClusterManager;

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

            bool BuildCandidateForBase(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetNestOptions& AOptions, const std::vector<bool>& AUsed, TetClusterCandidate& AOutCandidate);
            bool TryAppendFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& ACurrentCandidate, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
            bool TryAppendFillerInFreeRegions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
            bool TryAppendFillerInRectangleEnvelope(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
            bool TryAppendFillerTemplateInRectangleEnvelope(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ABaseCandidate, const TetClusterCandidate& AEnvelopeCandidate, const TetClusterCandidate& ACurrentCandidate, const TetItemTransform& ATemplateTransform, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
           protected:
            bool _TryAddFiller(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACurrentCandidate, int AFillerIndex, const TetNestOptions& AOptions, double AEnvelopeWidth, double AEnvelopeHeight, TetClusterCandidate& AOutCandidate);
            bool _TryFindFillerTransform(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACurrentCandidate, const std::vector<TetClusterFreeRegion>& AFreeRegions, int AFillerIndex, const TetNestOptions& AOptions, double AEnvelopeWidth, double AEnvelopeHeight, TetItemTransform& AOutTransform) const;
            bool _IsContourInsideFreeRegions(const CetPath& AContour, const std::vector<TetClusterFreeRegion>& AFreeRegions) const;
           // void _BuildProbePositions(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetClusterCandidate& ACandidate, int AFillerIndex, const CetPath& ARotatedFiller, double AFillerMinX, double AFillerMinY, double AFillerMaxX, double AFillerMaxY, double ARequiredGap, std::vector<std::pair<double, double>>& AOutPositions);
            bool _ContainsOriginalIndex(const TetClusterCandidate& ACandidate, int AOriginalIndex) const;
            double _GetFeatureArea(const CetNestItem& AItem, const TetShapeFeature& AFeature) const;
            double _CalculatePlacementScore(const CetTNestItemVector& AOriginalItems, const TetClusterCandidate& ACandidate, double AFillerLeft, double AFillerTop, double AFillerRight, double AFillerBottom, double ARequiredGap) const;
            void _AppendProbePosition(std::vector<std::pair<double, double>>& APositions, double AX, double AY, double AMaxX, double AMaxY) const;

            std::vector<int> _PrepareFillerIndices(const TetRectangleFillContext& ACtx, const TetClusterCandidate& ACandidate, double ABaseAvailableArea) const;

            std::uint64_t _BuildFillerShapeSignature(const TetShapeFeature& AFeature, int AIndex) const;

            // 注意：把原本 11 个参数的 _BuildProbePositions 更新为只接收 Context
            void _BuildProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const;

            void _BuildCircleProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const;

            void _BuildEllipseProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const;

            void _BuildEllipseNeighborLists(const std::vector<TetEllipseCenter>& ACenters,
                std::vector<std::vector<std::size_t>>& AOutNeighbors) const;

            void _BuildFreeRegionProbePositions(const TetProbeContext& ACtx,
                const std::vector<TetClusterFreeRegion>& AFreeRegions,
                std::vector<std::pair<double, double>>& AOutPositions) const;

            void _BuildChildContourProbePositions(const TetProbeContext& ACtx, std::vector<std::pair<double, double>>& AOutPositions) const;
        };
    }
}
