#pragma once
#include "EtTechCore_Object.h"
#include "Nest2D_DataType.h"
#include "Nest2D_PrivateDataType.h"

#include <map>
#include <string>
#include <vector>

namespace ET {
    namespace NEST2DMANAGERLIB {
        class CetLocalCompactor : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetLocalCompactor) protected : int _Init() override
            {
                CetCoreObject::_Init();
                return 0;
            }
            void _WrapFuncs() override
            {
                CetCoreObject::_WrapFuncs();
                _WrapFunc("RunLocalCompactPass", Type_Class_Func(RunLocalCompactPass));
                _WrapFunc("CalculateEnvelope", Type_Class_Func(CalculateEnvelope));
            }

        public:
            CetLocalCompactor();
            ~CetLocalCompactor();

            void RunLocalCompactPass(CetTNestItemVector &AItems, const TetNestOptions &AOptions, const std::vector<TetMetaItem> *AMetaItems);
            TetLocalCompactEnvelope CalculateEnvelope(const CetTNestItemVector &AItems, int ABinId, const std::vector<bool> *AExcluded = nullptr);

        protected:
            bool _ValidatePlacedItemsSpacing(const CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            bool _BuildBoardPath(const std::vector<TetNestPoint> &AVertices, bool AOuter, CetPath &AOutPath);
            bool _BuildBoardSubjectContours(const TetNestOptions &AOptions, ClipperLib::Paths &AOutContours);
            bool _BuildPlacedReservedContours(const CetTNestItemVector &AItems, int ABinId, double ASpacing, ClipperLib::Paths &AOutContours, libnest2d::Coord AExtraInflation);
            double _LocalCompactAngleDistance(double ALeft, double ARight);
            bool _LocalCompactGetBounds(const CetNestItem &AItem, double &AOutMinX, double &AOutMinY, double &AOutMaxX, double &AOutMaxY);
            void _LocalCompactAppendFreeRegions(const ClipperLib::PolyNode &ANode, std::vector<TetClusterFreeRegion> &AOutRegions);
            bool _BuildLocalCompactFreeRegions(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, int ABinId, const std::vector<bool> &ATargetMask, std::vector<TetClusterFreeRegion> &AOutRegions);
            TetLocalCompactFreeSpaceMetric _LocalCompactCalculateFreeSpaceMetric(const CetTNestItemVector &AItems, const TetNestOptions &AOptions, int ABinId);
            double _LocalCompactCalculateBoardArea(const TetNestOptions &AOptions);
            std::map<int, std::string> _LocalCompactBuildSkippedBins(const CetTNestItemVector &AItems, const TetNestOptions &AOptions);
            std::vector<std::size_t> _LocalCompactSelectContactVertices(const CetPath &AContour);
            void _LocalCompactApplyPose(CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, double ARotation, double AAnchorX, double AAnchorY);
            std::vector<TetLocalCompactFixedItem> _LocalCompactBuildFixedItemCache(const CetTNestItemVector &AItems, const std::vector<bool> &ATargetMask, const TetNestOptions &AOptions, int ABinId);
            bool _LocalCompactIsTargetPoseValid(const TetLocalCompactValidationRequest &ARequest, const char *&AOutReason);
            TetLocalCompactEnvelope _LocalCompactCalculateTargetEnvelope(const CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget);
            TetLocalCompactEnvelope _LocalCompactMergeEnvelopes(const TetLocalCompactEnvelope &AFixed, const TetLocalCompactEnvelope &ATarget);
            bool _LocalCompactTryBuildClusterTarget(const CetTNestItemVector &AItems, const TetMetaItem &AMeta, TetLocalCompactTarget &AOutTarget);
            bool _LocalCompactTryRecoverCurrentClusterTarget(const CetTNestItemVector &AItems, const TetMetaItem &AMeta, TetLocalCompactTarget &AOutTarget);
            std::vector<TetLocalCompactTarget> _BuildLocalCompactTargets(const CetTNestItemVector &AItems, const std::vector<TetMetaItem> *AMetaItems);
            bool _LocalCompactIsStrictImprovement(const TetLocalCompactEnvelope &ACandidate, const TetLocalCompactEnvelope &ABaseline);
            bool _LocalCompactIsNonWorsening(const TetLocalCompactEnvelope &ACandidate, const TetLocalCompactEnvelope &ABaseline);
            bool _LocalCompactIsFreeSpaceBetter(const TetLocalCompactFreeSpaceMetric &ACandidate, const TetLocalCompactFreeSpaceMetric &ABaseline);
            bool _LocalCompactIsCandidateBetter(const TetLocalCompactCandidate &ACandidate, const TetLocalCompactCandidate &ABest);
            void _ApplyLocalCompactBestCandidate(CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, const TetLocalCompactCandidate &ABest);
            void _AppendLocalCompactContactAnchors(const TetLocalCompactContactAnchorRequest &ARequest);
            std::vector<const CetPath *> _SelectLocalCompactHoleContacts(const TetLocalCompactAnchorBuildRequest &ARequest);
            std::vector<TetLocalCompactAnchor> _BuildLocalCompactCandidateAnchors(const TetLocalCompactAnchorBuildRequest &ARequest);
            std::vector<TetLocalCompactAnchor> _SelectLocalCompactCandidateAnchors(const TetLocalCompactAnchorSelectionRequest &ARequest);
            TetLocalCompactTargetPreparation _PrepareLocalCompactTarget(const CetTNestItemVector &AItems, const TetLocalCompactTarget &ATarget, const TetNestOptions &AOptions);
            void _EvaluateLocalCompactRotation(const TetLocalCompactRotationSearchRequest &ARequest);
            void _ReevaluateLocalCompactFreeSpace(const TetLocalCompactFreeSpaceReviewRequest &ARequest);
            TetLocalCompactSearchResult _SearchLocalCompactTarget(const TetLocalCompactTargetSearchRequest &ARequest);
        };
    } // namespace NEST2DMANAGERLIB
} // namespace ET
