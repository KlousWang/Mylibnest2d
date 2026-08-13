#pragma once
#include "Nest2D_DataType.h" 
#include "Nest2D_PrivateDataType.h"
#include "EtTechCore_Object.h"

namespace ET {
    namespace NEST2DMANAGERLIB {
       
        class CetNest2DEngine : public ET::CORE::CetCoreObject
        {
            Inherit_Invoke_Hook(CetNest2DEngine)

        protected:
            int _Init() override { 
                CetCoreObject::_Init(); 
                return 0; 
            }
            void _WrapFuncs() override {
                CetCoreObject::_WrapFuncs(); 
                _WrapFunc("RunNesting", Type_Class_Func(RunNesting_Impl));
            }
        public:
            CetNest2DEngine();
            ~CetNest2DEngine();

            int RunNesting_Impl(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t* AUsedBins);
        protected:
            std::size_t RunPolygonBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker);
			std::size_t RunPolygonNestOnce(CetTNestItemVector& ATestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker);
            std::size_t RunRectangleBoardNesting(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker);
            bool _RunLastBinEvacuation(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, std::size_t& ALayers);
            bool _RepairAndEvacuate(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, const CetPolygonImpl& ABinPoly, double ABinWidth, double ABinHeight, std::size_t& ALayers);
            bool _TryBoardFeedbackNest(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, std::size_t& ALayers);
            std::size_t RunRectangleNestOnce(CetTNestItemVector& ATestItems,const TetNestOptions& AOptions,TetNestProgressTracker& ATracker);
            TetLocalBestResult EvaluateSortingStrategies(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, TetExpandedSpacingFailure* AOutSpacingFailure = nullptr);
            TetLocalBestResult _EvaluateSingleSortingStrategy(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, MetENestOrderStrategy AStrategy, TetExpandedSpacingFailure* AOutSpacingFailure = nullptr);
            TetLocalBestResult _TryLocalClusterSpacingFallback(const TetClusterBuildResult& AClusterResult, const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, TetNestProgressTracker& ATracker, const TetExpandedSpacingFailure& AInitialFailure);
			bool _HasClusterItems(const std::vector<TetMetaItem>& AMetaItems) const;
			std::vector<std::size_t> _BuildPriorityOrder(CetTNestItemVector& AItems, const TetNestOptions& AOptions, MetENestOrderStrategy AStrategy) const;
			void _BuildSortedTestData(CetTNestItemVector& APriorityItems, const std::vector<TetMetaItem>& AMetaItems, const std::vector<std::size_t>& ASortedIndices, CetTNestItemVector& AOutItems, std::vector<TetMetaItem>& AOutMetaItems) const;
			void _UpdateLocalBest(TetLocalBestResult& ALocalBest, TetTNestEvalResult AEvaluation, std::size_t ALayers, CetTNestItemVector& AItems, std::vector<TetMetaItem>& AMetaItems, bool AHasCluster) const;
			bool ShoouldUpdateGlobalBest(const TetLocalBestResult& ALocalResult,bool AHasBest, const TetTNestEvalResult& ABestEval, std::size_t ABestLayers, bool ABestHasCluster);
            //std::size_t RunRectangleBoardNestingFill(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, TetNestProgressTracker& Tracker);
        };
    }
}
