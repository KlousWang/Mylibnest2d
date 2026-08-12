#pragma once
#include "EtTechCore_Object.h"
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>
namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetPolygonBoardRepairer :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetPolygonBoardRepairer)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("Repair", Type_Class_Func(Repair));
				_WrapFunc("SetContext", Type_Class_Func(SetContext));
				_WrapFunc("PackFromScratch", Type_Class_Func(PackFromScratch));
				//_WrapFunc("InsertTextBeforeSvgEnd", Type_Class_Func(InsertTextBeforeSvgEnd));
			}
		public:
			
			CetPolygonBoardRepairer();
			CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
			~CetPolygonBoardRepairer();
		public:
			void SetContext(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
			void Repair(std::size_t& ALayers);
			bool EvacuateLastBin(std::size_t& ALayers, TetLastBinEvacuationStats& AStats);
			
			//CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
        protected:
			void PackFromScratch(std::size_t& ALayers);
			void _BuildRotations();

			std::size_t _CompactItemBins();

			bool _TryPlaceItemInBinByGrid(std::size_t AItemIndex,int ATargetBin);

			bool _CanPlaceAt(const TetPlacementCandidate& APlacement);

			void _FillTranslationForBBoxMin(TetPlacementCandidate& APlacement,double ATargetMinX,double ATargetMinY);
			bool _IsCurrentPlacementValid(std::size_t AItemIndex);

			void _FixInvalidItems(std::size_t& ALayers);
			
			void _FillHoles(std::size_t& ALayers);
			bool _ExtractBoardFreeRegions(int ATargetBin, std::vector<TetClusterFreeRegion>& AOutRegions) const;
			bool _BuildBoardReservedContours(int ATargetBin, ClipperLib::Paths& AOutContours) const;
			bool _AppendBoardFreeRegion(const ClipperLib::PolyNode& ANode, std::vector<TetClusterFreeRegion>& AOutRegions) const;
			bool _FindBestCandidateForTargetBin(int ATargetBin, const std::vector<TetClusterFreeRegion>& AFreeRegions, TetHoleFillCandidate& ABestCandidate);
			bool _TryFindBestPlacementInBin(std::size_t AItemIndex, int ATargetBin, const std::vector<TetClusterFreeRegion>& AFreeRegions, TetHoleFillCandidate& ABestCandidate);
			bool _IsPlacementInsideFreeRegion(const TetPlacementCandidate& APlacement, const TetClusterFreeRegion& AFreeRegion) const;
			bool _ApplyHoleFillCandidate(const TetHoleFillCandidate& ACandidate);
			double _CalcHoleFillScore(std::size_t AItemIndex,int AOldBin,int ATargetBin,const libnest2d::Point& ATranslation);
			double _CalculateBinOccupiedArea(int ABinId) const;
			std::vector<std::size_t> _CollectLastBinItems(int ALastBinId) const;
			void _CaptureLastBinStats(const std::vector<std::size_t>& AItemIndices, TetLastBinEvacuationStats& AStats) const;
			int _CountUsedBins() const;
			std::vector<int> _BuildLastBinTargetOrder(int ALastBinId) const;
			bool _HasEnoughFreeAreaForLastBin(int ALastBinId, double ALastBinArea) const;
			bool _TryEvacuateItemDirect(std::size_t AItemIndex, const std::vector<int>& ATargetBins);
			std::vector<std::size_t> _CollectSmallRelocationCandidates(int ATargetBin) const;
			bool _TryEvacuateItemWithRelocation(std::size_t AItemIndex, int ALastBinId, const std::vector<int>& ATargetBins, TetLastBinEvacuationStats& AStats);
			bool _TryRelocateSmallItemWithinSameBin(std::size_t AItemIndex, int ABinId, double& AScore);
			bool _TryFindBestSmallRelocationInBin(std::size_t AItemIndex, int ATargetBin, TetPlacementCandidate& ABestPlacement, double& ABestScore);
			double _CalcSmallRelocationScore(const TetPlacementCandidate& APlacement);
			bool _ValidateAllItemsWithSpacing(int ALastBinId) const;
			bool _CanContinueSearch();
			double _GetEffectiveGridStep(long long ACheckLimit) const;
		protected:
			CetTNestItemVector* _Items = nullptr;
			const TetNestOptions* _Options = nullptr;
			const CetPolygonImpl* _BinPoly = nullptr;

			double m_BoardBinWidth = 0.0;
			double m_BoardBinHeight = 0.0;

			double m_StepMm = 3.0;
			long long m_SpacingCoord = 0;
			long long m_RemainingPlacementChecks = 0;
			long long m_PlacementChecks = 0;
			long long m_PerItemPlacementCheckLimit = 0;
			std::chrono::steady_clock::time_point m_SearchDeadline{};
			bool m_SearchBudgetReached = false;

			std::vector<libnest2d::Radians> m_Rotations;
		
		};

	}
}
