#pragma once
#include "EtTechCore_Object.h"
#include "NestUtils.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_DataType.h"

#include <vector>
#include <string>
#include <map>

namespace ET {
	namespace NEST2DMANAGERLIB {
		class CetClusterManager :public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetClusterManager)

		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("BuildClusterItems", Type_Class_Func(BuildClusterItems));
				_WrapFunc("ExpandClusterResultToOriginalItems", Type_Class_Func(ExpandClusterResultToOriginalItems));

			}

		public:
			CetClusterManager();
			~CetClusterManager();

		public:
			TetClusterBuildResult BuildClusterItems(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, MetClusterStrategy AStrategy);
			void ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems);

		protected:
			void _AddSingleItem(const CetTNestItemVector& AOriginalItems, int AOriginalIndex, TetClusterBuildResult& AResult);
			bool _TryMakeRightTrianglePair(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult);
			bool _NearlyEqual(double A, double B, double RelTol);
			double _GetItemWidth(const CetNestItem& AItem);
			double _GetItemHeight(const CetNestItem& AItem);
			bool _IsRightTriangleLike(const CetNestItem& AItem);
			bool _IsSameSizeTrianglePair(const CetNestItem& AItem, const CetNestItem& BItem);
			CetNestItem _MakeRectangleNestItemByNestCoord(double AW, double AH);
			double _CalcTrianglePairAxisGap(double AW, double AH, double ASpacing);
			//void ExpandOnePackedItemToOriginalItems(std::size_t APackedIndex,const CetNestItem& APackedItem,const TetMetaItem& AMeta,CetTNestItemVector& AOutOriginalItems);
			void _ExpandClusterChildren(const CetNestItem& PackedItem, const TetMetaItem& Meta, CetTNestItemVector& AOutOriginalItems);

			//
			TetClusterBuildResult _BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions);
			CetPath _GetItemIdentityContour(const CetNestItem& AItem);
			bool _TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
			bool _TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate);
			bool _TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate);
			void _AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult);
			double _CalcAutoPairScore(double ABeforeBBoxArea, double AAfterBBoxArea, double ARealArea, double AClusterW, double AClusterH);
			bool _RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int BIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& OutBest);
			CetNestItem _MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetAutoPairCandidate& ACandidate);
			void _AddTransformedItemPathToSubject(const CetNestItem& AItem, double AOffsetX, double AOffsetY, double ARotation, ClipperLib::Paths& ASubject);
			double _CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& B);
			std::vector<TetEdgeInfo> _CollectEdges(const ClipperLib::Path& AContour);
			bool _IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> BEdges);
		};
	}
}
