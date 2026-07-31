#include "pch.h"
#include "Nest2D_SelfFunction.h"
#include "Nest2D_PrivateDataType.h"
#include "Nest2D_TriangleClusterBuilder.h"
#include "Nest2D_CircleClusterBuilder.h"
#include "Nest2D_GapFillClusterBuilder.h"
#include "Nest2D_EllipseClusterBuilder.h"
#include "Nest2D_RectangleClusterBuilder.h"
#include "Nest2D_ArcClusterBuilder.h"
#include "Nest2D_CustomClusterBuilder.h"
#include "Nest2D_ClusterBoundary.h"
#include "Nest2D_ClusterGeometryHelper.h"
#include "Nest2D_RotationUtils.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <map>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET {
	namespace NEST2DMANAGERLIB {

		CetClusterManager::CetClusterManager() : CetCoreObject()
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
			if (AStrategy == MetClusterStrategy::None){
				for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
					_AddSingleItem(AOriginalItems, i, Result);
				}
				return Result;
			}
			
			
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
						// if (_TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)) 
						if (Nest2DUtils->Nest2dClusterTri->TryMakeRightTrianglePair(AOriginalItems, i, j, AOptions, Result)){
							Used[i] = true;
							Used[j] = true;
							Paired = true;
							std::cout << "[CLUSTER] Pair accepted: " << i << " + " << j << ", PackedCount = " << Result.NestItems.size() << std::endl;
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
			if (AStrategy == MetClusterStrategy::AutoPairCluster){
				return _BuildAutoPairClusters(AOriginalItems, AOptions);
			}
			
			for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
				_AddSingleItem(AOriginalItems, i, Result);
			}
			return Result;
		}

		TetClusterBuildResult CetClusterManager::BuildClusterItemsWithFeatures(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions, MetClusterStrategy AStrategy)
		{
			
			if (AStrategy == MetClusterStrategy::TemplateCluster){
				return _BuildTemplateClusters(AOriginalItems, AFeatures, AOptions);
			}
			



			return BuildClusterItems(AOriginalItems, AOptions, AStrategy);
		}

		void CetClusterManager::ExpandClusterResultToOriginalItems(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems, CetTNestItemVector& AOutOriginalItems)
		{
			AOutOriginalItems = AOriginalItems;
			if (APackedItems.size() != AMetaItems.size()){
				std::cout << "[CLUSTER][ERROR] PackedItems size != MetaItems size. PackedItems = " << APackedItems.size() << ", MetaItems = " << AMetaItems.size() << std::endl;
				return;
			}
			for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex){
				const auto& PackedItem = APackedItems[PackedIndex];
				const auto& Meta = AMetaItems[PackedIndex];
				auto PackedTranslation = PackedItem.translation();
				double PackedX = static_cast<double>(PackedTranslation.X);
				double PackedY = static_cast<double>(PackedTranslation.Y);
				double PackedRotation = PackedItem.rotation();
				double CosR = std::cos(PackedRotation);
				double SinR = std::sin(PackedRotation);
				std::cout << "[CLUSTER][EXPAND PACKED] PackedIndex = " << PackedIndex << ", IsCluster = " << Meta.IsCluster << ", PackedBin = " << PackedItem.binId() << ", PackedX = " << PackedX << ", PackedY = " << PackedY << ", PackedRotation = " << PackedRotation << ", Children = " << Meta.TransformData.size() << std::endl;
				_ExpandClusterChildren(PackedItem, Meta, AOutOriginalItems);
			}
			std::cout << "[CLUSTER] ExpandClusterResultToOriginalItems done. Original count = " << AOutOriginalItems.size() << std::endl;
		}

		bool CetClusterManager::ValidatePackedResultNoOverlap(const CetTNestItemVector& AOriginalItems, const CetTNestItemVector& APackedItems, const std::vector<TetMetaItem>& AMetaItems)
		{
			if (APackedItems.size() != AMetaItems.size()){
				return false;
			}

			CetTNestItemVector ExpandedItems = AOriginalItems;
			for (std::size_t PackedIndex = 0; PackedIndex < APackedItems.size(); ++PackedIndex){
				_ExpandClusterChildren(APackedItems[PackedIndex], AMetaItems[PackedIndex], ExpandedItems, false);
			}

			for (std::size_t FirstIndex = 0; FirstIndex < ExpandedItems.size(); ++FirstIndex){
				if (ExpandedItems[FirstIndex].binId() < 0){
					continue;
				}

				CetNestItem FirstItem = ExpandedItems[FirstIndex];
				FirstItem.inflation(0);
				for (std::size_t SecondIndex = FirstIndex + 1; SecondIndex < ExpandedItems.size(); ++SecondIndex){
					if (ExpandedItems[SecondIndex].binId() != FirstItem.binId()){
						continue;
					}

					CetNestItem SecondItem = ExpandedItems[SecondIndex];
					SecondItem.inflation(0);
					if (CetNestItem::intersects(FirstItem, SecondItem)){
						std::cout << "[CLUSTER][EXPANDED VALIDATION][REJECT] Overlap between original items " << FirstIndex << " and " << SecondIndex << " on bin " << FirstItem.binId() << std::endl;
						return false;
					}
				}
			}

			return true;
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

		double CetClusterManager::_GetItemWidth(const CetNestItem& AItem)
		{
			return static_cast<double>(AItem.boundingBox().width());
		}

		double CetClusterManager::_GetItemHeight(const CetNestItem& AItem)
		{
			return static_cast<double>(AItem.boundingBox().height());
		}

		void CetClusterManager::_ExpandClusterChildren(const CetNestItem& APackedItem, const TetMetaItem& AMeta, CetTNestItemVector& AOutOriginalItems, bool ALog)
		{
			auto PackedTranslation = APackedItem.translation();
			double PackedX = static_cast<double>(PackedTranslation.X);
			double PackedY = static_cast<double>(PackedTranslation.Y);
			double PackedRotation = APackedItem.rotation();
			
			double CosR = std::cos(PackedRotation);
			double SinR = std::sin(PackedRotation);
			if (ALog){
				std::cout << "[CLUSTER][EXPAND PACKED] IsCluster = " << AMeta.IsCluster << ", PackedBin = " << APackedItem.binId() << ", PackedX = " << PackedX << ", PackedY = " << PackedY << ", PackedRotation = " << PackedRotation << ", Children = " << AMeta.TransformData.size() << std::endl;
			}
			for (const auto& Transform : AMeta.TransformData){
				int originalId = Transform.OriginalId;
				if (originalId < 0 || originalId >= static_cast<int>(AOutOriginalItems.size())){
					std::cout << "[ClusTer][WARN] Invalid originalId in TransformData: " << originalId << std::endl;
					continue;
				}
				auto& OriginalItem = AOutOriginalItems[originalId];
				double LocalX = Transform.RelativeX;
				double LocalY = Transform.RelativeY;
				
				double RotatedLocalX = LocalX * CosR - LocalY * SinR;
				double RotatedLocalY = LocalX * SinR + LocalY * CosR;
				double FinalX = PackedX + RotatedLocalX;
				double FinalY = PackedY + RotatedLocalY;
				double FinalRotation = PackedRotation + Transform.RelativeRotation;
				
				OriginalItem.binId(APackedItem.binId());
				OriginalItem.translation(ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(FinalX), static_cast<ClipperLib::cInt>(FinalY)));
				OriginalItem.rotation(FinalRotation);
				if (ALog){
					std::cout << "[CLUSTER][EXPAND ITEM] OriginalId = " << originalId << ", Local = (" << LocalX << ", " << LocalY << ")" << ", Final = (" << FinalX << ", " << FinalY << ")" << ", FinalRotation = " << FinalRotation << ", Bin = " << APackedItem.binId() << std::endl;
				}
			}
		}

		TetClusterBuildResult CetClusterManager::_BuildTemplateClusters(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const TetNestOptions& AOptions)
		{
			const int Count = static_cast<int>(AOriginalItems.size());
			if (Count <= 0){
				return TetClusterBuildResult{};
			}
			if (AFeatures.size() != AOriginalItems.size()){
				std::cout << "[TEMPLATE][ERROR] Feature count mismatch. OriginalItems=" << AOriginalItems.size() << ", Features=" << AFeatures.size() << std::endl;
				return _BuildAllSingles(AOriginalItems);
			}

			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());

			std::vector<bool> Used(Count, false);
			std::map<MetShapeType, std::vector<int>> IndicesByType;
			for (int i = 0; i < Count; ++i){
				const TetShapeFeature& Feature = AFeatures[i];
				if (Feature.Width <= 0.0 || Feature.Height <= 0.0){
					continue;
				}
				IndicesByType[Feature.ShapeType].push_back(i);
			}

			std::cout << "[TEMPLATE][SHAPE COUNTS] Triangle=" << IndicesByType[MetShapeType::TriangleLike].size() << " Circle=" << IndicesByType[MetShapeType::CircleLike].size() << " Ellipse=" << IndicesByType[MetShapeType::EllipseLike].size() << " Rectangle=" << IndicesByType[MetShapeType::RectangleLike].size() << " Arc=" << IndicesByType[MetShapeType::ArcLike].size() << " Convex=" << IndicesByType[MetShapeType::ConvexPolygon].size() << " Concave=" << IndicesByType[MetShapeType::ConcavePolygon].size() << std::endl;

			std::vector<TetClusterCandidate> BaseCandidates;
			auto AppendBuilderLog = [&](const char* ABuilderName, std::size_t AOldCount) {
				std::cout << "[TEMPLATE][BUILDER] " << ABuilderName << " NewCandidates=" << BaseCandidates.size() - AOldCount << std::endl;
				};

			{
				const std::size_t OldCount = BaseCandidates.size();
				CetTriangleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, IndicesByType[MetShapeType::TriangleLike], AOptions, BaseCandidates);
				AppendBuilderLog("TriangleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = BaseCandidates.size();
				CetCircleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, IndicesByType[MetShapeType::CircleLike], AOptions, BaseCandidates);
				AppendBuilderLog("CircleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = BaseCandidates.size();
				CetEllipseClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, IndicesByType[MetShapeType::EllipseLike], AOptions, BaseCandidates);
				AppendBuilderLog("EllipseBuilder", OldCount);
			}
			{
				const std::size_t OldCount = BaseCandidates.size();
				CetRectangleClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, IndicesByType[MetShapeType::RectangleLike], AOptions, BaseCandidates);
				AppendBuilderLog("RectangleBuilder", OldCount);
			}
			{
				const std::size_t OldCount = BaseCandidates.size();
				CetArcClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, IndicesByType[MetShapeType::ArcLike], AOptions, BaseCandidates);
				AppendBuilderLog("ArcBuilder", OldCount);
			}
			{
				std::vector<int> CustomIndices;
				const auto AppendCustomIndices = [&](MetShapeType AShapeType) {
					const std::vector<int>& TypeIndices = IndicesByType[AShapeType];
					CustomIndices.insert(CustomIndices.end(), TypeIndices.begin(), TypeIndices.end());
					};
				AppendCustomIndices(MetShapeType::QuadrilateralLike);
				AppendCustomIndices(MetShapeType::ConvexPolygon);
				AppendCustomIndices(MetShapeType::ConcavePolygon);

				const std::size_t OldCount = BaseCandidates.size();
				CetCustomClusterBuilder Builder;
				Builder.BuildCandidates(AOriginalItems, AFeatures, CustomIndices, AOptions, BaseCandidates);
				AppendBuilderLog("CustomBuilder", OldCount);
			}

			std::stable_sort(BaseCandidates.begin(), BaseCandidates.end(), [](const TetClusterCandidate& A, const TetClusterCandidate& AB) {
				if (std::abs(A.Score - AB.Score) > 1e-9){
					return A.Score > AB.Score;
				}
				if (A.OriginalIndices.size() != AB.OriginalIndices.size()){
					return A.OriginalIndices.size() > AB.OriginalIndices.size();
				}
				if (std::abs(A.ProxyArea - AB.ProxyArea) > 1e-9){
					return A.ProxyArea < AB.ProxyArea;
				}
				return A.ClusterType < AB.ClusterType;
				});

			std::cout << "[TEMPLATE][BASE CANDIDATE TOTAL] " << BaseCandidates.size() << std::endl;

			std::vector<TetClusterCandidate> AcceptedCandidates;
			AcceptedCandidates.reserve(BaseCandidates.size());
			for (const TetClusterCandidate& Candidate : BaseCandidates){
				if (!_CanAcceptClusterCandidate(AOriginalItems, AOptions, Candidate, Used, Count)){
					std::cout << "[TEMPLATE][REJECT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " Score=" << Candidate.Score << std::endl;
					continue;
				}

				AcceptedCandidates.push_back(Candidate);
				for (int OriginalIndex : Candidate.OriginalIndices){
					Used[OriginalIndex] = true;
				}

				std::cout << "[TEMPLATE][BASE ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
			}

			int GapFilledClusterCount = 0;
			int GapFillerItemCount = 0;
			CetGapFillClusterBuilder GapFillBuilder;
			for (TetClusterCandidate& Candidate : AcceptedCandidates){
				TetClusterCandidate FilledCandidate;
				if (!GapFillBuilder.BuildCandidateForBase(AOriginalItems, AFeatures, Candidate, AOptions, Used, FilledCandidate)){
					continue;
				}

				int NewFillerCount = 0;
				for (int OriginalIndex : FilledCandidate.OriginalIndices){
					if (OriginalIndex < 0 || OriginalIndex >= Count){
						continue;
					}
					if (!Used[OriginalIndex]){
						Used[OriginalIndex] = true;
						++NewFillerCount;
					}
				}

				if (NewFillerCount <= 0){
					continue;
				}

				GapFillerItemCount += NewFillerCount;
				++GapFilledClusterCount;
				std::cout << "[TEMPLATE][GAPFILL ACCEPT] BaseType=" << Candidate.ClusterType << " FilledType=" << FilledCandidate.ClusterType << " Added=" << NewFillerCount << " ChildCount=" << FilledCandidate.OriginalIndices.size() << " Score=" << FilledCandidate.Score << std::endl;
				Candidate = std::move(FilledCandidate);
			}

			int AcceptedClusterCount = 0;
			for (const TetClusterCandidate& Candidate : AcceptedCandidates){
				if (!_AddClusterCandidate(Candidate, Result)){
					std::cout << "[TEMPLATE][ADD FAILED] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << std::endl;
					continue;
				}

				++AcceptedClusterCount;
				std::cout << "[TEMPLATE][ACCEPT] Builder=" << Candidate.BuilderName << " Type=" << Candidate.ClusterType << " ChildCount=" << Candidate.OriginalIndices.size() << " Score=" << Candidate.Score << std::endl;
			}

			int SingleCount = 0;
			for (int i = 0; i < Count; ++i){
				if (Used[i]){
					continue;
				}
				_AddSingleItem(AOriginalItems, i, Result);
				Used[i] = true;
				++SingleCount;
			}

			const bool CoverageValid = _ValidateBuildResultCoverage(Result, Count);
			std::cout << "[TEMPLATE][SUMMARY] OriginalCount=" << Count << " BaseCandidateCount=" << BaseCandidates.size() << " AcceptedClusterCount=" << AcceptedClusterCount << " GapFilledClusterCount=" << GapFilledClusterCount << " GapFillerItemCount=" << GapFillerItemCount << " SingleCount=" << SingleCount << " PackedItemCount=" << Result.NestItems.size() << " MetaItemCount=" << Result.MetaItems.size() << " CoverageValid=" << CoverageValid << std::endl;
			if (!CoverageValid){
				std::cout << "[TEMPLATE][FALLBACK] Coverage invalid, use all singles." << std::endl;
				return _BuildAllSingles(AOriginalItems);
			}

			return Result;
		}
		TetClusterBuildResult CetClusterManager::_BuildAutoPairClusters(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());
			const int Count = static_cast<int>(AOriginalItems.size());
			std::vector<bool> Used(Count, false);
			std::vector<TetAutoPairCandidate> AllCandidates;
			std::vector<bool> WorthAutoPair(Count, false);
			const long long TotalPairs = static_cast<long long>(Count) * static_cast<long long>(Count - 1) / 2;
			long long CheckedPairs = 0;
			
			for (int i = 0; i < Count; ++i){
				for (int j = i + 1; j < Count; ++j){
					
					if (!WorthAutoPair[i] && !WorthAutoPair[j]){
						continue;
					}
					TetAutoPairCandidate Candidate;
					if (_TryFindBestAutoPairCandidate(AOriginalItems, i, j, AOptions, Candidate)){
						if (Candidate.Valid){
							AllCandidates.push_back(std::move(Candidate));
						}
						if (CheckedPairs == 1 || CheckedPairs % 100 == 0 || CheckedPairs == TotalPairs){
							const double Percent = TotalPairs > 0 ? 100.0 * static_cast<double>(CheckedPairs) / static_cast<double>(TotalPairs) : 100.0;
							std::cout << "[AUTO_PAIR][PROGRESS] " << CheckedPairs << " / " << TotalPairs << " (" << Percent << "%)" << std::endl;
						}
					}
				}
			}
			std::cout << "[AUTO_PAIR][SEARCH DONE] CheckedPairs = " << CheckedPairs << ", CandidateCount = " << AllCandidates.size() << std::endl;
			
			std::sort(AllCandidates.begin(), AllCandidates.end(), [](const TetAutoPairCandidate& A, const TetAutoPairCandidate& AB) { return A.Score > AB.Score; });
			std::cout << "[AUTO_PAIR][GLOBAL] CandidateCount = " << AllCandidates.size() << std::endl;
			
			for (const auto& Candidate : AllCandidates){
				if (!Candidate.Valid){
					continue;
				}
				if (Used[Candidate.AIndex] || Used[Candidate.BIndex]){
					continue;
				}
				_AddAutoPairCluster(AOriginalItems, AOptions, Candidate, Result);
				Used[Candidate.AIndex] = true;
				Used[Candidate.BIndex] = true;
				std::cout << "[AUTO_PAIR][GLOBAL ACCEPT] " << Candidate.AIndex << " + " << Candidate.BIndex << ", Score = " << Candidate.Score << ", ClusterW = " << Candidate.ClusterW << ", ClusterH = " << Candidate.ClusterH << std::endl;
			}
			
			for (int i = 0; i < Count; ++i){
				if (!Used[i]){
					_AddSingleItem(AOriginalItems, i, Result);
					Used[i] = true;
				}
			}
			return Result;
		}

		bool CetClusterManager::_TryFindBestEdgePairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
		{
			if (AIndex < 0 || ABIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size()) || AIndex == ABIndex){
				return false;
			}
			CetClusterGeometryHelper Geometry;
			const ClipperLib::Path ContourA = Geometry.GetIdentityContour(AOriginalItems[AIndex]);
			const ClipperLib::Path ContourB = Geometry.GetIdentityContour(AOriginalItems[ABIndex]);
			const std::vector<TetEdgeInfo> EdgesA = _CollectEdges(ContourA);
			const std::vector<TetEdgeInfo> EdgesB = _CollectEdges(ContourB);
			if (EdgesA.empty() || EdgesB.empty()) return false;
			double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
			TetEdgePairContext ctx = { AOriginalItems, AIndex, ABIndex, AOptions, std::max(0.0, SpacingCoord) + std::max(2.0, SpacingCoord * 0.001), std::max(1.0, std::min(std::max(_GetItemWidth(AOriginalItems[AIndex]), _GetItemHeight(AOriginalItems[AIndex])), std::max(_GetItemWidth(AOriginalItems[ABIndex]), _GetItemHeight(AOriginalItems[ABIndex])))), _IsSimilarTriangleByEdges(EdgesA, EdgesB) };
			bool Found = false;
			for (const TetEdgeInfo& EdgeA : EdgesA){
				for (const TetEdgeInfo& EdgeB : EdgesB){
					if (_EvaluateEdgePair(ctx, EdgeA, EdgeB, ABestCandidate)){
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_TryFindBestAutoPairCandidate(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, TetAutoPairCandidate& ABestCandidate)
		{
			if (AIndex < 0 || ABIndex < 0 || AIndex >= static_cast<int>(AOriginalItems.size()) || ABIndex >= static_cast<int>(AOriginalItems.size())){
				return false;
			}
			if (_GetItemWidth(AOriginalItems[AIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[AIndex]) <= 0.0 || _GetItemWidth(AOriginalItems[ABIndex]) <= 0.0 || _GetItemHeight(AOriginalItems[ABIndex]) <= 0.0){
				return false;
			}
			
			if (_TryFindBestEdgePairCandidate(AOriginalItems, AIndex, ABIndex, AOptions, ABestCandidate)){
				return true;
			}
			std::vector<double> Rotations = CetRotationUtils::BuildAllowedRotations(AOptions.Rotations);
			TetAutoPairContext ctx = { AOriginalItems, AIndex, ABIndex, AOptions };
			return _RunGridSearchAllAngles(ctx, Rotations, ABestCandidate);
		}

		bool CetClusterManager::_TryBuildAutoPairAt(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairBuildInput& AInput, TetAutoPairCandidate& ACandidate)
		{
			using NestItemType = CetTNestItemVector::value_type;
			if (AInput.AIndex < 0 || AInput.BIndex < 0 || AInput.AIndex >= static_cast<int>(AOriginalItems.size()) || AInput.BIndex >= static_cast<int>(AOriginalItems.size())){
				return false;
			}
			const auto& AItem = AOriginalItems[AInput.AIndex];
			const auto& BItem = AOriginalItems[AInput.BIndex];
			CetNestItem A = AItem;
			CetNestItem B = BItem;
			A.translation(libnest2d::Point(0, 0));
			A.rotation(libnest2d::Radians(AInput.ARotation));
			A.inflation(0);
			const ClipperLib::cInt QuantizedOffsetX = static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetX));
			const ClipperLib::cInt QuantizedOffsetY = static_cast<ClipperLib::cInt>(std::llround(AInput.BOffsetY));
			B.translation(libnest2d::Point(QuantizedOffsetX, QuantizedOffsetY));
			B.rotation(libnest2d::Radians(AInput.BRotation));
			B.inflation(0);
			const double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(AOptions.Spacing));
			if (SpacingCoord > 0.0){
				const auto OldInflation = A.inflation();
				A.inflation(static_cast<decltype(OldInflation)>(SpacingCoord));
				if (NestItemType::intersects(A, B)){
					return false;
				}
				A.inflation(OldInflation);
			}
			else if (NestItemType::intersects(A, B)){
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
			if (ClusterW <= 0.0 || ClusterH <= 0.0){
				return false;
			}
			const double BinW = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinWidth));
			const double BinH = static_cast<double>(NestUtils::ToNestCoord(AOptions.BinHeight));
			const bool FitsNormally = ClusterW <= BinW && ClusterH <= BinH;
			const bool QuarterTurnAllowed = CetRotationUtils::IsAllowedRotation(CET_CLUSTER_HALF_PI, AOptions.Rotations, 1e-9);
			const bool FitsAfter90DegreeRotation = QuarterTurnAllowed && ClusterH <= BinW && ClusterW <= BinH;
			if (!FitsNormally && !FitsAfter90DegreeRotation){
				return false;
			}
			const double RotatedBBoxAreaA = std::abs((AMaxX - AMinX) * (AMaxY - AMinY));
			const double RotatedBBoxAreaB = std::abs((BMaxX - BMinX) * (BMaxY - BMinY));
			const double BeforeBBoxArea = RotatedBBoxAreaA + RotatedBBoxAreaB;
			const double AfterBBoxArea = ClusterW * ClusterH;
			if (BeforeBBoxArea <= 0.0 || AfterBBoxArea <= 0.0){
				return false;
			}
			const double SaveArea = BeforeBBoxArea - AfterBBoxArea;
			const double SaveRatio = SaveArea / BeforeBBoxArea;
			if (SaveRatio < 0.03){
				return false;
			}
			const double RealArea = std::abs(static_cast<double>(AItem.area())) + std::abs(static_cast<double>(BItem.area()));
			const double Score = _CalcAutoPairScore(BeforeBBoxArea, AfterBBoxArea, RealArea, ClusterW, ClusterH);
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

		void CetClusterManager::_AddAutoPairCluster(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate, TetClusterBuildResult& AResult)
		{
			if (!ACandidate.Valid){
				return;
			}
			auto ClusterItem = _MakeUnionNestItemFromCandidate(AOriginalItems, AOptions, ACandidate);
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
			if (ABeforeBBoxArea <= 0.0 || AAfterBBoxArea <= 0.0){
				return -1.0;
			}
			double SaveArea = ABeforeBBoxArea - AAfterBBoxArea;
			double SaveRatio = SaveArea / ABeforeBBoxArea;
			double FillRatio = 0.0;
			if (AAfterBBoxArea > 0.0){
				FillRatio = ARealArea / AAfterBBoxArea;
			}
			
			double Score = SaveRatio * 1000.0 + FillRatio * 100.0 - (AClusterW + AClusterH) * 0.000001;
			return Score;
		}

		bool CetClusterManager::_RunAutoPairGridSearch(const CetTNestItemVector& AOriginalItems, int AIndex, int ABIndex, const TetNestOptions& AOptions, const TetAutoPairGridConfig& AConfig, TetAutoPairCandidate& AOutBest)
		{
			if (AConfig.Step <= 0.0){
				return false;
			}
			const double AWidth = AConfig.RotAMaxX - AConfig.RotAMinX;
			const double AHeight = AConfig.RotAMaxY - AConfig.RotAMinY;
			const double BWidth = AConfig.RotBMaxX - AConfig.RotBMinX;
			const double BHeight = AConfig.RotBMaxY - AConfig.RotBMinY;
			const double BeforeBBoxArea = AWidth * AHeight + BWidth * BHeight;
			if (BeforeBBoxArea <= 0.0){
				return false;
			}
			bool Found = false;
			int CheckedCount = 0;
			for (double OffsetY = AConfig.MinOffsetY; OffsetY <= AConfig.MaxOffsetY; OffsetY += AConfig.Step){
				for (double OffsetX = AConfig.MinOffsetX; OffsetX <= AConfig.MaxOffsetX; OffsetX += AConfig.Step){
					++CheckedCount;
					if (CheckedCount > AConfig.MaxCheckedCount){
						return Found;
					}
					const double QuickMinX = std::min(AConfig.RotAMinX, OffsetX + AConfig.RotBMinX);
					const double QuickMinY = std::min(AConfig.RotAMinY, OffsetY + AConfig.RotBMinY);
					const double QuickMaxX = std::max(AConfig.RotAMaxX, OffsetX + AConfig.RotBMaxX);
					const double QuickMaxY = std::max(AConfig.RotAMaxY, OffsetY + AConfig.RotBMaxY);
					const double QuickW = QuickMaxX - QuickMinX;
					const double QuickH = QuickMaxY - QuickMinY;
					if (QuickW <= 0.0 || QuickH <= 0.0){
						continue;
					}
					const double QuickAfterArea = QuickW * QuickH;
					if (QuickAfterArea >= BeforeBBoxArea * 0.97){
						continue;
					}
					TetAutoPairBuildInput Input;
					Input.AIndex = AIndex;
					Input.BIndex = ABIndex;
					Input.ARotation = AConfig.ARot;
					Input.BRotation = AConfig.BRot;
					Input.BOffsetX = OffsetX;
					Input.BOffsetY = OffsetY;
					TetAutoPairCandidate Candidate;
					if (!_TryBuildAutoPairAt(AOriginalItems, AOptions, Input, Candidate)){
						continue;
					}
					if (!Found || Candidate.Score > AOutBest.Score){
						AOutBest = Candidate;
						Found = true;
					}
				}
			}
			return Found;
		}

		CetNestItem CetClusterManager::_MakeUnionNestItemFromCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetAutoPairCandidate& ACandidate)
		{
			CetClusterGeometryHelper Geometry;
			auto MakeRectangleFallback = [&Geometry, &ACandidate]() {
				CetPath Rectangle = Geometry.MakeRectangleContour(std::ceil(ACandidate.ClusterW), std::ceil(ACandidate.ClusterH));
				return Geometry.MakeNestItemFromProxyContour(Rectangle);
				};

			if (!ACandidate.Valid || ACandidate.AIndex < 0 || ACandidate.BIndex < 0 || ACandidate.AIndex >= static_cast<int>(AOriginalItems.size()) || ACandidate.BIndex >= static_cast<int>(AOriginalItems.size())){
				return MakeRectangleFallback();
			}

			std::vector<TetItemTransform> Transforms;
			Transforms.reserve(2);
			Transforms.push_back({ ACandidate.AIndex, ACandidate.RelAX, ACandidate.RelAY, ACandidate.RelARotation });
			Transforms.push_back({ ACandidate.BIndex, ACandidate.RelBX, ACandidate.RelBY, ACandidate.RelBRotation });

			CetClusterBoundary BoundaryBuilder;
			TetClusterBoundaryResult BoundaryResult;
			if (!BoundaryBuilder.BuildBoundaryWithResult(AOriginalItems, Transforms, AOptions, BoundaryResult)){
				std::cout << "[AUTO_PAIR][BOUNDARY][WARN] BuildBoundary failed, fallback to rectangle." << std::endl;
				return MakeRectangleFallback();
			}

			return Geometry.MakeNestItemFromProxyContour(BoundaryResult.Boundary);
		}
		double CetClusterManager::_CalcEdgeLength(const ClipperLib::IntPoint& A, const ClipperLib::IntPoint& AB)
		{
			const double DX = static_cast<double>(AB.X - A.X);
			const double DY = static_cast<double>(AB.Y - A.Y);
			return std::sqrt(DX * DX + DY * DY);
		}

		std::vector<TetEdgeInfo> CetClusterManager::_CollectEdges(const ClipperLib::Path& AContour)
		{
			std::vector<TetEdgeInfo> Result;
			if (AContour.size() < 3){
				return Result;
			}
			Result.reserve(AContour.size());
			for (std::size_t i = 0; i < AContour.size(); ++i){
				const auto& Start = AContour[i];
				const auto& End = AContour[(i + 1) % AContour.size()];
				const double Length = _CalcEdgeLength(Start, End);
				if (Length <= 1.0){
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
			std::sort(Result.begin(), Result.end(), [](const TetEdgeInfo& A, const TetEdgeInfo& AB) { return A.Length > AB.Length; });
			constexpr std::size_t MAX_EDGE_COUNT = 24;
			if (Result.size() > MAX_EDGE_COUNT){
				Result.resize(MAX_EDGE_COUNT);
			}
			return Result;
		}

		bool CetClusterManager::_IsSimilarTriangleByEdges(std::vector<TetEdgeInfo> AEdges, std::vector<TetEdgeInfo> ABEdges)
		{
			if (AEdges.size() != 3 || ABEdges.size() != 3){
				return false;
			}
			auto LongerFirst = [](const TetEdgeInfo& A, const TetEdgeInfo& AB) { return A.Length > AB.Length; };
			std::sort(AEdges.begin(), AEdges.end(), LongerFirst);
			std::sort(ABEdges.begin(), ABEdges.end(), LongerFirst);
			if (AEdges.front().Length <= 0.0 || ABEdges.front().Length <= 0.0){
				return false;
			}
			const double Scale = AEdges.front().Length / ABEdges.front().Length;
			constexpr double SHAPE_TOLERANCE = 0.08;
			for (std::size_t i = 0; i < 3; ++i){
				const double ScaledBLength = ABEdges[i].Length * Scale;
				const double Denominator = std::max(1.0, std::max(AEdges[i].Length, ScaledBLength));
				const double RelativeError = std::abs(AEdges[i].Length - ScaledBLength) / Denominator;
				if (RelativeError > SHAPE_TOLERANCE){
					return false;
				}
			}
			return true;
		}

		bool CetClusterManager::_SnapToAllowedRotation(double ATarget, int ARotations, double& AOutRotation)
		{
			constexpr double MAX_ANGLE_ERROR = 0.0523598775598299; // 3 degrees
			return CetRotationUtils::SnapToAllowedRotation(ATarget, ARotations, AOutRotation, MAX_ANGLE_ERROR);
		}

		bool CetClusterManager::_EvaluateEdgePair(const TetEdgePairContext& Actx, const TetEdgeInfo& AEdgeA, const TetEdgeInfo& AEdgeB, TetAutoPairCandidate& ABestCandidate)
		{
			constexpr double MIN_EDGE_MATCH_RATIO = 0.80;
			const double MaxLength = std::max(AEdgeA.Length, AEdgeB.Length);
			const double MinLength = std::min(AEdgeA.Length, AEdgeB.Length);
			if (MaxLength <= 0.0 || (MinLength / MaxLength) < MIN_EDGE_MATCH_RATIO) return false;
			const double TargetBRotation = AEdgeA.Angle + CET_CLUSTER_PI - AEdgeB.Angle;
			double BRotation = 0.0;
			if (!_SnapToAllowedRotation(TargetBRotation, Actx.Options.Rotations, BRotation)) return false;
			const double CosR = std::cos(BRotation), SinR = std::sin(BRotation);
			auto RotatePt = [&](const ClipperLib::IntPoint& APt, double& AOutX, double& AOutY) {
				AOutX = static_cast<double>(APt.X) * CosR - static_cast<double>(APt.Y) * SinR;
				AOutY = static_cast<double>(APt.X) * SinR + static_cast<double>(APt.Y) * CosR;
				};
			double RotBStartX = 0.0, RotBStartY = 0.0, RotBEndX = 0.0, RotBEndY = 0.0;
			RotatePt(AEdgeB.Start, RotBStartX, RotBStartY);
			RotatePt(AEdgeB.End, RotBEndX, RotBEndY);
			const double AMidX = (static_cast<double>(AEdgeA.Start.X) + static_cast<double>(AEdgeA.End.X)) * 0.5;
			const double AMidY = (static_cast<double>(AEdgeA.Start.Y) + static_cast<double>(AEdgeA.End.Y)) * 0.5;
			const double BMidX = (RotBStartX + RotBEndX) * 0.5;
			const double BMidY = (RotBStartY + RotBEndY) * 0.5;
			TetEdgeMatchState state;
			state.BRotation = BRotation;
			state.LengthMatchRatio = MinLength / MaxLength;
			state.MinLength = MinLength;
			state.BaseOffsets = {
				{ AMidX - BMidX, AMidY - BMidY },
				{ static_cast<double>(AEdgeA.Start.X) - RotBEndX, static_cast<double>(AEdgeA.Start.Y) - RotBEndY },
				{ static_cast<double>(AEdgeA.End.X) - RotBStartX, static_cast<double>(AEdgeA.End.Y) - RotBStartY }
			};
			return _TestEdgeOffsets(Actx, state, AEdgeA, ABestCandidate);
		}

		bool CetClusterManager::_TestEdgeOffsets(const TetEdgePairContext& Actx, const TetEdgeMatchState& Astate, const TetEdgeInfo& AEdgeA, TetAutoPairCandidate& ABestCandidate)
		{
			const double EdgeDX = static_cast<double>(AEdgeA.End.X - AEdgeA.Start.X);
			const double EdgeDY = static_cast<double>(AEdgeA.End.Y - AEdgeA.Start.Y);
			const double EdgeLength = std::sqrt(EdgeDX * EdgeDX + EdgeDY * EdgeDY);
			if (EdgeLength <= 0.0) return false;
			const double NormalX = -EdgeDY / EdgeLength, NormalY = EdgeDX / EdgeLength;
			bool Found = false;
			for (double Direction : { -1.0, 1.0 }){
				for (const auto& BaseOffset : Astate.BaseOffsets){
					TetAutoPairBuildInput Input;
					Input.AIndex = Actx.AIndex;
					Input.BIndex = Actx.BIndex;
					Input.ARotation = 0.0;
					Input.BRotation = Astate.BRotation;
					Input.BOffsetX = BaseOffset.first + NormalX * Actx.RequiredGap * Direction;
					Input.BOffsetY = BaseOffset.second + NormalY * Actx.RequiredGap * Direction;
					TetAutoPairCandidate Candidate;
					if (!_TryBuildAutoPairAt(Actx.OriginalItems, Actx.Options, Input, Candidate)) continue;
					double EdgeCoverage = std::max(0.0, std::min(1.0, Astate.MinLength / Actx.RefLength));
					Candidate.Score += Astate.LengthMatchRatio * 60.0 + EdgeCoverage * 40.0 + (Actx.SimilarTrianglePair ? 50.0 : 0.0);
					if (!Found || Candidate.Score > ABestCandidate.Score){
						ABestCandidate = Candidate;
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_RunGridSearchAllAngles(const TetAutoPairContext& Actx, const std::vector<double>& Arotations, TetAutoPairCandidate& ABestCandidate)
		{
			bool Found = false;
			for (double ARot : Arotations){
				for (double BRot : Arotations){
					if (_EvaluateRotationPair(Actx, ARot, BRot, ABestCandidate)){
						Found = true;
					}
				}
			}
			return Found;
		}

		bool CetClusterManager::_EvaluateRotationPair(const TetAutoPairContext& Actx, double ARot, double ABRot, TetAutoPairCandidate& ABestCandidate)
		{
			auto GetRotatedBBox = [&](const CetNestItem& ASrcItem, double ARotation, double& AOutMinX, double& AOutMinY, double& AOutMaxX, double& AOutMaxY, double& AOutW, double& AOutH) {
				CetNestItem Tmp = ASrcItem;
				Tmp.translation(libnest2d::Point(0, 0));
				Tmp.rotation(libnest2d::Radians(ARotation));
				Tmp.inflation(0);
				const auto BB = Tmp.boundingBox();
				AOutMinX = static_cast<double>(getX(BB.minCorner()));
				AOutMinY = static_cast<double>(getY(BB.minCorner()));
				AOutMaxX = static_cast<double>(getX(BB.maxCorner()));
				AOutMaxY = static_cast<double>(getY(BB.maxCorner()));
				AOutW = AOutMaxX - AOutMinX;
				AOutH = AOutMaxY - AOutMinY;
				};
			double RotAMinX = 0, RotAMinY = 0, RotAMaxX = 0, RotAMaxY = 0, RotWA = 0, RotHA = 0;
			double RotBMinX = 0, RotBMinY = 0, RotBMaxX = 0, RotBMaxY = 0, RotWB = 0, RotHB = 0;
			GetRotatedBBox(Actx.OriginalItems[Actx.AIndex], ARot, RotAMinX, RotAMinY, RotAMaxX, RotAMaxY, RotWA, RotHA);
			GetRotatedBBox(Actx.OriginalItems[Actx.BIndex], ABRot, RotBMinX, RotBMinY, RotBMaxX, RotBMaxY, RotWB, RotHB);
			if (RotWA <= 0.0 || RotHA <= 0.0 || RotWB <= 0.0 || RotHB <= 0.0) return false;
			double BaseSize = std::min(std::min(RotWA, RotHA), std::min(RotWB, RotHB));
			if (BaseSize <= 0.0) return false;
			double SpacingCoord = static_cast<double>(NestUtils::ToNestCoord(Actx.Options.Spacing));
			double CoarseStep = std::max(SpacingCoord > 0.0 ? SpacingCoord : 1.0, BaseSize / 4.0);
			double FineStep = std::max(SpacingCoord > 0.0 ? SpacingCoord / 2.0 : 1.0, BaseSize / 16.0);
			TetAutoPairGridConfig CoarseConfig;
			CoarseConfig.ARot = ARot;
			CoarseConfig.BRot = ABRot;
			CoarseConfig.RotWA = RotWA;
			CoarseConfig.RotHA = RotHA;
			CoarseConfig.RotWB = RotWB;
			CoarseConfig.RotHB = RotHB;
			CoarseConfig.RotAMinX = RotAMinX;
			CoarseConfig.RotAMinY = RotAMinY;
			CoarseConfig.RotAMaxX = RotAMaxX;
			CoarseConfig.RotAMaxY = RotAMaxY;
			CoarseConfig.RotBMinX = RotBMinX;
			CoarseConfig.RotBMinY = RotBMinY;
			CoarseConfig.RotBMaxX = RotBMaxX;
			CoarseConfig.RotBMaxY = RotBMaxY;
			CoarseConfig.MinOffsetX = RotAMinX - RotBMaxX - SpacingCoord;
			CoarseConfig.MaxOffsetX = RotAMaxX - RotBMinX + SpacingCoord;
			CoarseConfig.MinOffsetY = RotAMinY - RotBMaxY - SpacingCoord;
			CoarseConfig.MaxOffsetY = RotAMaxY - RotBMinY + SpacingCoord;
			CoarseConfig.Step = CoarseStep;
			CoarseConfig.MaxCheckedCount = 5000;
			TetAutoPairCandidate CoarseBest;
			if (!_RunAutoPairGridSearch(Actx.OriginalItems, Actx.AIndex, Actx.BIndex, Actx.Options, CoarseConfig, CoarseBest)) return false;
			TetAutoPairGridConfig FineConfig = CoarseConfig;
			FineConfig.MinOffsetX = CoarseBest.RawBOffsetX - CoarseStep;
			FineConfig.MaxOffsetX = CoarseBest.RawBOffsetX + CoarseStep;
			FineConfig.MinOffsetY = CoarseBest.RawBOffsetY - CoarseStep;
			FineConfig.MaxOffsetY = CoarseBest.RawBOffsetY + CoarseStep;
			FineConfig.Step = FineStep;
			FineConfig.MaxCheckedCount = 3000;
			TetAutoPairCandidate FineBest;
			bool FineFound = _RunAutoPairGridSearch(Actx.OriginalItems, Actx.AIndex, Actx.BIndex, Actx.Options, FineConfig, FineBest);
			const TetAutoPairCandidate& CurrentBest = FineFound ? FineBest : CoarseBest;
			if (CurrentBest.Score > ABestCandidate.Score){
				ABestCandidate = CurrentBest;
				return true;
			}
			return false;
		}

		bool CetClusterManager::_CanAcceptClusterCandidate(const CetTNestItemVector& AOriginalItems, const TetNestOptions& AOptions, const TetClusterCandidate& ACandidate, const std::vector<bool>& AUsed, int AOriginalCount)
		{
			(void)AOptions;
			if (AOriginalCount < 0 || AOriginalItems.size() != static_cast<std::size_t>(AOriginalCount)){
				return false;
			}
			if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.OriginalIndices.size() != ACandidate.Transforms.size() || ACandidate.ProxyContour.size() < 3){
				return false;
			}
			if (ACandidate.ClusterWidth <= 0.0 || ACandidate.ClusterHeight <= 0.0 || ACandidate.ProxyArea <= 0.0){
				return false;
			}

			std::set<int> CandidateIds;
			std::set<int> TransformIds;
			for (int OriginalIndex : ACandidate.OriginalIndices){
				if (OriginalIndex < 0 || OriginalIndex >= AOriginalCount || OriginalIndex >= static_cast<int>(AUsed.size()) || AUsed[OriginalIndex]){
					return false;
				}
				if (!CandidateIds.insert(OriginalIndex).second){
					return false;
				}
			}

			for (const TetItemTransform& Transform : ACandidate.Transforms){
				if (Transform.OriginalId < 0 || Transform.OriginalId >= AOriginalCount){
					return false;
				}
				if (!std::isfinite(Transform.RelativeX) || !std::isfinite(Transform.RelativeY) || !std::isfinite(Transform.RelativeRotation)){
					return false;
				}
				if (!TransformIds.insert(Transform.OriginalId).second){
					return false;
				}
			}

			return CandidateIds == TransformIds;
		}
		CetNestItem CetClusterManager::_MakeClusterProxyItem(const TetClusterCandidate& ACandidate)
		{
			CetClusterGeometryHelper Geometry;
			if (ACandidate.ProxyContour.size() >= 3){
				return Geometry.MakeNestItemFromProxyContour(ACandidate.ProxyContour);
			}
			return Geometry.MakeNestItemFromProxyContour(Geometry.MakeRectangleContour(ACandidate.ClusterWidth, ACandidate.ClusterHeight));
		}

		bool CetClusterManager::_AddClusterCandidate(const TetClusterCandidate& ACandidate, TetClusterBuildResult& AResult)
		{
			if (!ACandidate.Valid || ACandidate.OriginalIndices.empty() || ACandidate.Transforms.empty()){
				return false;
			}
			CetNestItem ClusterItem = _MakeClusterProxyItem(ACandidate);
			const int PackedIndex = static_cast<int>(AResult.NestItems.size());
			AResult.NestItems.push_back(std::move(ClusterItem));
			TetMetaItem Meta;
			Meta.PackedItemIndex = PackedIndex;
			Meta.IsCluster = true;
			Meta.ClusterType = ACandidate.ClusterType.empty() ? "UnknownTemplateCluster" : ACandidate.ClusterType;
			Meta.TransformData = ACandidate.Transforms;
			AResult.MetaItems.push_back(std::move(Meta));
			std::cout << "[TEMPLATE][CANDIDATE ADD] Builder=" << ACandidate.BuilderName << " Type=" << ACandidate.ClusterType << " ChildCount=" << ACandidate.OriginalIndices.size() << " Width=" << ACandidate.ClusterWidth << " Height=" << ACandidate.ClusterHeight << " FillRatio=" << ACandidate.FillRatio << " Score=" << ACandidate.Score << " PackedIndex=" << PackedIndex << std::endl;
			return true;
		}

		TetClusterBuildResult CetClusterManager::_BuildAllSingles(const CetTNestItemVector& AOriginalItems)
		{
			TetClusterBuildResult Result;
			Result.NestItems.reserve(AOriginalItems.size());
			Result.MetaItems.reserve(AOriginalItems.size());
			for (int i = 0; i < static_cast<int>(AOriginalItems.size()); ++i){
				_AddSingleItem(AOriginalItems, i, Result);
			}
			return Result;
		}

		bool CetClusterManager::_ValidateBuildResultCoverage(const TetClusterBuildResult& AResult, int AOriginalCount)
		{
			if (AOriginalCount < 0){
				std::cout << "[TEMPLATE][COVERAGE ERROR] OriginalCount < 0." << std::endl;
				return false;
			}
			if (AResult.NestItems.size() != AResult.MetaItems.size()){
				std::cout << "[TEMPLATE][COVERAGE ERROR] NestItems.size != MetaItems.size. NestItems=" << AResult.NestItems.size() << ", MetaItems=" << AResult.MetaItems.size() << std::endl;
				return false;
			}
			std::vector<int> HitCount(static_cast<std::size_t>(AOriginalCount), 0);
			for (std::size_t MetaIndex = 0; MetaIndex < AResult.MetaItems.size(); ++MetaIndex){
				const TetMetaItem& Meta = AResult.MetaItems[MetaIndex];
				if (Meta.PackedItemIndex < 0 || Meta.PackedItemIndex >= static_cast<int>(AResult.NestItems.size())){
					std::cout << "[TEMPLATE][COVERAGE ERROR] Invalid PackedItemIndex. MetaIndex=" << MetaIndex << ", PackedItemIndex=" << Meta.PackedItemIndex << ", NestItems.size=" << AResult.NestItems.size() << std::endl;
					return false;
				}
				if (Meta.TransformData.empty()){
					std::cout << "[TEMPLATE][COVERAGE ERROR] Empty TransformData. MetaIndex=" << MetaIndex << ", ClusterType=" << Meta.ClusterType << std::endl;
					return false;
				}
				for (const TetItemTransform& Transform : Meta.TransformData){
					const int OriginalId = Transform.OriginalId;
					if (OriginalId < 0 || OriginalId >= AOriginalCount){
						std::cout << "[TEMPLATE][COVERAGE ERROR] Invalid OriginalId. MetaIndex=" << MetaIndex << ", OriginalId=" << OriginalId << ", OriginalCount=" << AOriginalCount << std::endl;
						return false;
					}
					++HitCount[OriginalId];
				}
			}
			for (int OriginalId = 0; OriginalId < AOriginalCount; ++OriginalId){
				if (HitCount[OriginalId] != 1){
					std::cout << "[TEMPLATE][COVERAGE ERROR] Original item coverage invalid. OriginalId=" << OriginalId << ", HitCount=" << HitCount[OriginalId] << std::endl;
					return false;
				}
			}
			return true;
		}

	}
}
