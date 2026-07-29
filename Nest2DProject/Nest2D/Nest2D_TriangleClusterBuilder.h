#pragma once
#include"EtTechCore_Object.h"
#include"Nest2D_PrivateDataType.h"
#include"Nest2D_DataType.h"
#include<vector>
#include<set>
namespace ET {
	namespace NEST2DMANAGERLIB
	{
		class CetTriangleClusterBuilder:public ET::CORE::CetCoreObject
		{
			Inherit_Invoke_Hook(CetTriangleClusterBuilder)
		protected:
			int _Init() override {
				CetCoreObject::_Init();
				return 0;
			}
			void _WrapFuncs() override {
				CetCoreObject::_WrapFuncs();
				_WrapFunc("TryMakeRightTrianglePair",Type_Class_Func(TryMakeRightTrianglePair));
				_WrapFunc("BuildCandidates",Type_Class_Func(BuildCandidates));
			}
		public:
			CetTriangleClusterBuilder();
			~CetTriangleClusterBuilder();
		public:
			bool TryMakeRightTrianglePair(const CetTNestItemVector& AOriginaIItem, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterBuildResult& AResult);
			void BuildCandidates(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, const std::vector<int>& AIndices, const TetNestOptions& AOptions, std::vector<TetClusterCandidate>& AOutCandidates);
		protected:
	     	bool _IsRightTriangleLike(const CetNestItem& AItem);
			bool _IsSameSizeTrianglePair(const CetNestItem& AItem,const CetNestItem& ABItem);
			bool _NearlyEqual(double A,double AB,double ARelativeTolerance);
			double _GetItemWidth(const CetNestItem& AItem);
			double _GetItemHeight(const CetNestItem& AItem);
			double _CalcTrianglePairAxisGap(double AWidth,double AHeight,double ASpacing);
			CetNestItem _MakeRectangleNestItem(double AWidth,double AHeight);

			bool _BuildRightTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
			void _BuildRightTriangleRectangleClusterCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates,std::set<int>& AOutHandledIndices);
			bool _BuildRightTriangleRectangleClusterCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
			bool _BuildRightTriangleRectangleLayoutCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,int ACellRows,int ACellCols,double ACellWidth,double ACellHeight,double AAxisGap,double ACellGap,double AHalfTurn,TetClusterCandidate& AOutCandidate);
			double _CalculateRightTriangleRectangleScore(const TetClusterCandidate& ACandidate,int APairCount,int ACellRows,int ACellCols);
			bool _AreCongruentTriangles(const TetShapeFeature& AA, const TetShapeFeature& AB);
			void _BuildAnyTriangleClusterCandidates(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,std::vector<TetClusterCandidate>& AOutCandidates);
			bool _BuildAnyTriangleClusterCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,const std::vector<int>& AIndices,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
			
			bool _BuildAnyTrianglePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
			bool _TryBuildTriangleEdgePairCandidate(const CetTNestItemVector& AOriginalItems, const std::vector<TetShapeFeature>& AFeatures, int AAIndex, int ABIndex, int AEdgeIndex, int ABEdgeIndex, const TetNestOptions& AOptions, TetClusterCandidate& AOutCandidate);
			bool _GetTriangleEdgePose(const CetPath& AContour, int AEdgeIndex, TetTriangleEdgePose& AOutEdge) ;
			double _NormalizeAngle(double AAngle) ;
			CetInpoint _RotatePoint(const CetInpoint& APoint, double ARotation) ;
			bool _BuildOppositeTrianglePairCandidate(const CetTNestItemVector& AOriginalItems,const std::vector<TetShapeFeature>& AFeatures,int AAIndex,int ABIndex,const TetNestOptions& AOptions,TetClusterCandidate& AOutCandidate);
			double _Cross(const CetInpoint&AA,const CetInpoint &AB,CetInpoint&AP);
			bool _GetTriangleThirdPoint(const CetPath& AContour,int AEdgeIndex,CetInpoint& AOutPoint);

		};
	}
}


