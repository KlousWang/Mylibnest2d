#pragma once
#include "EtTechCore_Object.h"
#include"Nest2D_DataType.h"
#include"Nest2D_PrivateDataType.h"

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
			CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const libnest2d::PolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
			~CetPolygonBoardRepairer();
		public:
			void SetContext(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const libnest2d::PolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
			void Repair(std::size_t& ALayers);
			
			//CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const libnest2d::PolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight);
		protected:
			struct TetPlacementCandidate;

		protected:
			void PackFromScratch(std::size_t& ALayers);
			void _BuildRotations();

			std::size_t _CompactItemBins();

			bool _TryPlaceItemInBinByGrid(std::size_t AItemIndex,int ATargetBin);

			bool _CanPlaceAt(const TetPlacementCandidate& APlacement);

			void _FillTranslationForBBoxMin(TetPlacementCandidate& APlacement,double ATargetMinX,double ATargetMinY);
			bool _IsCurrentPlacementValid(std::size_t AItemIndex);

			void _FixInvalidItems(std::size_t& ALayers);

		protected:
			CetTNestItemVector* _Items = nullptr;
			const TetNestOptions* _Options = nullptr;
			const libnest2d::PolygonImpl* _BinPoly = nullptr;

			double m_BoardBinWidth = 0.0;
			double m_BoardBinHeight = 0.0;

			double m_StepMm = 1.0;
			long long m_SpacingCoord = 0;

			std::vector<libnest2d::Radians> m_Rotations;
		
		};

	}
}