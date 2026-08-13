#include"pch.h"
#include"File.h"
#include"Nest2DAPI.h"
#include"Nest2D_Engine.h"
#include"Nest2D_InvokeFunctor.h"
#include"NestDataMapper.h"
#include"ExportPhoto.h"
#include "Nest2D_BoardUtils.h"
#include "Nest2D_GeometryUtils.h"
#include "Nest2D_SvgUtils.h"
#include "Nest2D_PolygonBoardRepairer.h"
#include"Nest2D_ClusterManager.h"
#include"Nest2D_StrategyManager.h"
#include"Nest2D_Sort.h"
#include"Nest2D_AreaUsageCalculator.h"
#include"Nest2D_ShapeAnalyzer.h"
#include"Nest2D_TriangleClusterBuilder.h"
//#include"Nest2D_ClusterInvokeFunctor.h"
#include"Nest2D_CircleClusterBuilder.h"
#include"Nest2D_RectangleFillClusterBuilder.h"
namespace ET {
	namespace NEST2DMANAGERLIB {
		using namespace ET::CORE;

		Reg_EtCore_Obj_Str(Nest_2D, CetNest2DManager)
		Reg_EtCore_Obj_Str(File_Load, CetFile)
		Reg_EtCore_Obj_Str(Nest2D_Engine, CetNest2DEngine)
		Reg_EtCore_Obj_Str(Nest2D_InvokeFunctor, CetNest2DInvokeFunctor)
		Reg_EtCore_Obj_Str(Nest2D_DataMapper, CetNestDataMapper)
		Reg_EtCore_Obj_Str(Nest2D_ExportPhoto, CetExportPhoto)
		Reg_EtCore_Obj_Str(Nest2D_BoardUtils, CetNest2DBoardUtils)
		Reg_EtCore_Obj_Str(Nest2D_GeometryUtils, CetGeometryUtils)
		Reg_EtCore_Obj_Str(Nest2D_SvgUtils, CetSvgUtils)
		// Keep factory allocation synchronized with the repairer's private layout.
		Reg_EtCore_Obj_Str(Nest2D_PolygonBoardRepairer, CetPolygonBoardRepairer)
		Reg_EtCore_Obj_Str(Nest2D_ClusterManager, CetClusterManager)
		Reg_EtCore_Obj_Str(Nest2D_StrategyManager, CetStrategyManager)
		Reg_EtCore_Obj_Str(Nest2D_Sort, CetSort)
		Reg_EtCore_Obj_Str(Nest2D_AreaUsageCalculator, CetAreaUsageCalculator)

			//Reg_EtCore_Obj_Str(Nest2D_ClusterInvokeFunctor, CetNest2DClusterInvokeFunctor)
		Reg_EtCore_Obj_Str(Nest2D_ShapeAnalyzer, CetShapeAnalyzer)
		Reg_EtCore_Obj_Str(Nest2D_TriangleClusterBuilder, CetTriangleClusterBuilder)
		Reg_EtCore_Obj_Str(Nest2D_CircleClusterBuilder, CetCircleClusterBuilder)
		Reg_EtCore_Obj_Str(Nest2D_RectangleFillClusterBuilder, CetRectangleFillClusterBuilder)
		//Reg_EtCore_Obj_Str(File_Load, CetFile)
	}

}
