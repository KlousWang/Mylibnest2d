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
		Reg_EtCore_Obj_Str(Nest2D_PolygonBoardRepairer, CetPolygonBoardRepairer)
		//Reg_EtCore_Obj_Str(File_Load, CetFile)
	}

}