#include"pch.h"
#include"File.h"
#include"Nest2DAPI.h"
#include"Nest2D_Engine.h"
#include"Nest2D_InvokeFunctor.h"
#include"NestDataMapper.h"
#include"ExportPhoto.h"


namespace ET {
	namespace NEST2DMANAGERLIB {
		using namespace ET::CORE;

		Reg_EtCore_Obj_Str(Nest_2D, CetNest2DManager)
		Reg_EtCore_Obj_Str(File_Load, CetFile)
		Reg_EtCore_Obj_Str(Nest2D_Engine, CetNest2DEngine)
		Reg_EtCore_Obj_Str(Nest2D_InvokeFunctor, CetNest2DInvokeFunctor)
		Reg_EtCore_Obj_Str(Nest2D_DataMapper, CetNestDataMapper)
		Reg_EtCore_Obj_Str(Nest2D_ExportPhoto, CetExportPhoto)
		//Reg_EtCore_Obj_Str(File_Load, CetFile)
	}

}