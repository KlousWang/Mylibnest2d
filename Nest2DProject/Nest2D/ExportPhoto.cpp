#include "pch.h"
#include "ExportPhoto.h"
#include"NestUtils.h"
#include"Nest2D_DataConst.h"
#include"Nest2D_PrivateDataType.h"
#include"Nest2D_SelfFunction.h"
#include"Nest2D_AreaUsageCalculator.h"
//#include<libnest2d/backends/clipper/geometries.hpp>
//#include<libnest2d/libnest2d.hpp>
#include <libnest2d/utils/svgtools.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
//#include"EtTechCore_Object.h"

using namespace libnest2d;
using namespace ClipperLib;

namespace {
    std::string MakeUtilizationSvgText(const TetBoardUsageResult& AUsage, double ASvgWidth, double ASvgHeight)
    {
        if (AUsage.BoardArea <= 0.0 || ASvgWidth <= 0.0 || ASvgHeight <= 0.0){
            return "";
        }

        double Percent = std::max(0.0, AUsage.UsagePercent);
        double PureUsagePercnt = std::max(0.0, AUsage.PureUsagePercnt);

        if (!std::isfinite(Percent)){
            return "";
        }

        const double FontSize = std::clamp(std::min(ASvgWidth, ASvgHeight) * 0.025, 3.0, 12.0);
        const double Margin = std::max(2.0, FontSize * 0.6);

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "<text x=\"" << Margin
            << "\" y=\"" << (Margin + FontSize)
            << "\" font-family=\"Arial, Helvetica, sans-serif\""
            << " font-size=\"" << FontSize
            << "\" fill=\"#111111\""
            << " fill-opacity=\"0.85\">UsagePercent: "
            << Percent<< "%"
            << " PureUsagePercnt: "
            << PureUsagePercnt<< "%</text>\n";

        return ss.str();
    }

    std::vector<libnest2d::Item> BuildSvgItemsForBin(const std::vector<TetNestPolygon>& AItems, int ABin)
    {
        std::vector<libnest2d::Item> Result;
        for (const auto& Item : AItems){
            if (Item.Out_bin != ABin){ continue; }
            Path Outer; for (const auto& PointData : Item.Vertices){ Outer.push_back(Point(NestUtils::ToNestCoord(PointData.X), NestUtils::ToNestCoord(PointData.Y))); }
            if (Outer.size() < 3){ continue; }
            if (!ClipperLib::Orientation(Outer)){ std::reverse(Outer.begin(), Outer.end()); }
            Paths Holes;
            for (const auto& HoleData : Item.Holes){
                if (HoleData.size() < 3){ continue; }
                Path Inner; for (const auto& PointData : HoleData){ Inner.push_back(Point(NestUtils::ToNestCoord(PointData.X), NestUtils::ToNestCoord(PointData.Y))); }
                if (ClipperLib::Orientation(Inner)){ std::reverse(Inner.begin(), Inner.end()); }
                Holes.push_back(std::move(Inner));
            }
            CetPolygonImpl Polygon(std::move(Outer), std::move(Holes));
            libnest2d::Item SvgItem(Polygon);
            SvgItem.rotation(Item.Out_angle); SvgItem.translation(Point(NestUtils::ToNestCoord(Item.Out_x), NestUtils::ToNestCoord(Item.Out_y))); SvgItem.binId(Item.Out_bin);
            Result.push_back(std::move(SvgItem));
        }
        return Result;
    }
}

namespace ET {
	namespace NEST2DMANAGERLIB {
		CetExportPhoto::CetExportPhoto() :CetCoreObject()
		{
		}
		CetExportPhoto::~CetExportPhoto()
		{
		}
		int CetExportPhoto::ExportSvg(const std::vector<TetNestPolygon>& AItems, const TetNestOptions& AOptions, int AUsedBins) 
		{
            std::cout << "[SVG] Board.Enabled = " << AOptions.Board.Enabled << ", Board.Vertices.size = " << AOptions.Board.Vertices.size() << std::endl;

            for (size_t i = 0; i < AOptions.Board.Vertices.size(); ++i){
                std::cout << "[SVG] Board Pt " << i << " = " << AOptions.Board.Vertices[i].X << ", " << AOptions.Board.Vertices[i].Y << std::endl;
            }
			std::cout << "[DLL]this is export svg1" << std::endl;
			if (AItems.empty() || AUsedBins <= 0) return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
			if (AOptions.SvgPath.empty()) return NEST2D_ERR_EXPORT_NO_PATH;

			Box binSize(NestUtils::ToNestCoord(AOptions.BinWidth), NestUtils::ToNestCoord(AOptions.BinHeight));

            CetAreaUsageCalculator LocalUsageCalculator;
            CetAreaUsageCalculator* UsageCalculator = Nest2DUtils->Nest2DAreaUsage != nullptr ? Nest2DUtils->Nest2DAreaUsage : &LocalUsageCalculator;
            const std::vector<TetBoardUsageResult> BoardUsages = UsageCalculator->CalculateBoardUsages(AItems, AOptions, AUsedBins);

			using SvgWriter = svg::SVGWriter<CetPolygonImpl>;
			SvgWriter::Config conf;
			conf.mm_in_coord_units = mm();

			
			std::string basePath = AOptions.SvgPath;
			size_t extPos = basePath.find(".svg");
			if (extPos != std::string::npos){
				basePath = basePath.substr(0, extPos);
			}

			
            for (int currentBin = 0; currentBin < AUsedBins; ++currentBin){

                SvgWriter svgw(conf);
                svgw.setSize(binSize);

                std::vector<libnest2d::Item> currentBinItems = BuildSvgItemsForBin(AItems, currentBin);
                
                if (currentBinItems.empty()){
                    std::cout << "[WARN] Skip empty bin: " << currentBin << std::endl;
                    continue;
                }
                CetPackGround pgrp(1);
                for (auto& svgItem : currentBinItems){
                    pgrp[0].emplace_back(svgItem);
                }
                svgw.writePackGroup(pgrp);
                std::string finalPath;
                if(AUsedBins>1)  finalPath = basePath + "_" + std::to_string(currentBin);
                else finalPath = basePath;
                svgw.save(finalPath);
                std::string realSvgPath = finalPath;
                std::ifstream testFile(realSvgPath.c_str(), std::ios::in | std::ios::binary);
                if (!testFile.is_open()){
                    realSvgPath = finalPath + ".svg";
                }
                else {testFile.close();}
                std::cout << "[SVG] realSvgPath = " << realSvgPath << std::endl;
                std::string ExtraSvg;
                if (AOptions.Board.Enabled && AOptions.Board.Vertices.size() >= 3){
                    std::string boardPath = Nest2DUtils->Nest2DSvgUtils->MakeBoardSvgPath(AOptions.Board,AOptions.BinHeight);
                    ExtraSvg += boardPath;
                }
                if (currentBin >= 0 && static_cast<std::size_t>(currentBin) < BoardUsages.size()){
                    ExtraSvg += MakeUtilizationSvgText(BoardUsages[static_cast<std::size_t>(currentBin)],AOptions.BinWidth,AOptions.BinHeight);
                }

                Nest2DUtils->Nest2DSvgUtils->InsertTextBeforeSvgEnd(realSvgPath, ExtraSvg);
            }

			return 0;
		}

		int CetExportPhoto::ExportSvgItems(CetTNestItemVector& ANestItems, const TetNestOptions& AOptions, int AUsedBins)
		{
			//std::cout << "[DLL]this is export svg2" << std::endl;
			if (ANestItems.empty() || AUsedBins <= 0) return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
			if (AOptions.SvgPath.empty()) return NEST2D_ERR_EXPORT_NO_PATH;
			const auto binWidth = NestUtils::ToNestCoord(AOptions.BinWidth);
			const auto binHeight = NestUtils::ToNestCoord(AOptions.BinHeight);

			Box binSize(binWidth, binHeight);
			using SvgWriter = svg::SVGWriter<CetPolygonImpl>;

			SvgWriter::Config conf;
			conf.mm_in_coord_units = mm();

			SvgWriter svgw(conf);
			svgw.setSize(binSize);

			CetPackGround pgrp(AUsedBins);

			for (auto& item : ANestItems){
				int binId = static_cast<int>(item.binId());
				if (binId >= 0 && binId < AUsedBins){
					pgrp[static_cast<size_t>(binId)].emplace_back(item);
				}
			}
			std::string basePath = AOptions.SvgPath;
			if (basePath.size() >= 4 && basePath.substr(basePath.size() - 4) == ".svg"){
				basePath = basePath.substr(0, basePath.size() - 4);
			}

			svgw.writePackGroup(pgrp);
			svgw.save(basePath);

			return 0;
		}
        int CetExportPhoto::ExportSvgPackGroup(const CetPackGround& APackGroup,const TetNestOptions& AOptions)
        {
            if (APackGroup.empty()){
                return NEST2D_ERR_EXPORT_EMPTY_ITEMS;
            }

            if (AOptions.SvgPath.empty()){
                return NEST2D_ERR_EXPORT_NO_PATH;
            }

            const auto binWidth = NestUtils::ToNestCoord(AOptions.BinWidth);
            const auto binHeight = NestUtils::ToNestCoord(AOptions.BinHeight);

            Box binSize(binWidth, binHeight);

            using SvgWriter = svg::SVGWriter<CetPolygonImpl>;

            SvgWriter::Config conf;
            conf.mm_in_coord_units = mm();

            std::string basePath = AOptions.SvgPath;
            if (basePath.size() >= 4 && basePath.substr(basePath.size() - 4) == ".svg"){
                basePath = basePath.substr(0, basePath.size() - 4);
            }

            for (std::size_t binIndex = 0; binIndex < APackGroup.size(); ++binIndex){
                if (APackGroup[binIndex].empty()){
                    std::cout << "[SVG][Filler] Skip empty bin: " << binIndex << std::endl;
                    continue;
                }

                SvgWriter svgw(conf);
                svgw.setSize(binSize);

                CetPackGround singleBinGroup(1);

                for (const auto& itemRef : APackGroup[binIndex]){
                    singleBinGroup[0].emplace_back(itemRef);
                }

                svgw.writePackGroup(singleBinGroup);

                std::string finalPath;

                if (APackGroup.size() > 1){
                    finalPath = basePath + "_" + std::to_string(binIndex);
                }
                else {
                    finalPath = basePath;
                }

                svgw.save(finalPath);

                std::cout << "[SVG][Filler] saved bin " << binIndex << " to " << finalPath << ".svg" << std::endl;
            }

            return Nest2D_Success;
        }
	}
}

