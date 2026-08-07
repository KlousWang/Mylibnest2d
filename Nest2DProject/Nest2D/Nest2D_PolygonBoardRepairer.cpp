#include "pch.h"
#include "Nest2D_PolygonBoardRepairer.h"

#include "Nest2D_RotationUtils.h"

#include "NestUtils.h"

#include <algorithm>
#include <iostream>
#include <map>

using namespace ClipperLib;
using namespace libnest2d;

namespace ET {
    namespace NEST2DMANAGERLIB {

      /*  struct TetPlacementCandidate
        {
            TetPlacementCandidate(): ItemIndex(0), TargetBin(-1), Translation(Point(0, 0)), Rotation(libnest2d::Radians(0.0)){
            }

            std::size_t ItemIndex;
            int TargetBin;
            Point Translation;
            libnest2d::Radians Rotation;
        };*/

        CetPolygonBoardRepairer::CetPolygonBoardRepairer(): CetCoreObject()
        {
        }

        CetPolygonBoardRepairer::~CetPolygonBoardRepairer()
        {
        }

        CetPolygonBoardRepairer::CetPolygonBoardRepairer(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight): CetCoreObject()
        {
            SetContext(ANestItems,AOptions,ABinPoly,ABoardBinWidth,ABoardBinHeight);
        }
        void CetPolygonBoardRepairer::SetContext(CetTNestItemVector& ANestItems,const TetNestOptions& AOptions,const CetPolygonImpl& ABinPoly,double ABoardBinWidth,double ABoardBinHeight)
        {
            _Items = &ANestItems;
            _Options = &AOptions;
            _BinPoly = &ABinPoly;
            m_BoardBinWidth = ABoardBinWidth;
            m_BoardBinHeight = ABoardBinHeight;

            m_StepMm = std::max(0.5, _Options->Spacing/AOptions.Spacing);
            m_SpacingCoord = NestUtils::ToNestCoord(_Options->Spacing);

            _BuildRotations();
        }

        void CetPolygonBoardRepairer::Repair(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                std::cout << "[REPAIR][ERROR] Repairer context is null." << std::endl;
                return;
            }
            if (ALayers == 0){
                PackFromScratch(ALayers);
                return;
            }      
            auto& Items = *_Items;
            std::cout << "[REPAIR] start polygon board repair. Layers = " << ALayers << ", StepMm = " << m_StepMm << std::endl;
            _FixInvalidItems(ALayers);
            if (ALayers <= 1){
                ALayers = _CompactItemBins();

                std::cout << "[REPAIR] finish polygon board repair. Layers = " << ALayers << std::endl;
                return;
            }

			_FillHoles(ALayers);

            ALayers = _CompactItemBins();

            std::cout << "[REPAIR] finish polygon board repair. Layers = " << ALayers << std::endl;
        }

        void CetPolygonBoardRepairer::PackFromScratch(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                std::cout << "[PACK][ERROR] context is null." << std::endl;
                ALayers = 0;
                return;
            }

            if (!_Options->Board.Enabled || _Options->Board.Vertices.size() < 3){
                ALayers = 0;
                return;
            }

            auto& Items = *_Items;

            std::cout << "[PACK] start pack from scratch. Items = " << Items.size() << ", StepMm = " << m_StepMm << std::endl;

            
            for (auto& Item : Items){
                Item.binId(-1);
                Item.translation(Point(0, 0));
                Item.rotation(libnest2d::Radians(0.0));
            }

            std::size_t UsedLayers = 0;

            for (std::size_t i = 0; i < Items.size(); ++i){
                bool Placed = false;

                
                for (int TargetBin = 0; TargetBin < static_cast<int>(UsedLayers); ++TargetBin){
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)){
                        Placed = true;
                        break;
                    }
                }

                
                if (!Placed){
                    int NewBin = static_cast<int>(UsedLayers);

                    if (_TryPlaceItemInBinByGrid(i, NewBin)){
                        Placed = true;
                        ++UsedLayers;

                        std::cout << "[PACK] item " << i << " placed in new bin " << NewBin << std::endl;
                    }
                }

                
                if (!Placed){
                    Items[i].binId(-1);

                    std::cout << "[PACK][WARN] item " << i << " cannot be placed in polygon board." << std::endl;
                }
            }

            ALayers = _CompactItemBins();

            std::cout << "[PACK] finish pack from scratch. Layers = " << ALayers << std::endl;
        }

        void CetPolygonBoardRepairer::_BuildRotations()
        {
            const int RotationCount = _Options == nullptr ? 0 : _Options->Rotations;
            m_Rotations = CetRotationUtils::BuildAllowedLibRotations(RotationCount);
        }

        std::size_t CetPolygonBoardRepairer::_CompactItemBins()
        {
            if (_Items == nullptr){
                return 0;
            }

            auto& Items = *_Items;

            std::map<int, int> Remap;
            int NextBin = 0;

            for (auto& Item : Items){
                int OldBin = static_cast<int>(Item.binId());

                if (OldBin < 0){
                    continue;
                }

                auto It = Remap.find(OldBin);

                if (It == Remap.end()){
                    Remap[OldBin] = NextBin;
                    Item.binId(NextBin);
                    ++NextBin;
                }
                else {
                    Item.binId(It->second);
                }
            }

            return static_cast<std::size_t>(NextBin);
        }

        bool CetPolygonBoardRepairer::_TryPlaceItemInBinByGrid(std::size_t AItemIndex,int ATargetBin)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                return false;
            }

            auto& Items = *_Items;

            if (AItemIndex >= Items.size()){
                return false;
            }

            for (auto Angle : m_Rotations){
                for (double Y = 0.0; Y < m_BoardBinHeight; Y += m_StepMm){
                    for (double X = 0.0; X < m_BoardBinWidth; X += m_StepMm){
                        TetPlacementCandidate Placement;

                        Placement.ItemIndex = AItemIndex;
                        Placement.TargetBin = ATargetBin;
                        Placement.Rotation = Angle;

                        _FillTranslationForBBoxMin(Placement,X,Y);

                        if (_CanPlaceAt(Placement)){
                            Items[AItemIndex].translation(Placement.Translation);
                            Items[AItemIndex].rotation(Placement.Rotation);
                            Items[AItemIndex].binId(ATargetBin);

                            std::cout << "[REPAIR] item " << AItemIndex << " moved to bin " << ATargetBin << ", x = " << X << ", y = " << Y << std::endl;

                            return true;
                        }
                    }
                }
            }

            return false;
        }

        bool CetPolygonBoardRepairer::_CanPlaceAt(const TetPlacementCandidate& APlacement)
        {
            if (_Items == nullptr || _BinPoly == nullptr){
                return false;
            }

            auto& Items = *_Items;
            const auto& BinPoly = *_BinPoly;

            using NestItemType = CetTNestItemVector::value_type;

            if (APlacement.ItemIndex >= Items.size()){
                return false;
            }

            auto& Candidate = Items[APlacement.ItemIndex];

            Point OldTranslation = Candidate.translation();
            libnest2d::Radians OldRotation = Candidate.rotation();
            int OldBin = static_cast<int>(Candidate.binId());
            auto OldInflation = Candidate.inflation();

            Candidate.translation(APlacement.Translation);
            Candidate.rotation(APlacement.Rotation);
            Candidate.binId(APlacement.TargetBin);
            Candidate.inflation(0);

            bool CanPlace = true;

            auto BB = Candidate.boundingBox();

            if (getX(BB.minCorner()) < 0 ||getY(BB.minCorner()) < 0){
                CanPlace = false;
            }
            if (CanPlace &&(getX(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinWidth) ||getY(BB.maxCorner()) > NestUtils::ToNestCoord(m_BoardBinHeight))){
                CanPlace = false;
            }
            
            if (CanPlace && !Candidate.isInside(BinPoly)){
                CanPlace = false;
            }

            
            if (CanPlace && m_SpacingCoord > 0){
                Candidate.inflation(static_cast<decltype(OldInflation)>(m_SpacingCoord));
            }

            if (CanPlace){
                for (std::size_t i = 0; i < Items.size(); ++i){
                    if (i == APlacement.ItemIndex){
                        continue;
                    }

                    const auto& Other = Items[i];
                    int OtherBin = static_cast<int>(Other.binId());

                    if (OtherBin != APlacement.TargetBin || OtherBin < 0){
                        continue;
                    }

                    if (NestItemType::intersects(Candidate, Other)){
                        CanPlace = false;
                        break;
                    }
                }
            }

            Candidate.translation(OldTranslation);
            Candidate.rotation(OldRotation);
            Candidate.binId(OldBin);
            Candidate.inflation(OldInflation);

            return CanPlace;
        }

        void CetPolygonBoardRepairer::_FillTranslationForBBoxMin(TetPlacementCandidate& APlacement,double ATargetMinX,double ATargetMinY)
        {
            if (_Items == nullptr){
                APlacement.Translation = Point(0, 0);
                return;
            }

            auto& Items = *_Items;

            if (APlacement.ItemIndex >= Items.size()){
                APlacement.Translation = Point(0, 0);
                return;
            }

            auto& Item = Items[APlacement.ItemIndex];

            Point OldTranslation = Item.translation();
            libnest2d::Radians OldRotation = Item.rotation();

            
            Item.translation(Point(0, 0));
            Item.rotation(APlacement.Rotation);

            auto BB = Item.boundingBox();

            Point DesiredMin(NestUtils::ToNestCoord(ATargetMinX),NestUtils::ToNestCoord(ATargetMinY));

            
            APlacement.Translation = DesiredMin - BB.minCorner();

            Item.translation(OldTranslation);
            Item.rotation(OldRotation);
        }

        bool CetPolygonBoardRepairer::_IsCurrentPlacementValid(std::size_t AItemIndex)
        {
            if (_Items == nullptr){
                return false;
            }

            auto& Items = *_Items;

            if (AItemIndex >= Items.size()){
                return false;
            }

            int CurrentBin = static_cast<int>(Items[AItemIndex].binId());

            if (CurrentBin < 0){
                return false;
            }

            TetPlacementCandidate Placement;

            Placement.ItemIndex = AItemIndex;
            Placement.TargetBin = CurrentBin;
            Placement.Translation = Items[AItemIndex].translation();
            Placement.Rotation = Items[AItemIndex].rotation();

            return _CanPlaceAt(Placement);
        }

        void CetPolygonBoardRepairer::_FixInvalidItems(std::size_t& ALayers)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                return;
            }

            auto& Items = *_Items;

            if (ALayers == 0){
                return;
            }

            for (std::size_t i = 0; i < Items.size(); ++i){
                int OriginalBin = static_cast<int>(Items[i].binId());

                if (OriginalBin < 0){
                    continue;
                }

                if (_IsCurrentPlacementValid(i)){
                    continue;
                }

                std::cout << "[REPAIR] item " << i << " current placement invalid. Try relocate." << std::endl;

                Point OldTranslation = Items[i].translation();
                libnest2d::Radians OldRotation = Items[i].rotation();
                int OldBin = OriginalBin;

                
                Items[i].binId(-1);

                bool Placed = false;

                
                for (int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin){
                    if (_TryPlaceItemInBinByGrid(i, TargetBin)){
                        Placed = true;

                        std::cout << "[REPAIR] invalid item " << i << " relocated to existing bin " << TargetBin << std::endl;

                        break;
                    }
                }

                
                if (!Placed){
                    int NewBin = static_cast<int>(ALayers);

                    if (_TryPlaceItemInBinByGrid(i, NewBin)){
                        Placed = true;
                        ++ALayers;

                        std::cout << "[REPAIR] invalid item " << i << " relocated to new bin " << NewBin << std::endl;
                    }
                }

                
                if (!Placed){
                    Items[i].translation(OldTranslation);
                    Items[i].rotation(OldRotation);
                    Items[i].binId(OldBin);

                    std::cout << "[REPAIR][WARN] item " << i << " cannot be relocated." << std::endl;
                }
            }
        }

        void CetPolygonBoardRepairer::_FillHoles(std::size_t& ALayers)
        {
            if (_Items == nullptr || ALayers <= 1){
                return;
            }
			auto& Items = *_Items;

			bool Changed = true;

			int Iteration = 0;
			int MaxIterations = static_cast<int>(Items.size())*3;

			

            while (Changed && Iteration < MaxIterations){
                Changed = false;
                ++Iteration;

                for(int TargetBin = 0; TargetBin < static_cast<int>(ALayers); ++TargetBin){
					TetHoleFillCandidate BestCandidate;
                    if (!_FindBestCandidateForTargetBin(TargetBin,BestCandidate)){
                        continue;
                    }
                    _ApplyHoleFillCandidate(BestCandidate);
                    Changed = true;

                    std::cout << "[HOLE_FILL] move item " << BestCandidate.ItemIndex << " from bin " << BestCandidate.OldBin << " to bin " << BestCandidate.TargetBin << ", score = " << BestCandidate.Score << std::endl;
                }
                ALayers = _CompactItemBins();
            }
            std::cout << "[HOLE_FILL] finish. Iteration = " << Iteration << ", Layers = " << ALayers << std::endl;
        }

        bool CetPolygonBoardRepairer::_FindBestCandidateForTargetBin(int ATargetBin, TetHoleFillCandidate& ABestCandidate)
        {
            if (_Items == nullptr){
                return false;
            }

			auto& Items = *_Items;
            bool Found = false;

            for (std::size_t i = 0; i < Items.size(); ++i){
                int OldBin = static_cast<int>(Items[i].binId());

                
				
				if (OldBin >= 0 && OldBin <= ATargetBin){
					continue;
				}
				TetHoleFillCandidate Candidate;

				if (!_TryFindBestPlacementInBin(i, ATargetBin, Candidate)){
					continue;
				}
				if (!Found || Candidate.Score > ABestCandidate.Score){
					ABestCandidate = Candidate;
					Found = true;
				}
			}
            return Found;
        }

        bool CetPolygonBoardRepairer::_TryFindBestPlacementInBin(std::size_t AItemIndex, int ATargetBin, TetHoleFillCandidate& ABestCandidate)
        {
            if (_Items == nullptr || _Options == nullptr || _BinPoly == nullptr){
                return false;
            }
            auto& Items = *_Items;
            if (AItemIndex >= Items.size()){
                return false;
            }
            int OldBin = static_cast<int>(Items[AItemIndex].binId());
            bool Found = false;
            long long CheckedCount = 0;
            const long long TotalChecks =static_cast<long long>(m_Rotations.size()) *static_cast<long long>(std::ceil(m_BoardBinWidth / m_StepMm)) *static_cast<long long>(std::ceil(m_BoardBinHeight / m_StepMm));
           
            for (auto Angle : m_Rotations){
                for (double Y = 0.0; Y < m_BoardBinHeight; Y += m_StepMm){
                    for (double X = 0.0; X < m_BoardBinWidth; X += m_StepMm){
                        ++CheckedCount;
                        if (CheckedCount == 1 ||CheckedCount % 5000 == 0 ||CheckedCount == TotalChecks){
                            const double Percent =TotalChecks > 0? 100.0 * CheckedCount / TotalChecks: 100.0;
                            std::cout << "[HOLE_FILL][GRID] item = " << AItemIndex << ", oldBin = " << OldBin << ", targetBin = " << ATargetBin << ", progress = " << CheckedCount << " / " << TotalChecks << " (" << Percent << "%)" << std::endl;
                        }

                        TetPlacementCandidate Placement;
                        Placement.ItemIndex = AItemIndex;
                        Placement.TargetBin = ATargetBin;
                        Placement.Rotation = Angle;
                        _FillTranslationForBBoxMin(Placement,X,Y);
                        if (!_CanPlaceAt(Placement)){
                            continue;
                        }
                        double Score = _CalcHoleFillScore(AItemIndex,OldBin,ATargetBin,Placement.Translation);
                        if (!Found || Score > ABestCandidate.Score){
                            ABestCandidate.Valid = true;
                            ABestCandidate.ItemIndex = AItemIndex;
                            ABestCandidate.OldBin = OldBin;
                            ABestCandidate.TargetBin = ATargetBin;
                            ABestCandidate.Translation = Placement.Translation;
                            ABestCandidate.Rotation = Placement.Rotation;
                            ABestCandidate.Score = Score;
                            Found = true;
                        }
                    }
                }
            }

            return Found;
        }

        void CetPolygonBoardRepairer::_ApplyHoleFillCandidate(const TetHoleFillCandidate& ACandidate)
        {
            if (_Items == nullptr || !ACandidate.Valid){
                return;
            }
            auto& Items = *_Items;
            if (ACandidate.ItemIndex >= Items.size()){
                return;
            }
            auto& Item = Items[ACandidate.ItemIndex];
            Item.translation(ACandidate.Translation);
            Item.rotation(ACandidate.Rotation);
            Item.binId(ACandidate.TargetBin);
        }

        double CetPolygonBoardRepairer::_CalcHoleFillScore(std::size_t AItemIndex, int AOldBin, int ATargetBin, const libnest2d::Point& ATranslation)
        {
            if (_Items == nullptr || AItemIndex >= _Items->size()){
                return -std::numeric_limits<double>::max();
            }
            const auto& Item = (*_Items)[AItemIndex];
            double ItemArea = std::abs(static_cast<double>(Item.area()));
            
            double BinImprove = static_cast<double>(AOldBin - ATargetBin);
           
            double XPenalty = static_cast<double>(ATranslation.X) * 0.000001;
            double YPenalty = static_cast<double>(ATranslation.Y) * 0.000001;
            double Score =ItemArea * 10.0+ BinImprove * 1000000.0- XPenalty- YPenalty;
            return Score;
        }

    } // namespace NEST2DMANAGERLIB
} // namespace ET