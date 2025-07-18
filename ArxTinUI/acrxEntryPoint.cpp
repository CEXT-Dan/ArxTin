// (C) Copyright 2002-2007 by Autodesk, Inc. 
//
// Permission to use, copy, modify, and distribute this software in
// object code form for any purpose and without fee is hereby granted, 
// provided that the above copyright notice appears in all copies and 
// that both that copyright notice and the limited warranty and
// restricted rights notice below appear in all supporting 
// documentation.
//
// AUTODESK PROVIDES THIS PROGRAM "AS IS" AND WITH ALL FAULTS. 
// AUTODESK SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTY OF
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE.  AUTODESK, INC. 
// DOES NOT WARRANT THAT THE OPERATION OF THE PROGRAM WILL BE
// UNINTERRUPTED OR ERROR FREE.
//
// Use, duplication, or disclosure by the U.S. Government is subject to 
// restrictions set forth in FAR 52.227-19 (Commercial Computer
// Software - Restricted Rights) and DFAR 252.227-7013(c)(1)(ii)
// (Rights in Technical Data and Computer Software), as applicable.
//

//-----------------------------------------------------------------------------
//----- acrxEntryPoint.cpp
//-----------------------------------------------------------------------------
#include "StdAfx.h"
#include "resource.h"
#include "CeDbTin.h"

//-----------------------------------------------------------------------------
#define szRDS _RXST("")

//-----------------------------------------------------------------------------
//----- ObjectARX EntryPoint
class CArxTinUIApp : public AcRxArxApp {

public:
    CArxTinUIApp() : AcRxArxApp() {}

    virtual AcRx::AppRetCode On_kInitAppMsg(void* pkt)
    {
        AcRx::AppRetCode retCode = AcRxArxApp::On_kInitAppMsg(pkt);
        acrxLockApplication(pkt);
        return (retCode);
    }

    virtual AcRx::AppRetCode On_kUnloadAppMsg(void* pkt)
    {
        AcRx::AppRetCode retCode = AcRxArxApp::On_kUnloadAppMsg(pkt);
        return (retCode);
    }

    virtual void RegisterServerComponents()
    {
    }

    static auto entsel() -> std::tuple<Acad::PromptStatus, AcDbObjectId, AcGePoint3d>
    {
        AcDbObjectId id;
        AcGePoint3d pnt;
        ads_name name = { 0L };
        int res = acedEntSel(L"\nSelect it: ", name, asDblArray(pnt));
        if (auto es = acdbGetObjectId(id, name); es != eOk)
            return std::make_tuple(Acad::PromptStatus::eError, id, pnt);
        return std::make_tuple(Acad::PromptStatus(res), id, pnt);
    }

    static auto ssget() -> std::tuple<Acad::PromptStatus, AcDbObjectIdArray>
    {
        AcDbObjectIdArray ids;
        ads_name ssname = { 0L };
        AcResBufPtr filter{ acutBuildList(RTDXF0, ACRX_T("POINT"), RTNONE) };
        int res = acedSSGet(NULL, NULL, NULL, filter.get(), ssname);
        if (res != RTNORM || acedGetCurrentSelectionSet(ids) != eOk)
            return std::make_tuple(Acad::PromptStatus::eError, ids);
        acedSSFree(ssname);
        return std::make_tuple(Acad::PromptStatus(res), std::move(ids));
    }

    static auto getPoint() -> std::tuple<Acad::PromptStatus, AcGePoint3d>
    {
        AcGePoint3d pnt;
        int res = acedGetPoint(NULL, _T("\nGetPoint: "), asDblArray(pnt));;
        return std::make_tuple(Acad::PromptStatus(res), pnt);
    }

    static auto getGePoints(const AcDbObjectIdArray& ids) -> CePoints
    {
        CePoints points;
        points.reserve(ids.length());
        for (const auto& id : ids)
        {
            if (AcDbObjectPointer<AcDbPoint> pPoint(id); pPoint.openStatus() == eOk)
                points.emplace_back(pPoint->position());
        }
        return points;
    }

    static void CArxTinUIApp_tinner(void)
    {
        using CextDbTinUPtr = AcDbObjectUPtr<CextDbTin>;
        auto [es, ids] = ssget();
        if (es != Acad::PromptStatus::eNormal || ids.length() < 3)
        {
            acutPrintf(_T("\nOOF"));
            return;
        }

        double maxz = 1;
        double minz = 1;
        int32_t drawFlags = 0;
        if (auto res = acedGetReal(_T("\nEnter a major contour interval (Enter 0 for none): "), &maxz); res == RTNORM && !isZero(maxz))
        {
            drawFlags |= int32_t(CextDbTin::DrawFlags::kDrawContours);
            if (auto res = acedGetReal(_T("\nEnter a minor contour interval (Enter 0 for none): "), &minz); res != RTNORM || isZero(minz))
                minz = maxz;
        }

        int drawtin = 0;
        if (auto res = acedGetInt(_T("\nDraw triangles (1 = Y, 0 = N) <0> : "), &drawtin); res == RTNORM && drawtin != 0)
            drawFlags |= int32_t(CextDbTin::DrawFlags::kDrawTin);

        auto points = getGePoints(ids);
        AcDbDatabase* pDb = acdbCurDwg();
        AcDbBlockTableRecordPointer model(acdbSymUtil()->blockModelSpaceId(pDb), AcDb::OpenMode::kForWrite);

        PerfTimer timer(__FUNCTIONW__);
        CextDbTinUPtr ptin(new CextDbTin(points));

        //what to draw
        ptin->setDrawFlags(static_cast<CextDbTin::DrawFlags>(drawFlags));

        AcCmColor tincolor;
        tincolor.setColorIndex(139);
        ptin->setTinColor(tincolor);
        AcCmTransparency tinTr{ 1.0 - (50 * 0.01) };
        ptin->setTinTransparency(tinTr);

        //contours
        ptin->setMinorZ(minz);
        ptin->setMajorZ(maxz);

        AcCmColor mincolor;
        mincolor.setColorIndex(3);
        ptin->setMinorContourColor(mincolor);
        AcCmTransparency minorTr{ 1.0 - (50 * 0.01) };
        ptin->setMinorTransparency(minorTr);

        AcCmColor majcolor;
        majcolor.setColorIndex(1);
        ptin->setMajorContourColor(majcolor);

        model->appendAcDbEntity(ptin.get());
        timer.end(_T("Done "));
    }

    static void CArxTinUIApp_tintest(void)
    {
        auto [ps, id, pnt] = entsel();
        AcDbObjectPointer<CextDbTin> tin(id);

        auto [ps2, pnt2] = getPoint();

        PerfTimer timer(__FUNCTIONW__);
        double elev = 0;
        tin->getElevationFromPoint(pnt2, elev);
        acutPrintf(_T("\nFound %f"), elev);
        timer.end(_T("Done "));
    }
};

//-----------------------------------------------------------------------------
#pragma warning( disable: 4838 ) //prevents a cast compiler warning, 
ACED_ARXCOMMAND_ENTRY_AUTO(CArxTinUIApp, CArxTinUIApp, _tinner, tinner, ACRX_CMD_TRANSPARENT, NULL)
ACED_ARXCOMMAND_ENTRY_AUTO(CArxTinUIApp, CArxTinUIApp, _tintest, tintest, ACRX_CMD_TRANSPARENT, NULL)
IMPLEMENT_ARX_ENTRYPOINT(CArxTinUIApp)
#pragma warning( pop )
