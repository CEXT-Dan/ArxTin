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
//----- CeDbTin.cpp : Implementation of CCeDbTin
//-----------------------------------------------------------------------------
#include "StdAfx.h"
#include "CeDbTin.h"
#include "delaunator-header-only.hpp"

//-----------------------------------------------------------------------------
Adesk::UInt32 CextDbTin::kCurrentVersionNumber = 1;
constexpr auto CExtProxyFlags = AcDbProxyEntity::kAllAllowedBits;

ACRX_DXF_DEFINE_MEMBERS(
    CextDbTin, AcDbEntity,
    AcDb::kDHL_CURRENT, AcDb::kMReleaseCurrent,
    CExtProxyFlags, CEXTDBTIN,
    "CEXTDBTIN|Product Desc:  A TIN for CAD|Company:        CadExt|WEB Address:  github.com/CEXT-Dan/ArxTin"
)

//-----------------------------------------------------------------------------
CextDbTin::CextDbTin() : AcDbEntity()
{
}

CextDbTin::CextDbTin(const CePoints& points)
    : AcDbEntity(), m_points(points)
{
}

Acad::ErrorStatus CextDbTin::dwgOutFields(AcDbDwgFiler* pFiler) const
{
    assertReadEnabled();
    Acad::ErrorStatus es = AcDbEntity::dwgOutFields(pFiler);
    if (es != Acad::eOk)
        return (es);
    if ((es = pFiler->writeUInt32(CextDbTin::kCurrentVersionNumber)) != Acad::eOk)
        return (es);

    m_pointColor.dwgOutAsTrueColor(pFiler);
    m_tinColor.dwgOutAsTrueColor(pFiler);
    m_minorContourColor.dwgOutAsTrueColor(pFiler);
    m_majorContourColor.dwgOutAsTrueColor(pFiler);

    pFiler->writeUInt32(m_pointTransparency.serializeOut());
    pFiler->writeUInt32(m_tinTransparency.serializeOut());
    pFiler->writeUInt32(m_minorTransparency.serializeOut());
    pFiler->writeUInt32(m_majorTransparency.serializeOut());

    pFiler->writeDouble(m_minorZ);
    pFiler->writeDouble(m_majorZ);

    pFiler->writeInt32(static_cast<Adesk::Int32>(m_drawFlags));
    pFiler->writeInt32(static_cast<Adesk::Int32>(m_tinFlags));

    if (es = pFiler->writeUInt64(Adesk::UInt64(m_points.size())); es != eOk)
        return (es);
    for (const auto& p : m_points)
    {
        if (es = pFiler->writePoint3d(p); es != eOk)
            return es;
    }
    return (pFiler->filerStatus());
}

Acad::ErrorStatus CextDbTin::dwgInFields(AcDbDwgFiler* pFiler)
{
    assertWriteEnabled();
    Acad::ErrorStatus es = AcDbEntity::dwgInFields(pFiler);
    if (es != Acad::eOk)
        return (es);
    Adesk::UInt32 version = 0;
    if ((es = pFiler->readUInt32(&version)) != Acad::eOk)
        return (es);
    if (version > CextDbTin::kCurrentVersionNumber)
        return (Acad::eMakeMeProxy);

    //
    if (version < CextDbTin::kCurrentVersionNumber)
        return (Acad::eMakeMeProxy);

    m_pointColor.dwgInAsTrueColor(pFiler);
    m_tinColor.dwgInAsTrueColor(pFiler);
    m_minorContourColor.dwgInAsTrueColor(pFiler);
    m_majorContourColor.dwgInAsTrueColor(pFiler);
    {//Transparency
        Adesk::UInt32 tmptr = 0;
        pFiler->readUInt32(&tmptr);
        m_pointTransparency.serializeIn(tmptr);
        tmptr = 0;
        pFiler->readUInt32(&tmptr);
        m_tinTransparency.serializeIn(tmptr);
        tmptr = 0;
        pFiler->readUInt32(&tmptr);
        m_minorTransparency.serializeIn(tmptr);
        tmptr = 0;
        pFiler->readUInt32(&tmptr);
        m_majorTransparency.serializeIn(tmptr);
    }
    pFiler->readDouble(&m_minorZ);
    pFiler->readDouble(&m_majorZ);
    {//flags
        Adesk::Int32 tmpFlags = 0;
        pFiler->readInt32(&tmpFlags);
        m_drawFlags = static_cast<DrawFlags>(tmpFlags);

        tmpFlags = 0;
        pFiler->readInt32(&tmpFlags);
        m_tinFlags = static_cast<TinFlags>(tmpFlags);
    }
    Adesk::UInt64 npoints = 0;
    if (es = pFiler->readUInt64(&npoints); es != eOk)
        return es;
    m_points.clear();
    m_points.reserve(npoints);
    for (Adesk::Int64 idx = 0; idx < npoints; idx++)
    {
        AcGePoint3d pnt;
        if (es = pFiler->readPoint3d(&pnt); es != eOk)
            return es;
        else
            m_points.push_back(pnt);
    }
    m_dirty = true;
    return (pFiler->filerStatus());
}

#ifdef _NEVER //TODO
Acad::ErrorStatus CextDbTin::dxfOutFields(AcDbDxfFiler* pFiler) const
{
    assertReadEnabled();
    Acad::ErrorStatus es = AcDbEntity::dxfOutFields(pFiler);
    if (es != Acad::eOk)
        return (es);
    es = pFiler->writeItem(AcDb::kDxfSubclass, _RXST("CEXTDBTIN"));
    if (es != Acad::eOk)
        return (es);
    if ((es = pFiler->writeUInt32(kDxfInt32, CextDbTin::kCurrentVersionNumber)) != Acad::eOk)
        return (es);
    if (es = pFiler->writeInt64(kDxfInt64, Adesk::Int64(m_points.size())); es != eOk)
        return (es);
    for (const auto& p : m_points)
    {
        if (es = pFiler->writePoint3d(kDxfXCoord, p); es != eOk)
            return es;
    }
    return (pFiler->filerStatus());
}

Acad::ErrorStatus CextDbTin::dxfInFields(AcDbDxfFiler* pFiler) {
    assertWriteEnabled();

    Acad::ErrorStatus es = AcDbEntity::dxfInFields(pFiler);
    if (es != Acad::eOk || !pFiler->atSubclassData(_RXST("CEXTDBTIN")))
        return (pFiler->filerStatus());

    struct resbuf rb;
    pFiler->readItem(&rb);
    if (rb.restype != AcDb::kDxfInt32)
    {
        pFiler->pushBackItem();
        pFiler->setError(Acad::eInvalidDxfCode, _RXST("\nError: expected group code %d (version #)"), AcDb::kDxfInt32);
        return (pFiler->filerStatus());
    }
    Adesk::UInt32 version = (Adesk::UInt32)rb.resval.rlong;
    if (version > CextDbTin::kCurrentVersionNumber)
        return (Acad::eMakeMeProxy);
    if (version < CextDbTin::kCurrentVersionNumber)
        return (Acad::eMakeMeProxy);

    pFiler->readItem(&rb);
    if (rb.restype != AcDb::kDxfInt64)
    {
        pFiler->pushBackItem();
        pFiler->setError(Acad::eInvalidDxfCode, _RXST("\nError: expected group code %d (length #)"), AcDb::kDxfInt64);
        return (pFiler->filerStatus());
    }

    m_points.clear();
    Adesk::Int64 npoints = (Adesk::Int64)rb.resval.mnInt64;
    m_points.reserve(npoints);

    while (es == Acad::eOk && (es = pFiler->readResBuf(&rb)) == Acad::eOk)
    {
        switch (rb.restype)
        {
            case AcDb::kDxfXCoord:
                m_points.push_back(asPnt3d(rb.resval.rpoint));
                break;
            case AcDb::kDxfInt64:
                npoints = rb.resval.mnInt64;
                break;
            default:
                pFiler->pushBackItem();
                es = Acad::eEndOfFile;
                break;
        }
    }
    if (es != Acad::eEndOfFile)
        return (Acad::eInvalidResBuf);
    return (pFiler->filerStatus());
}
#endif

Acad::ErrorStatus CextDbTin::subOpen(AcDb::OpenMode mode)
{
    if (mode == AcDb::OpenMode::kForWrite)
        m_dirty = true;
    recompute();
    return (AcDbEntity::subOpen(mode));
}

Acad::ErrorStatus CextDbTin::subErase(Adesk::Boolean erasing) {
    return (AcDbEntity::subErase(erasing));
}

Acad::ErrorStatus CextDbTin::subCancel() {
    return (AcDbEntity::subCancel());
}

Acad::ErrorStatus CextDbTin::subClose()
{
    recompute();
    return (AcDbEntity::subClose());
}

void CextDbTin::subList() const
{
    assertReadEnabled();
    AcDbEntity::subList();
    acutPrintf(_T("\nNumber of points:     %ld"), m_points.size());
    acutPrintf(_T("\nNumber of triangles:  %ld"), m_triangles.size());
    acutPrintf(_T("\nMinimum elevation:    %lf"), m_zmin);
    acutPrintf(_T("\nMaximum elevation:    %lf"), m_zmax);
    acutPrintf(_T("\n          2D area:    %lf"), m_area2d);
    acutPrintf(_T("\n          3D area:    %lf"), m_area3d);
}

Adesk::Boolean CextDbTin::subWorldDraw(AcGiWorldDraw* mode)
{
    assertReadEnabled();
    auto& rTraits = mode->subEntityTraits();
    auto& rGeo = mode->geometry();

    computeTiangles();
    genMajorContours();
    genMinorContours();

    drawPoints(rTraits, rGeo);
    drawTriangles(rTraits, rGeo);
    drawContours(rTraits, rGeo);

    return true;
}

Adesk::UInt32 CextDbTin::subSetAttributes(AcGiDrawableTraits* traits) {
    assertReadEnabled();
    return (AcDbEntity::subSetAttributes(traits));
}

Acad::ErrorStatus CextDbTin::subGetOsnapPoints(
    AcDb::OsnapMode osnapMode,
    Adesk::GsMarker gsSelectionMark,
    const AcGePoint3d& pickPoint,
    const AcGePoint3d& lastPoint,
    const AcGeMatrix3d& viewXform,
    AcGePoint3dArray& snapPoints,
    AcDbIntArray& geomIds,
    const AcGeMatrix3d& insertionMat) const
{
    assertReadEnabled();
    if (osnapMode != AcDb::kOsModeEnd)
        return eOk;
    for (const auto& p : m_points)
        snapPoints.append(p);
    return eOk;
}

Acad::ErrorStatus CextDbTin::subTransformBy(const AcGeMatrix3d& xform)
{
    assertWriteEnabled();
    std::for_each(std::execution::par, m_points.begin(), m_points.end(), [&](AcGePoint3d& p) { p.transformBy(xform); });
    return xDataTransformBy(xform);
}

Adesk::Boolean CextDbTin::subCloneMeForDragging()
{
    return false; //TODO test
}

Acad::ErrorStatus CextDbTin::subExplode(AcDbVoidPtrArray& entitySet) const
{
    if (m_points.size())
    {
        if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawPoints)))
        {
            for (const auto& p : m_points)
            {
                AcDbPoint* point = new AcDbPoint(p);
                point->setColor(m_pointColor);
                point->setTransparency(m_pointTransparency);
                entitySet.append(point);
            }
        }
        if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawTin)))
        {
            for (const auto& tri : m_triangles)
            {
                AcDbFace* pFace = new AcDbFace(m_points[tri[0]], m_points[tri[1]], m_points[tri[2]]);
                pFace->setColor(m_tinColor);
                pFace->setTransparency(m_tinTransparency);
                entitySet.append(pFace);
            }
        }
        if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawContours)))
        {
            for (const auto& ctline : m_majorContours)
            {
                AcDbPolyline* pline = new AcDbPolyline(ctline.size());
                for (UInt32 idx = 0; idx < ctline.size(); idx++)
                {
                    if (idx == 0)
                        pline->setElevation(ctline[idx].z);
                    pline->addVertexAt(idx, AcGePoint2d(ctline[idx].x, ctline[idx].y));
                }
                pline->setColor(m_majorContourColor);
                pline->setTransparency(m_majorTransparency);
                entitySet.append(pline);
            }
            for (const auto& ctline : m_minorContours)
            {
                AcDbPolyline* pline = new AcDbPolyline(ctline.size());
                for (UInt32 idx = 0; idx < ctline.size(); idx++)
                {
                    if (idx == 0)
                        pline->setElevation(ctline[idx].z);
                    pline->addVertexAt(idx, AcGePoint2d(ctline[idx].x, ctline[idx].y));
                }
                pline->setColor(m_minorContourColor);
                pline->setTransparency(m_minorTransparency);
                entitySet.append(pline);
            }
        }
        return eOk;
    }
    return eNotApplicable;
}

//-----------------------------------------------------------------------------
//----- Not Autodesk
void CextDbTin::setPoints(const CePoints& points)
{
    assertWriteEnabled();
    m_points = points;
    m_dirty = true;
    recompute();
}


void CextDbTin::recompute(bool force /*= false*/)
{
    if (m_dirty || force)
    {
        //PerfTimer timer1(L"\nbegin computeTiangles");
        computeTiangles();
        //timer1.end(L"end computeTiangles");

        //PerfTimer timer2(L"\nbegin Contours");
        genMajorContours();
        genMinorContours();
        //timer2.end(L"end Contours");

        //PerfTimer timer3(L"\nbegin tree");
        createTree();
        //timer3.end(L"end tree");
        m_dirty = false;
    }
}

static double roundToNearest(double value, double multiple)
{
    double epsilon = std::numeric_limits<double>::epsilon() * multiple;
    double adj = std::signbit(value) ? -epsilon : epsilon;
    return std::round((value + adj) / multiple) * multiple;
}

static bool isMultiple(double a, double b)
{
    if (isZero(b))
        return false;
    return (int64_t(a) % int64_t(b) == 0);
}

Adesk::Boolean CextDbTin::drawPoints(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const
{
    if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawPoints)))
    {
        traits.setTransparency(m_pointTransparency);
        traits.setTrueColor(m_pointColor.entityColor());
        geo.polypoint(m_points.size(), m_points.data());
    }
    return Adesk::kTrue;
}

Adesk::Boolean CextDbTin::drawTriangles(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const
{
    if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawTin)))
    {
        traits.setTransparency(m_tinTransparency);
        traits.setTrueColor(m_tinColor.entityColor());
        std::array<AcGePoint3d, 3>pnts;
        for (const auto& tri : m_triangles)
        {
            pnts[0] = m_points[tri[0]];
            pnts[1] = m_points[tri[1]];
            pnts[2] = m_points[tri[2]];
            geo.polygon(pnts.size(), pnts.data());
        }
    }
    return Adesk::kTrue;
}

Adesk::Boolean CextDbTin::drawContours(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const
{
    if (GETBIT(int(m_drawFlags), int(DrawFlags::kDrawContours)))
    {
        for (const auto& pline : m_minorContours)
        {
            traits.setTransparency(m_minorTransparency);
            traits.setTrueColor(m_minorContourColor.entityColor());
            if (pline.size())
            {
                if (isMultiple(pline[0].z, m_minorZ))
                    geo.polyline(pline.size(), pline.data());
            }
        }
        for (const auto& pline : m_majorContours)
        {
            if (pline.size())
            {
                traits.setTransparency(m_majorTransparency);
                traits.setTrueColor(m_majorContourColor.entityColor());
                if (isMultiple(pline[0].z, m_majorZ))
                    geo.polyline(pline.size(), pline.data());
            }
        }
    }
    return Adesk::kTrue;
}

static void connectSegmentsIntoPolylines(const CeSegments& segments, CePolylines& polylines)
{
    // Map from point to all segments starting or ending at that point
    std::unordered_set<const CeSegment*, SegmentPtrHash> visited;
    std::unordered_multimap<AcGePoint3d, const CeSegment*, Point3DHash> pointToSegs;
    visited.reserve(segments.size());
    pointToSegs.reserve(segments.size() * 2);
    for (const auto& seg : segments)
    {
        pointToSegs.emplace(seg.first, &seg);
        pointToSegs.emplace(seg.second, &seg);
    }
    for (const auto& seg : segments)
    {
        if (visited.count(&seg))
            continue;

        CePolyline polyline, first, sec;
        sec.push_back(seg.second);
        first.push_back(seg.first);
        visited.insert(&seg);

        // Extend forward
        AcGePoint3d current = seg.second;
        while (true)
        {
            bool extended = false;
            auto range = pointToSegs.equal_range(current);
            for (auto it = range.first; it != range.second; ++it)
            {
                const CeSegment* nextSeg = it->second;
                if (visited.contains(nextSeg))
                    continue;

                if (current.isEqualTo(nextSeg->first))
                {
                    first.push_back(nextSeg->second);
                    current = nextSeg->second;
                }
                else if (current.isEqualTo(nextSeg->second))
                {
                    first.push_back(nextSeg->first);
                    current = nextSeg->first;
                }
                else
                {
                    continue;
                }
                visited.insert(nextSeg);
                extended = true;
                break;
            }
            if (!extended)
                break;
        }

        // Extend backward
        current = seg.first;
        while (true)
        {
            bool extended = false;
            auto range = pointToSegs.equal_range(current);
            for (auto it = range.first; it != range.second; ++it)
            {
                const CeSegment* prevSeg = it->second;
                if (visited.contains(prevSeg))
                    continue;

                if (current.isEqualTo(prevSeg->first))
                {
                    sec.push_back(prevSeg->second);
                    current = prevSeg->second;
                }
                else if (current.isEqualTo(prevSeg->second))
                {
                    sec.push_back(prevSeg->first);
                    current = prevSeg->first;
                }
                else
                {
                    continue;
                }
                visited.insert(prevSeg);
                extended = true;
                break;
            }
            if (!extended)
                break;
        }
        polyline.reserve(first.size() + sec.size());
        polyline.insert(polyline.end(), sec.rbegin(), sec.rend());
        polyline.insert(polyline.end(), first.begin(), first.end());
        polylines.push_back(polyline);
    }
}

static auto interpolate(const AcGePoint3d& a, const AcGePoint3d& b, double contourLevel) -> AcGePoint3d
{
    const double t = (contourLevel - a.z) / (b.z - a.z);
    return
    {
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        contourLevel
    };
}

static void processTriangle(const CePoints& points, const CeTriangle& tri, double contourLevel, CeSegmentsMap& map)
{
    CePoints crossings;
    for (int i = 0; i < 3; ++i)
    {
        const AcGePoint3d& a = points[tri[i]];
        const AcGePoint3d& b = points[tri[(i + 1) % 3]];
        if ((a.z < contourLevel && b.z > contourLevel) ||
            (a.z > contourLevel && b.z < contourLevel))
        {
            crossings.push_back(interpolate(a, b, contourLevel));
        }
    }
    if (crossings.size() == 2)
        map[crossings[0].z].push_back({ crossings[0], crossings[1] });
}

static void generateContours(const CePoints& points, const CeTriangles& triangles, const CeContourLevels& contourLevels, CeSegmentsMap& map)
{
    CeSegments segments;
    for (const auto& tri : triangles)
    {
        for (double level : contourLevels)
            processTriangle(points, tri, level, map);
    }
}

static void getCoords(const CePoints& points, CeCoords& outCoords, double& zmin, double& zmax)
{
    outCoords.reserve(points.size() * 2);
    for (const auto& item : points)
    {
        outCoords.emplace_back(item.x);
        outCoords.emplace_back(item.y);
        zmin = std::min(zmin, item.z);
        zmax = std::max(zmax, item.z);
    }
}

static double areaOfTriangle(const AcGePoint3d& point1, const AcGePoint3d& point2, const AcGePoint3d& point3)
{
    const AcGeVector3d& vector1 = point2 - point1;
    const AcGeVector3d& vector2 = point3 - point1;
    return 0.5 * vector1.crossProduct(vector2).length();
}

static double area2dOfTriangle(const AcGePoint3d& point1, const AcGePoint3d& point2, const AcGePoint3d& point3)
{
    AcGePoint3d _point1 = point1;
    AcGePoint3d _point2 = point2;
    AcGePoint3d _point3 = point3;
    _point1.z = 0.0;
    _point2.z = 0.0;
    _point3.z = 0.0;
    return areaOfTriangle(_point1, _point2, _point3);
}

void CextDbTin::computeTiangles()
{
    m_area2d = 0.0;
    m_area3d = 0.0;
    m_triangles.clear();
    CeCoords outCoords;
    m_zmin = std::numeric_limits<int64_t>::max();
    m_zmax = std::numeric_limits<int64_t>::min();
    getCoords(m_points, outCoords, m_zmin, m_zmax);
    delaunator::Delaunator d(outCoords);

    for (size_t i = 0; i < d.triangles.size(); i += 3)
    {
        const auto a = d.triangles[i + 0];
        const auto b = d.triangles[i + 1];
        const auto c = d.triangles[i + 2];
        m_triangles.emplace_back(CeTriangle{ a, b, c });
        m_area3d += areaOfTriangle(m_points[a], m_points[b], m_points[c]);
        m_area2d += area2dOfTriangle(m_points[a], m_points[b], m_points[c]);
    }
}

void CextDbTin::genMajorContours()
{
    m_majorContours.clear();
    if (isZero(m_majorZ))
        return;
    CeContourLevels contourLevels;
    for (int64_t idx = int64_t(m_zmin); idx < int64_t(m_zmax); idx += m_majorZ)
    {
        const auto nearest = roundToNearest(idx, m_majorZ);
        contourLevels.push_back(nearest);
        m_contourSet.insert(nearest);
    }
    m_minorContours.reserve(contourLevels.size());

    CeSegmentsMap map;
    map.reserve(contourLevels.size());
    generateContours(m_points, m_triangles, contourLevels, map);
    std::for_each(std::execution::par_unseq,map.begin(), map.end(), [&](const auto& kv)
        {
            std::lock_guard<std::mutex> lock(m_mtx_);
            connectSegmentsIntoPolylines(kv.second, m_majorContours);
        });
}

void CextDbTin::genMinorContours()
{
    m_minorContours.clear();
    if (isZero(m_minorZ))
        return;
    CeContourLevels contourLevels;
    for (int64_t idx = int64_t(m_zmin); idx < int64_t(m_zmax); idx += m_minorZ)
    {
        const auto nearest = roundToNearest(idx, m_minorZ);
        if (!m_contourSet.contains(nearest))
            contourLevels.push_back(nearest);
    }

    CeSegmentsMap map;
    map.reserve(contourLevels.size());
    generateContours(m_points, m_triangles, contourLevels, map);
    std::for_each(std::execution::par_unseq,map.begin(), map.end(), [&](const auto& kv)
        {
            std::lock_guard<std::mutex> lock(m_mtx_);
            connectSegmentsIntoPolylines(kv.second, m_minorContours);
        });
}

AcCmColor CextDbTin::pointColor() const
{
    return m_pointColor;
}

void CextDbTin::setpointColor(const AcCmColor& val)
{
    assertWriteEnabled();
    m_pointColor = val;
}

AcCmColor CextDbTin::tinColor() const
{
    return m_tinColor;
}

void CextDbTin::setTinColor(const AcCmColor& val)
{
    assertWriteEnabled();
    m_tinColor = val;
}

AcCmColor CextDbTin::majorContourColor() const
{
    return m_majorContourColor;
}

void CextDbTin::setMajorContourColor(const AcCmColor& val)
{
    assertWriteEnabled();
    m_majorContourColor = val;
}

AcCmColor CextDbTin::getMinorContourColor() const
{
    return m_minorContourColor;
}

void CextDbTin::setMinorContourColor(const AcCmColor& val)
{
    assertWriteEnabled();
    m_minorContourColor = val;
}

double CextDbTin::majorZ() const
{
    return m_majorZ;
}

void CextDbTin::setMajorZ(double val)
{
    assertWriteEnabled();
    m_majorZ = val;
}

double CextDbTin::minorZ() const
{
    return m_minorZ;
}

void CextDbTin::setMinorZ(double val)
{
    assertWriteEnabled();
    m_minorZ = val;
}

double CextDbTin::area2d() const
{
    return m_area2d;
}

double CextDbTin::area3d() const
{
    return m_area3d;
}

CextDbTin::DrawFlags CextDbTin::drawFlags() const
{
    return m_drawFlags;
}

void CextDbTin::setDrawFlags(DrawFlags val)
{
    assertWriteEnabled();
    m_drawFlags = val;
}

AcCmTransparency CextDbTin::pointTransparency() const
{
    return m_pointTransparency;
}

void CextDbTin::setPointTransparency(const AcCmTransparency& val)
{
    assertWriteEnabled();
    m_pointTransparency = val;
}

AcCmTransparency CextDbTin::tinTransparency() const
{
    return m_tinTransparency;
}

void CextDbTin::setTinTransparency(const AcCmTransparency& val)
{
    assertWriteEnabled();
    m_tinTransparency = val;
}

AcCmTransparency CextDbTin::minorTransparency() const
{
    return m_minorTransparency;
}

void CextDbTin::setMinorTransparency(const AcCmTransparency& val)
{
    assertWriteEnabled();
    m_minorTransparency = val;
}

AcCmTransparency CextDbTin::majorTransparency() const
{
    return m_majorTransparency;
}

void CextDbTin::setMajorTransparency(const AcCmTransparency& val)
{
    assertWriteEnabled();
    m_majorTransparency = val;
}

void CextDbTin::createTree()
{
    nanoflann::KDTreeSingleIndexAdaptorParams params;
    params.leaf_max_size = 16;
    params.n_thread_build = 0;
    m_pTree.reset(new kd_tree3d_t(2, m_adapter, params));
    m_pTree->buildIndex();
}
