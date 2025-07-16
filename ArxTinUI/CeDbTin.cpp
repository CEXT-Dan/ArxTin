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
    CEXTDBTIN
    | Product Desc : A TIN for CAD
    | Company : CadExt
    | WEB Address : www.cadext.com
)

//-----------------------------------------------------------------------------
CextDbTin::CextDbTin() : AcDbEntity()
{
}

CextDbTin::CextDbTin(const CePoints & points)
    : AcDbEntity(), m_points(points)
{

}

//-----------------------------------------------------------------------------
//----- AcDbObject protocols
//- Dwg Filing protocol
Acad::ErrorStatus CextDbTin::dwgOutFields(AcDbDwgFiler * pFiler) const
{
    assertReadEnabled();
    Acad::ErrorStatus es = AcDbEntity::dwgOutFields(pFiler);
    if (es != Acad::eOk)
        return (es);
    if ((es = pFiler->writeUInt32(CextDbTin::kCurrentVersionNumber)) != Acad::eOk)
        return (es);
    if (es = pFiler->writeInt64(Adesk::Int64(m_points.size())); es != eOk)
        return (es);
    for (const auto& p : m_points)
    {
        if (es = pFiler->writePoint3d(p); es != eOk)
            return es;
    }
    return (pFiler->filerStatus());
}

Acad::ErrorStatus CextDbTin::dwgInFields(AcDbDwgFiler * pFiler)
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

    Adesk::Int64 npoints = 0;
    if (es = pFiler->readInt64(&npoints); es != eOk)
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

Acad::ErrorStatus CextDbTin::dxfOutFields(AcDbDxfFiler * pFiler) const
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

Acad::ErrorStatus CextDbTin::dxfInFields(AcDbDxfFiler * pFiler) {
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

//-----------------------------------------------------------------------------
//----- AcDbEntity protocols
Adesk::Boolean CextDbTin::subWorldDraw(AcGiWorldDraw * mode)
{
    assertReadEnabled();
    auto& rTraits = mode->subEntityTraits();
    auto& rGeo = mode->geometry();

    computeTiangles();
    genCountours();

    drawPoints(rTraits, rGeo);
    drawTriangles(rTraits, rGeo);
    drawContours(rTraits, rGeo);

    return true;
}

Adesk::UInt32 CextDbTin::subSetAttributes(AcGiDrawableTraits * traits) {
    assertReadEnabled();
    return (AcDbEntity::subSetAttributes(traits));
}

Acad::ErrorStatus CextDbTin::subGetOsnapPoints(
    AcDb::OsnapMode osnapMode,
    Adesk::GsMarker gsSelectionMark,
    const AcGePoint3d & pickPoint,
    const AcGePoint3d & lastPoint,
    const AcGeMatrix3d & viewXform,
    AcGePoint3dArray & snapPoints,
    AcDbIntArray & geomIds,
    const AcGeMatrix3d & insertionMat) const
{
    assertReadEnabled();
    if (osnapMode != AcDb::kOsModeEnd)
        return eOk;
    for (const auto& p : m_points)
        snapPoints.append(p);
    return eOk;
}

Acad::ErrorStatus CextDbTin::subTransformBy(const AcGeMatrix3d & xform)
{
    assertWriteEnabled();
    std::for_each(std::execution::par, m_points.begin(), m_points.end(), [&](AcGePoint3d& p) { p.transformBy(xform); });
    return xDataTransformBy(xform);
}

Adesk::Boolean CextDbTin::subCloneMeForDragging()
{
    return false; //TODO test
}

//-----------------------------------------------------------------------------
//----- Not Autodesk
void CextDbTin::setPoints(const CePoints & points)
{
    assertWriteEnabled();
    m_points = points;
    m_dirty = true;
    recompute();
}

void CextDbTin::recompute()
{
    if (m_dirty)
    {
        computeTiangles();
        genCountours();
        createTree();
        m_dirty = false;
    }
}

static double roundToNearest(double value, double multiple)
{
    double epsilon = std::numeric_limits<double>::epsilon() * multiple;
    double adj = std::signbit(value) ? -epsilon : epsilon;
    return std::round((value + adj) / multiple) * multiple;
}

static bool isMultiple(double a, double b) {
    if (b == 0)
        return false;
    return (int64_t(a) % int64_t(b) == 0);
}

Adesk::Boolean CextDbTin::drawPoints(AcGiSubEntityTraits & traits, AcGiWorldGeometry & geo) const
{
    traits.setColor(6);
    if (m_drawPoints)
    {
        geo.polypoint(m_points.size(), m_points.data());
    }
    return Adesk::kTrue;
}

Adesk::Boolean CextDbTin::drawTriangles(AcGiSubEntityTraits & traits, AcGiWorldGeometry & geo) const
{
    traits.setColor(139);
    if (m_drawTin)
    {
        for (const auto& tri : m_triangles)
        {
            geo.polygon(tri.size(), tri.data());
        }
    }
    return Adesk::kTrue;
}

Adesk::Boolean CextDbTin::drawContours(AcGiSubEntityTraits & traits, AcGiWorldGeometry & geo) const
{
    if (m_drawContours)
    {
        for (const auto& pline : m_plines)
        {
            if (pline.size())
            {
                if (isMultiple(pline[0].z, 50))
                    traits.setColor(3);
                if (isMultiple(pline[0].z, 100))
                    traits.setColor(1);
                geo.polyline(pline.size(), pline.data());
            }
        }
    }
    return Adesk::kTrue;
}

static auto connectSegmentsIntoPolylines(const CeSegments & segments) -> CePolylines
{
    // Map from point to all segments starting or ending at that point
    std::unordered_multimap<AcGePoint3d, const CeSegment*, Point3DHash> pointToSegs;
    for (const auto& seg : segments)
    {
        pointToSegs.emplace(seg.first, &seg);
        pointToSegs.emplace(seg.second, &seg);
    }

    std::unordered_set<const CeSegment*, SegmentPtrHash> visited;
    CePolylines polylines;

    for (const auto& seg : segments)
    {
        if (visited.count(&seg))
            continue;

        CePolyline polyline;
        polyline.push_back(seg.first);
        polyline.push_back(seg.second);
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
                if (visited.count(nextSeg))
                    continue;
                // Find the next point to extend
                AcGePoint3d nextPoint;
                {
                    if (current.isEqualTo(nextSeg->first))
                        nextPoint = nextSeg->second;
                    else if (current.isEqualTo(nextSeg->second))
                        nextPoint = nextSeg->first;
                    else
                        continue;
                }
                polyline.push_back(nextPoint);
                current = nextPoint;
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
            for (auto it = range.first; it != range.second; ++it) {
                const CeSegment* prevSeg = it->second;
                if (visited.count(prevSeg))
                    continue;
                AcGePoint3d prevPoint;
                {
                    if (current.isEqualTo(prevSeg->first))
                        prevPoint = prevSeg->second;
                    else if (current.isEqualTo(prevSeg->second))
                        prevPoint = prevSeg->first;
                    else
                        continue;
                }
                polyline.insert(polyline.begin(), prevPoint);
                current = prevPoint;
                visited.insert(prevSeg);
                extended = true;
                break;
            }
            if (!extended)
                break;
        }
        polylines.push_back(polyline);
    }
    return polylines;
}

static auto interpolate(const AcGePoint3d & a, const AcGePoint3d & b, double contourLevel) -> AcGePoint3d
{
    const double t = (contourLevel - a.z) / (b.z - a.z);
    return
    {
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        contourLevel
    };
}

static void processTriangle(const CeTriangle & tri, double contourLevel, CeSegments & segments)
{
    CePoints crossings;
    for (int i = 0; i < 3; ++i)
    {
        const AcGePoint3d& a = tri[i];
        const AcGePoint3d& b = tri[(i + 1) % 3];
        if ((a.z < contourLevel && b.z > contourLevel) ||
            (a.z > contourLevel && b.z < contourLevel))
        {
            crossings.push_back(interpolate(a, b, contourLevel));
        }
    }
    if (crossings.size() == 2)
        segments.emplace_back(crossings[0], crossings[1]);
}

static auto generateContours(const CeTriangles & triangles, const CeContourLevels & contourLevels) -> CeSegments
{
    CeSegments segments;
    for (const auto& tri : triangles)
    {
        for (double level : contourLevels)
            processTriangle(tri, level, segments);
    }
    return segments;
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

void CextDbTin::computeTiangles()
{
    m_triangles.clear();
    CeCoords outCoords;
    m_zmin = std::numeric_limits<int64_t>::max();
    m_zmax = std::numeric_limits<int64_t>::min();
    getCoords(m_points, outCoords, m_zmin, m_zmax);
    delaunator::Delaunator d(outCoords);

    for (size_t i = 0; i < d.triangles.size(); i += 3)
    {
        CeTriangle tri;
        const size_t a = d.triangles[i + 0];
        const size_t b = d.triangles[i + 1];
        const size_t c = d.triangles[i + 2];
        tri[0].set(m_points[a].x, m_points[a].y, m_points[a].z);
        tri[1].set(m_points[b].x, m_points[b].y, m_points[b].z);
        tri[2].set(m_points[c].x, m_points[c].y, m_points[c].z);
        m_triangles.push_back(tri);
    }
}

void CextDbTin::genCountours()
{
    m_plines.clear();
    CeContourLevels contourLevels;
    for (int64_t idx = int64_t(m_zmin); idx < int64_t(m_zmax); idx += 50)
    {
        contourLevels.push_back(roundToNearest(idx, 50));
    }
    auto contours = generateContours(m_triangles, contourLevels);
    m_plines = connectSegmentsIntoPolylines(contours);
}

void CextDbTin::createTree()
{
    nanoflann::KDTreeSingleIndexAdaptorParams params;
    params.leaf_max_size = 16;
    params.n_thread_build = 0;
    m_pTree.reset(new kd_tree3d_t(3, m_adapter, params));
    m_pTree->buildIndex();
}
