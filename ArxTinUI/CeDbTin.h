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
//----- CeDbTin.h : Declaration of the CCeDbTin
//-----------------------------------------------------------------------------
#pragma once

#ifdef ARXTINUI_MODULE
#define DLLIMPEXP __declspec( dllexport )
#else

// Note: we don't use __declspec(dllimport) here, because of the
// "local vtable" problem with msvc.  If you use __declspec(dllimport),
// then, when a client dll does a new on the class, the object's
// vtable pointer points to a vtable allocated in that client
// dll.  If the client dll then passes the object to another dll,
// and the client dll is then unloaded, the vtable becomes invalid
// and any virtual calls on the object will access invalid memory.
//
// By not using __declspec(dllimport), we guarantee that the
// vtable is allocated in the server dll during the ctor and the
// client dll does not overwrite the vtable pointer after calling
// the ctor.  And, since we expect the server dll to remain in
// memory indefinitely, there is no problem with vtables unexpectedly
// going away.
// 
#define DLLIMPEXP
#endif

//-----------------------------------------------------------------------------
class DLLIMPEXP CextDbTin : public AcDbEntity
{

public:
    ACRX_DECLARE_MEMBERS(CextDbTin);

protected:
    static Adesk::UInt32 kCurrentVersionNumber;
public:
    enum class DrawFlags : int32_t
    {
        kDrawPoints = 1 << 0,
        kDrawTin = 1 << 1,
        kDrawContours = 1 << 2,
    };

    enum class TinFlags : int32_t
    {
        kNone = 0,
    };

public:
    CextDbTin();
    CextDbTin(const CePoints& points);
    virtual ~CextDbTin() override = default;

    virtual Acad::ErrorStatus dwgOutFields(AcDbDwgFiler* pFiler) const override;
    virtual Acad::ErrorStatus dwgInFields(AcDbDwgFiler* pFiler) override;

#ifdef _NEVER
    virtual Acad::ErrorStatus dxfOutFields(AcDbDxfFiler* pFiler) const override;
    virtual Acad::ErrorStatus dxfInFields(AcDbDxfFiler* pFiler) override;
#endif

    virtual Acad::ErrorStatus subOpen(AcDb::OpenMode mode) override;
    virtual Acad::ErrorStatus subErase(Adesk::Boolean erasing) override;
    virtual Acad::ErrorStatus subCancel() override;
    virtual Acad::ErrorStatus subClose() override;
    virtual void subList() const override;


protected:
    virtual Adesk::Boolean subWorldDraw(AcGiWorldDraw* mode) override;
    virtual Adesk::UInt32 subSetAttributes(AcGiDrawableTraits* traits) override;

public:
    virtual Acad::ErrorStatus  subGetOsnapPoints(
        AcDb::OsnapMode     osnapMode,
        Adesk::GsMarker     gsSelectionMark,
        const AcGePoint3d& pickPoint,
        const AcGePoint3d& lastPoint,
        const AcGeMatrix3d& viewXform,
        AcGePoint3dArray& snapPoints,
        AcDbIntArray& geomIds,
        const AcGeMatrix3d& insertionMat) const override;

    virtual Acad::ErrorStatus subTransformBy(const AcGeMatrix3d& xform) override;
    virtual Adesk::Boolean    subCloneMeForDragging() override;
    virtual Acad::ErrorStatus subExplode(AcDbVoidPtrArray& entitySet) const override;


public:
    Adesk::Boolean drawPoints(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;
    Adesk::Boolean drawTriangles(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;
    Adesk::Boolean drawContours(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;

    void        setPoints(const CePoints& points);
    void        recompute(bool force = false);
    void        createTree();
    void        computeTiangles();
    void        genMajorContours();
    void        genMinorContours();

    AcCmColor   pointColor() const;
    void        setpointColor(const AcCmColor& val);
    AcCmColor   tinColor() const;
    void        setTinColor(const AcCmColor& val);
    AcCmColor   majorContourColor() const;
    void        setMajorContourColor(const AcCmColor& val);
    AcCmColor   getMinorContourColor() const;
    void        setMinorContourColor(const AcCmColor& val);

    double      majorZ() const;
    void        setMajorZ(double val);
    double      minorZ() const;
    void        setMinorZ(double val);
    double      area2d() const;
    double      area3d() const;

    AcCmTransparency pointTransparency() const;
    void             setPointTransparency(const AcCmTransparency &val);
    AcCmTransparency tinTransparency() const;
    void             setTinTransparency(const AcCmTransparency& val);
    AcCmTransparency minorTransparency() const;
    void             setMinorTransparency(const AcCmTransparency& val);
    AcCmTransparency majorTransparency() const;
    void             setMajorTransparency(const AcCmTransparency& val);

    CextDbTin::DrawFlags drawFlags() const;
    void                 setDrawFlags(CextDbTin::DrawFlags val);

protected:
    //filed
    CePoints m_points;
    AcCmColor m_pointColor;
    AcCmColor m_tinColor;
    AcCmColor m_minorContourColor;
    AcCmColor m_majorContourColor;

    AcCmTransparency m_pointTransparency;
    AcCmTransparency m_tinTransparency;
    AcCmTransparency m_minorTransparency;
    AcCmTransparency m_majorTransparency;

    double m_majorZ = 0.0;
    double m_minorZ = 0.0;
    DrawFlags m_drawFlags = DrawFlags::kDrawTin;
    TinFlags m_tinFlags = TinFlags::kNone;

    //not filed
    std::mutex m_mtx_;
    CePolylines m_majorContours;
    CePolylines m_minorContours;
    CeTriangles m_triangles;
    KdAcGePointAdapter m_adapter{ m_points };
    std::shared_ptr<kd_tree3d_t> m_pTree;
    double m_zmin = std::numeric_limits<int64_t>::max();
    double m_zmax = std::numeric_limits<int64_t>::min();
    std::unordered_set<double> m_contourSet;

    double m_area2d = 0.0;
    double m_area3d = 0.0;

    bool m_dirty = false;
};

#ifdef ARXTINUI_MODULE
ACDB_REGISTER_OBJECT_ENTRY_AUTO(CextDbTin)
#endif
