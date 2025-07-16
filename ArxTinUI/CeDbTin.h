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
    CextDbTin();
    CextDbTin(const CePoints& points);
    virtual ~CextDbTin() override = default;

    //----- AcDbObject protocols
    //- Dwg Filing protocol
    virtual Acad::ErrorStatus dwgOutFields(AcDbDwgFiler* pFiler) const override;
    virtual Acad::ErrorStatus dwgInFields(AcDbDwgFiler* pFiler) override;

    //- Dxf Filing protocol
    virtual Acad::ErrorStatus dxfOutFields(AcDbDxfFiler* pFiler) const;
    virtual Acad::ErrorStatus dxfInFields(AcDbDxfFiler* pFiler);

    //- SubXXX() methods (self notification)
    virtual Acad::ErrorStatus subOpen(AcDb::OpenMode mode);
    virtual Acad::ErrorStatus subErase(Adesk::Boolean erasing);
    virtual Acad::ErrorStatus subCancel();
    virtual Acad::ErrorStatus subClose();

    //----- AcDbEntity protocols
    //- Graphics protocol
protected:
    virtual Adesk::Boolean subWorldDraw(AcGiWorldDraw* mode);
    virtual Adesk::UInt32 subSetAttributes(AcGiDrawableTraits* traits);

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

public:
    Adesk::Boolean drawPoints(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;
    Adesk::Boolean drawTriangles(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;
    Adesk::Boolean drawContours(AcGiSubEntityTraits& traits, AcGiWorldGeometry& geo) const;

    void setPoints(const CePoints& points);
    void recompute();
    void createTree();
    void computeTiangles();
    void genCountours();


protected:
    CePoints m_points;
    CePolylines m_plines;
    CeTriangles m_triangles;
    KdAcGePointAdapter m_adapter{ m_points };
    std::shared_ptr<kd_tree3d_t> m_pTree;
    double m_zmin = std::numeric_limits<int64_t>::max();
    double m_zmax = std::numeric_limits<int64_t>::min();

    //major z
    //major clr

    //minor z
    //minor clr

    //pnt color
    //tin color
    //contour color

    bool m_drawPoints = true;
    bool m_drawTin = true;
    bool m_drawContours = true;
    bool m_dirty = false;
};

#ifdef ARXTINUI_MODULE
ACDB_REGISTER_OBJECT_ENTRY_AUTO(CextDbTin)
#endif
