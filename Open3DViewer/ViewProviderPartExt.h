/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <juergen.riegel@web.de>             *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef VIEWPROVIDERPARTEXT_H
#define VIEWPROVIDERPARTEXT_H

#include <Standard_math.hxx>
#include <Standard_Boolean.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoIndexedPointSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>

#include <Open3DViewer\DLLConfig.h>
#include <Open3DViewer/SoBrepPointSet.h>
#include <Open3DViewer/SoBrepEdgeSet.h>
#include <Open3DViewer/SoBrepFaceSet.h>
#include <Open3DViewer/ViewProvider.h>
#include <Open3DViewer/Material.h>

using namespace App;

class TopoDS_Shape;
class TopoDS_Edge;
class TopoDS_Wire;
class TopoDS_Face;
class SoSeparator;
class SoGroup;
class SoSwitch;
class SoVertexShape;
class SoPickedPoint;
class SoShapeHints;
class SoEventCallback;
class SbVec3f;
class SoSphere;
class SoScale;
class SoCoordinate3;
class SoIndexedFaceSet;
class SoNormal;
class SoNormalBinding;
class SoMaterialBinding;
class SoIndexedLineSet;


class DLLExport ViewProviderPartExt : public ViewProvider
{
public:
	/// constructor
	ViewProviderPartExt();
	/// destructor
	virtual ~ViewProviderPartExt();
	void updateVisual();
	void newUpdateVisual();
	void setShape(TopoDS_Shape shape);
	TopoDS_Shape getShape() const;
	virtual std::string getElement(const SoDetail* detail) const;
	bool getElementPicked(const SoPickedPoint *pp, std::string &subname) const;
	virtual SoDetail* getDetail(const char*) const override;
	//range: 0~255
	virtual void setShapeColor(double r, double g, double b);
	void clearColor();
	//range: 0~255
	void clearColor(double r, double g, double b);
	void applyColors(std::vector<Color> colors);
	void setHighlightedFaces(const std::vector<App::Color>& colors);
	const std::vector<Color> getDiffuseColor() const;
	const Color getGlobeColor();

protected:
	void getNormals(const TopoDS_Face&  theFace, const Handle(Poly_Triangulation)& aPolyTri,
		TColgp_Array1OfDir& theNormals);

public:
	double	Deviation;
	bool	ControlPoints;
	double	AngularDeflection;
	double	PointSize;
	double	LineWidth;
protected:
	SoMaterialBinding * pcFaceBind;
	SoMaterialBinding * pcLineBind;
	SoMaterialBinding * pcPointBind;
	SoMaterial        * pcLineMaterial;
	SoMaterial        * pcPointMaterial;
	SoDrawStyle       * pcLineStyle;
	SoDrawStyle       * pcPointStyle;
	SoShapeHints      * pShapeHints;


	SoCoordinate3     * coords;
	SoBrepFaceSet     * faceset;
	SoNormal          * norm;
	SoNormalBinding   * normb;
	SoBrepEdgeSet     * lineset;
	SoBrepPointSet    * nodeset;
	SoMaterial       * pcShapeMaterial;

	bool VisualTouched;
	bool NormalsFromUV;

	std::vector<Color> DiffuseColor;
	Color globeColor;

private:
	TopoDS_Shape _shape;
};

#endif // !VIEWPROVIDERPARTEXT_H