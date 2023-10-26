/***************************************************************************
 *   Copyright (c) 2013 Thomas Anderson <blobfish[at]gmx.com>              *
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
#pragma once
#include <QtWidgets/QApplication>
#include <QLabel>
#include <QDialogButtonBox>

#include <gp_Vec.hxx>
#include <gp_Lin.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <BRepExtrema_DistShapeShape.hxx>

#include <Inventor/fields/SoSFVec3f.h>
#include <Inventor/fields/SoSFMatrix.h>
#include <Inventor/fields/SoSFString.h>
#include <Inventor/nodekits/SoSeparatorKit.h>
#include <Inventor/fields/SoSFColor.h>
#include <Inventor/fields/SoSFRotation.h>
#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/engines/SoSubEngine.h>
#include <Inventor/engines/SoEngine.h>

#include <Open3DViewer\DLLConfig.h>
#include <Open3DViewer/View3DInventor.h>
#include <Open3DViewer/Selection.h>
#include <Open3DViewer/ViewProvider.h>

class DLLExport AbstractButton : public QObject
{
	Q_OBJECT
public:
	AbstractButton(QObject* parent);
	~AbstractButton() {};
	void setToggle(bool);
public:
	Q_SIGNAL
		void toggled(bool);
private:
	bool _trigger;
};

class DLLExport DimensionLinearEvent : public QEvent
{
public:
	DimensionLinearEvent(const Base::Type& s);
	~DimensionLinearEvent() {};
	const Base::Type& style() const;
private:
	Base::Type t;
};

class DLLExport SteppedSelection : public QWidget
{
	Q_OBJECT
public:
	SteppedSelection(const uint &buttonCountIn, QWidget *parent = 0);
	~SteppedSelection();
	AbstractButton* getButton(const uint &index);

protected:
	typedef std::pair<AbstractButton *, QLabel *> ButtonIconPairType;
	std::vector<ButtonIconPairType> buttons;

private Q_SLOTS:
	void selectionSlot(bool checked);
};

class DLLExport DimSelections
{
public:
	enum ShapeType { None, Vertex, Edge, Face };
	struct DimSelection
	{
		ViewProvider* vp;
		std::string subObjectName;
		float x;
		float y;
		float z;
		ShapeType shapeType;
	};
	std::vector<DimSelection> selections;
};

class DLLExport TaskMeasure
{
public:
	TaskMeasure() {};
	View3DInventor* getInventor();
	void eraseAllDimensions();
};

class DLLExport DimensionControl : public QWidget, public TaskMeasure
{
	Q_OBJECT
public:
	explicit DimensionControl(QWidget* parent);
	QPushButton *resetButton;
public Q_SLOTS:
	void toggle3dSlot(bool);
	void clearAllSlot(bool);
};

class DLLExport TaskMeasureLinear : public QObject, public SelectionObserver, public TaskMeasure
{
	Q_OBJECT
public:
	TaskMeasureLinear();
	~TaskMeasureLinear();
	void buildDimension(const DimSelections &sel1, const DimSelections &sel2);
	void addLinearDimensions(const BRepExtrema_DistShapeShape &measure);
	void addLinearDimensions(const gp_Pnt p1, const gp_Pnt p2);
protected:
	virtual void onSelectionChanged(const SelectionChanges& msg);

protected Q_SLOTS:
	void selection1Slot(bool checked);
	void selection2Slot(bool checked);
	void toggle3dSlot(bool);
	void selectionClearDelayedSlot();
public Q_SLOT:
	void clearAllSlot(bool);

protected:
	uint buttonSelectedIndex;
	DimSelections selections1;
	DimSelections selections2;
	SteppedSelection *stepped;
	void buildDimension();
	void clearSelectionStrings();
	void goDimensionLinearNoTask(const TopoDS_Shape &shape1, const TopoDS_Shape &shape2);
};

class DLLExport DimensionLinear : public SoSeparatorKit
{
	SO_KIT_HEADER(DimensionLinear);

	SO_KIT_CATALOG_ENTRY_HEADER(transformation);
	SO_KIT_CATALOG_ENTRY_HEADER(annotate);
	SO_KIT_CATALOG_ENTRY_HEADER(leftArrow);
	SO_KIT_CATALOG_ENTRY_HEADER(rightArrow);
	SO_KIT_CATALOG_ENTRY_HEADER(line);
	SO_KIT_CATALOG_ENTRY_HEADER(textSep);
public:
	DimensionLinear();
	static void initClass();
	virtual SbBool affectsState() const;
	void setupDimension();
	void drawAxis(const SbColor color);

	SoSFVec3f point1;
	SoSFVec3f point2;
	SoSFString text;
	SoSFColor dColor;
protected:
	SoSFRotation rotate;
	SoSFFloat length;
	SoSFVec3f origin;

private:
	virtual ~DimensionLinear();
};

class VectorAdapter
{
public:
	/*!default construction isValid is set to false*/
	VectorAdapter();
	/*!Build a vector from a faceIn
	 * @param faceIn vector will be normal to plane and equal to cylindrical axis.
	 * @param pickedPointIn location of pick. straight conversion from sbvec. not accurate.*/
	VectorAdapter(const TopoDS_Face &faceIn, const gp_Vec &pickedPointIn);
	/*!Build a vector from an edgeIn
	 * @param edgeIn vector will be lastPoint - firstPoint.
	 * @param pickedPointIn location of pick. straight conversion from sbvec. not accurate.*/
	VectorAdapter(const TopoDS_Edge &edgeIn, const gp_Vec &pickedPointIn);
	/*!Build a vector From 2 vertices.
	 *vector will be equal to @param vertex2In - @param vertex1In.*/
	VectorAdapter(const TopoDS_Vertex &vertex1In, const TopoDS_Vertex &vertex2In);
	/*!Build a vector From 2 vectors.
	 *vector will be equal to @param vector2 - @param vector1.*/
	VectorAdapter(const gp_Vec &vector1, const gp_Vec &vector2);

	/*!make sure no errors in vector construction.
	 * @return true = vector is good. false = vector is NOT good.*/
	bool isValid() const { return status; }
	/*!get the calculated vector.
	 * @return the vector. use isValid to ensure correct results.*/
	operator gp_Vec() const { return vector; }
	/*!build occ line used for extrema calculation*/
	operator gp_Lin() const;
	gp_Vec getPickPoint() const { return origin; }

private:
	void projectOriginOntoVector(const gp_Vec &pickedPointIn);
	bool status;
	gp_Vec vector;
	gp_Vec origin;
};

class DLLExport DimensionAngular : public SoSeparatorKit
{
	SO_KIT_HEADER(DimensionAngular);

	SO_KIT_CATALOG_ENTRY_HEADER(transformation);
	SO_KIT_CATALOG_ENTRY_HEADER(annotate);
	SO_KIT_CATALOG_ENTRY_HEADER(arrow1);
	SO_KIT_CATALOG_ENTRY_HEADER(arrow2);
	SO_KIT_CATALOG_ENTRY_HEADER(arcSep);
	SO_KIT_CATALOG_ENTRY_HEADER(textSep);
public:
	DimensionAngular();
	static void initClass();
	virtual SbBool affectsState() const;

	SoSFFloat radius;//radians.
	SoSFFloat angle;//radians.
	SoSFString text;
	SoSFColor dColor;
	SoSFMatrix matrix;
	void setupDimension();
	void setupDimension(gp_Trsf picked1, gp_Trsf picked2, SbVec3f origin);

private:
	virtual ~DimensionAngular();
};

class DLLExport ArcEngine : public SoEngine
{
	SO_ENGINE_HEADER(ArcEngine);
public:
	ArcEngine();
	static void initClass();

	SoSFFloat radius;
	SoSFFloat angle;
	SoSFFloat deviation;

	SoEngineOutput points;
	SoEngineOutput pointCount;
protected:
	virtual void evaluate();
private:
	virtual ~ArcEngine() {}
	void defaultValues(); //some non error values if something goes wrong.
};

//角度测量根类
class DLLExport TaskMeasureAngular : public QObject, public SelectionObserver, public TaskMeasure
{
	Q_OBJECT
public:
	TaskMeasureAngular();
	~TaskMeasureAngular();

	virtual QDialogButtonBox::StandardButtons getStandardButtons() const;
	virtual bool isAllowedAlterDocument(void) const;
	virtual bool needsFullSpace() const;
protected:
	virtual void onSelectionChanged(const SelectionChanges& msg);

protected Q_SLOTS:
	void selection1Slot(bool checked);
	void selection2Slot(bool checked);
	void resetDialogSlot(bool);
	void toggle3dSlot(bool);
	void toggleDeltaSlot(bool);
	void clearAllSlot(bool);
	void selectionClearDelayedSlot();

public:
	void buildDimension(const DimSelections &sel1, const DimSelections &sel2);

protected:
	void buildDimension();
	void setUpGui();
	void clearSelection();
	DimSelections selections1;
	DimSelections selections2;
	uint buttonSelectedIndex;
	SteppedSelection *stepped;
	static VectorAdapter buildAdapter(const DimSelections &selection);
	void goDimensionAngularNoTask(const VectorAdapter &vector1Adapter, const VectorAdapter &vector2Adapter);
	void goDimensionAngularNoTask_new(const VectorAdapter &vector1Adapter, const VectorAdapter &vector2Adapter, gp_Pnt arcOrigin, double trueAngle);
};

/*!start of the measure angular command*/
void goDimensionAngularRoot();
/*!examine angular pre selection
  * @param vector1Out first shape in current selection
  * @param vector2Out second shape in current selection
  * @return signal if preselection is valid. false means vector1Out and vector2Out are invalid.
  */
bool evaluateAngularPreSelection(VectorAdapter &vector1Out, VectorAdapter &vector2Out);
/*!build angular dimension*/
void goDimensionAngularNoTask(const VectorAdapter &vector1Adapter, const VectorAdapter &vector2Adapter);

//测量面积根类
class DLLExport TaskMeasureArea : public QObject, public SelectionObserver, public TaskMeasure
{
	Q_OBJECT
public:
	TaskMeasureArea();
	~TaskMeasureArea();
protected:
	virtual void onSelectionChanged(const SelectionChanges& msg);
protected:
	void buildDimension(TopoDS_Shape shape, Vector3d pickedPoint);
	void addAreaDimension();
	DimSelections selection;
	AbstractButton* button;
};

class DLLExport DimensionArea : public SoSeparatorKit
{
	SO_KIT_HEADER(DimensionArea);

	SO_KIT_CATALOG_ENTRY_HEADER(transformation);
	SO_KIT_CATALOG_ENTRY_HEADER(annotate);
	SO_KIT_CATALOG_ENTRY_HEADER(Arrow);
	SO_KIT_CATALOG_ENTRY_HEADER(line);
	SO_KIT_CATALOG_ENTRY_HEADER(textSep);
public:
	DimensionArea();
	static void initClass();
	virtual SbBool affectsState() const;
	void setupDimension();

	SoSFFloat area;
	SoSFVec3f point1;
	SoSFVec3f point2;
	SoSFString text;
	SoSFColor dColor;
protected:
	SoSFRotation rotate;
	SoSFFloat length;
	SoSFVec3f origin;

private:
	virtual ~DimensionArea();
};

class DLLExport TaskMeasureRadius : public QObject, public SelectionObserver, public TaskMeasure
{
	Q_OBJECT

public:
	TaskMeasureRadius();
	~TaskMeasureRadius();
protected:
	virtual void onSelectionChanged(const SelectionChanges& msg);
protected:
	void buildDimension(const gp_Pnt o, const gp_Pnt pnt, const Standard_Real R);
};