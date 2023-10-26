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

#include <boost/algorithm/string/predicate.hpp>
#include <boost_bind_bind.hpp>

#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/engines/SoCalculator.h>
#include <Inventor/engines/SoComposeRotationFromTo.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/engines/SoComposeVec3f.h>
#include <Inventor/engines/SoConcatenate.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoResetTransform.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/engines/SoComposeRotation.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoLineSet.h>

#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopExp.hxx>
#include <BRepExtrema_ExtCC.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Line.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Plane.hxx>
#include <GeomAPI_IntSS.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Quaternion.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepGProp_Face.hxx>

#include <QPushButton>
#include <QTimer>

#include <Open3DViewer\Measure.h>
#include <Open3DViewer\ViewProviderPartExt.h>
#include <Open3DViewer\Material.h>
#include <Open3DViewer\View3DInventor.h>
#include <Open3DViewer\MainViewer.h>


static bool _MeasureInfoInited;

bool visibility = true;

static const char * cursor_ruler[] = {
"32 32 3 1",
" 	c None",
".	c #FF0000",
"+	c #000000",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"                                ",
"..... . .....                   ",
"                                ",
"      .                         ",
"      .                         ",
"      .        ++               ",
"      .       +  +              ",
"      .      +   ++             ",
"            +   +  +            ",
"           +   +    +           ",
"          +   +     ++          ",
"          +        +  +         ",
"           +           +        ",
"            +         + +       ",
"             +       +   +      ",
"              +           +     ",
"               +         + +    ",
"                +       +   +   ",
"                 +           +  ",
"                  +         + + ",
"                   +       +  ++",
"                    +     +   + ",
"                     +       +  ",
"                      +     +   ",
"                       +   +    ",
"                        + +     ",
"                         +      " };

static const char* cursor_angular[]{
"32 32 3 1",
" 	c None",
".	c #FF0000",
"+	c #000000",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"                                ",
"..... . .....                   ",
"                         ++     ",
"      .                ++       ",
"      .              ++         ",
"      .            ++           ",
"      .          +++            ",
"      .        ++  ++           ",
"             ++     ++          ",
"           ++        ++         ",
"         ++           ++        ",
"          ++           ++       ",
"           ++          ++       ",
"            ++         ++       ",
"             ++        ++       ",
"              ++      ++        ",
"               ++    ++         ",
"                ++  ++          ",
"                 ++++           ",
"                  ++            ",
"                   ++           ",
"                    ++          ",
"                     ++         ",
"                      ++        ",
"                       ++       ",
"                        ++      ",
"                         ++     " };

static const char* cursor_area[]{
"32 32 3 1",
" 	c None",
".	c #FF0000",
"+	c #000000",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"                                ",
"..... . .....                   ",
"                                ",
"      .                         ",
"      .    +++++++++++++++++    ",
"      .   +               ++    ",
"      .  +               + +    ",
"      . +               +  +    ",
"       +++++++++++++++++   +    ",
"       + +   +   +   + +   +    ",
"       +  +   +   +   ++   +    ",
"       +   +   +   +   +   +    ",
"       ++   +   +   +  +   +    ",
"       + +   +   +   + +   +    ",
"       +  +   +   +   ++   +    ",
"       +   +   +   +   +   +    ",
"       ++   +   +   +  +   +    ",
"       + +   +   +   + +   +    ",
"       +  +   +   +   ++   +    ",
"       +   +   +   +   +  +     ",
"       ++   +   +   +  + +      ",
"       + +   +   +   + ++       ",
"       +++++++++++++++++        ",
"                                ",
"                                ",
"                                ",
"                                " };

static const char* cursor_radius[]{
"32 32 3 1",
" 	c None",
".	c #FF0000",
"+	c #000000",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"      .                         ",
"                                ",
"..... . .....                   ",
"                                ",
"      .                         ",
"      .         ++++++          ",
"      .       ++      ++        ",
"      .     ++          ++      ",
"      .    +              +     ",
"           +              +     ",
"          +                +    ",
"          +                +    ",
"         +                  +   ",
"         +                  +   ",
"         +        +++++++++++   ",
"         +        +++++++++++   ",
"         +                  +   ",
"         +                  +   ",
"          +                +    ",
"          +                +    ",
"           +              +     ",
"           +              +     ",
"            ++          ++      ",
"              ++      ++        ",
"                ++++++          ",
"                                ",
"                                ",
"                                " };

AbstractButton::AbstractButton(QObject* parent)
	:QObject(parent), _trigger(false)
{

}

void AbstractButton::setToggle(bool is)
{
	_trigger = is;
	toggled(is);
}

//将value保留pos位小数,输出字符串
std::string keepDecimal(double value ,uint pos) 
{
	if (value < 0) value = -value;
	int multiPos = 10;
	for (uint i = 1; i < pos; i++)
		multiPos = multiPos * 10;
	if (pos != 0)
		value = value * multiPos;
	int intValue = (int)value;
	double delta = value - (double)intValue;
	if (delta * 10 >= 5) intValue += 1;
	std::string str = std::to_string(intValue);
	if (pos != 0)
	{
		for (int idx = str.size() - pos; idx < str.size(); idx++) {
			if (str[idx] != '0') {
				str.insert(str.size() - pos, 1, '.');
				return str;
			}
		}
		do {
			str.pop_back();
			pos--;
		} while (pos > 0);
	}
	return str;
}

/*
将值四舍五入转为无符号整型带单位字符串结果
value:输出数值
type:单位符号,0(毫米)、1(度)、2(平方毫米)
*/
QString getPrintValue(double value, int type)
{
	std::string res = keepDecimal(value, 2);
	QString result = QString::fromStdString(res);
	switch (type)
	{
	case 0:result += "mm"; break;
	case 1:result += char(176); break;
	case 2: {
		if (value > 100000)
		{
			result.clear();
			result += QString::fromStdString(keepDecimal(value / 1000000.0, 6));
			result += "m";
			char16_t square = 0xB2;
			result += QString::fromUtf16(&square, 1);
			result.push_front('0');
		}
		else 
		{
			result += "mm";
			char16_t square = 0xB2;
			result += QString::fromUtf16(&square, 1);
		}
		break;
	}
	default:break;
	}
	return result;
}

struct MeasureInfo {
	DimSelections sel1;
	DimSelections sel2;
	bool linear;
	MeasureInfo(const DimSelections &sel1, const DimSelections &sel2, bool linear)
		:sel1(sel1), sel2(sel2), linear(linear)
	{
		if (!_MeasureInfoInited) {
			_MeasureInfoInited = true;
		}
	}
};
static std::list<MeasureInfo> _Measures;

//获取shape子元素(Vertex Edge Face)
static std::array<std::string, TopAbs_SHAPE> _ShapeNames;
static void initShapeNameMap() {
	if (_ShapeNames[TopAbs_VERTEX].empty()) {
		_ShapeNames[TopAbs_VERTEX] = "Vertex";
		_ShapeNames[TopAbs_EDGE] = "Edge";
		_ShapeNames[TopAbs_FACE] = "Face";
		_ShapeNames[TopAbs_WIRE] = "Wire";
		_ShapeNames[TopAbs_SHELL] = "Shell";
		_ShapeNames[TopAbs_SOLID] = "Solid";
		_ShapeNames[TopAbs_COMPOUND] = "Compound";
		_ShapeNames[TopAbs_COMPSOLID] = "CompSolid";
	}
}

TopAbs_ShapeEnum shapeType(const char *type, bool silent) {
	if (type) {
		initShapeNameMap();
		for (size_t idx = 0; idx < _ShapeNames.size(); ++idx) {
			if (_ShapeNames[idx].size() && boost::starts_with(type, _ShapeNames[idx]))
				return (TopAbs_ShapeEnum)idx;
		}
	}
	return TopAbs_SHAPE;
}

const std::string &shapeName(TopAbs_ShapeEnum type) {
	initShapeNameMap();
	if (type >= 0 && type < _ShapeNames.size() && _ShapeNames[type].size())
		return _ShapeNames[type];
	static std::string ret("");
	return ret;
}

std::pair<TopAbs_ShapeEnum, int> shapeTypeAndIndex(const char *name) {
	int idx = 0;
	TopAbs_ShapeEnum type = TopAbs_SHAPE;
	static const std::string _subshape("SubShape");
	if (boost::starts_with(name, _subshape)) {
		std::istringstream iss(name + _subshape.size());
		iss >> idx;
		if (!iss.eof())
			idx = 0;
	}
	else {
		type = shapeType(name, true);
		if (type != TopAbs_SHAPE) {
			std::istringstream iss(name + shapeName(type).size());
			iss >> idx;
			if (!iss.eof()) {
				idx = 0;
				type = TopAbs_SHAPE;
			}
		}
	}
	return std::make_pair(type, idx);
}

TopoDS_Shape getSubElementShape(TopoDS_Shape parentShape, const char* subName)
{
	if (parentShape.IsNull()) return TopoDS_Shape();
	auto result = shapeTypeAndIndex(subName);
	auto type = result.first;
	auto index = result.second;
	try {
		if (type == TopAbs_SHAPE) {
			int i = 1;
			for (TopoDS_Iterator it(parentShape); it.More(); it.Next(), ++i) {
				if (i == index)
					return it.Value();
			}
		}
		else {
			TopTools_IndexedMapOfShape anIndices;
			TopExp::MapShapes(parentShape, type, anIndices);
			if (index <= anIndices.Extent())
				return anIndices.FindKey(index);
		}
	}
	catch (...) { }
	return TopoDS_Shape();
}

SoNode* createLinearDimension(const gp_Pnt &point1, const gp_Pnt &point2, const SbColor &color)
{
	SbVec3f vec1(point1.X(), point1.Y(), point1.Z());
	SbVec3f vec2(point2.X(), point2.Y(), point2.Z());
	if ((vec2 - vec1).length() < FLT_EPSILON)
		return new SoSeparator(); //empty object.
	DimensionLinear *dimension = new DimensionLinear();
	auto pickS = (SoPickStyle*)dimension->getPart("pickStyle", true);
	if (pickS) { 
		pickS->style.setValue(SoPickStyle::Style::UNPICKABLE); 
	}
	dimension->point1.setValue(vec1);
	dimension->point2.setValue(vec2);
	dimension->setupDimension();

	//写入测量值
	dimension->text.setValue(getPrintValue(static_cast<double>((vec2 - vec1).length()), 0).toStdString().c_str());
	dimension->dColor.setValue(color);
	return dimension;
}

void TaskMeasureLinear::addLinearDimensions(const BRepExtrema_DistShapeShape &measure)
{
	App::Color c(1.0, 0.0, 0.0);
	App::Color d(0.0, 1.0, 0.0);

	View3DInventor* viewer = getInventor();
	
	if (!viewer)
		return;
	gp_Pnt point1 = measure.PointOnShape1(1);
	gp_Pnt point2 = measure.PointOnShape2(1);
	viewer->addDimension3d(createLinearDimension(point1, point2, SbColor(c.r, c.g, c.b)));

	//create deltas. point1 will always be the same.
	gp_Pnt temp = point1;
	gp_Pnt lastTemp = temp;
	temp.SetX(point2.X());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
	lastTemp = temp;
	temp.SetY(point2.Y());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
	lastTemp = temp;
	temp.SetZ(point2.Z());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
}

void TaskMeasureLinear::addLinearDimensions(const gp_Pnt p1, const gp_Pnt p2)
{
	App::Color c(1.0, 0.0, 0.0);
	App::Color d(0.0, 1.0, 0.0);

	//View3DInventor* viewer = View3DInventor::Instance;
	View3DInventor* viewer = getInventor();
	if (!viewer)
		return;

	viewer->addDimension3d(createLinearDimension(p1, p2, SbColor(c.r, c.g, c.b)));

	//create deltas. point1 will always be the same.
	gp_Pnt temp = p1;
	gp_Pnt lastTemp = temp;
	temp.SetX(p2.X());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
	lastTemp = temp;
	temp.SetY(p2.Y());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
	lastTemp = temp;
	temp.SetZ(p2.Z());
	viewer->addDimensionDelta(createLinearDimension(lastTemp, temp, SbColor(d.r, d.g, d.b)));
}

void TaskMeasureLinear::goDimensionLinearNoTask(const TopoDS_Shape &shape1, const TopoDS_Shape &shape2)
{
	//Warning: BRepExtrema_DistShapeShape solution array is NOT 0 based.
	if (QGuiApplication::keyboardModifiers() == Qt::KeyboardModifier::ShiftModifier &&
		shape1.ShapeType() == TopAbs_ShapeEnum::TopAbs_EDGE &&
		shape2.ShapeType() == TopAbs_ShapeEnum::TopAbs_EDGE)
	{
		BRepExtrema_ExtCC extremaDist(TopoDS::Edge(shape1), TopoDS::Edge(shape2));
		auto count = extremaDist.NbExt();
		auto p1 = extremaDist.PointOnE1(count);
		auto p2 = extremaDist.PointOnE2(count);
		addLinearDimensions(p1,p2);
	}
	//dumpLinearResults(measure);
	else
	{
		BRepExtrema_DistShapeShape measure(shape1, shape2);
		if (!measure.IsDone() || measure.NbSolution() < 1)
			return;
		addLinearDimensions(measure);
	}

	//if we ever make this a class add viewer to member.
	//View3DInventor *viewer = View3DInventor::Instance;
	View3DInventor* viewer = getInventor();
	if (!viewer)
		return;
	viewer->turn3dDimensionsOn();
}

void toggle3d()
{
	if (visibility)
		visibility = false;
	else
		visibility = true;
}

void TaskMeasure::eraseAllDimensions()
{
	View3DInventor* viewer = getInventor();
	if (!viewer)
		return;
	viewer->eraseAllDimensions();
}

gp_Vec convert(const TopoDS_Vertex &vertex)
{
	gp_Pnt point = BRep_Tool::Pnt(vertex);
	gp_Vec out(point.X(), point.Y(), point.Z());
	return out;
}

void TaskMeasureAngular::goDimensionAngularNoTask(const VectorAdapter &vector1Adapter, const VectorAdapter &vector2Adapter)
{
	gp_Vec vector1 = vector1Adapter;
	gp_Vec vector2 = vector2Adapter;
	double angle = vector1.Angle(vector2);

	SbMatrix dimSys;
	double radius;
	double displayAngle;//have to fake the angle in the 3d.

	if (vector1.IsParallel(vector2, Precision::Angular()))
	{
		//take first point project it onto second vector.
		Handle(Geom_Curve) heapLine2 = new Geom_Line(vector2Adapter);
		gp_Pnt tempPoint(vector1Adapter.getPickPoint().XYZ());

		GeomAPI_ProjectPointOnCurve projection(tempPoint, heapLine2);
		if (projection.NbPoints() < 1) return;
		gp_Vec newPoint2;
		newPoint2.SetXYZ(projection.Point(1).XYZ());

		//if points are colinear, projection doesn't work and returns the same point.
		//In this case we just use the original point.
		if ((newPoint2 - vector1Adapter.getPickPoint()).Magnitude() < Precision::Confusion())
			newPoint2 = vector2Adapter.getPickPoint();

		//now get midpoint between for dim origin.
		gp_Vec point1 = vector1Adapter.getPickPoint();
		gp_Vec midPointProjection = newPoint2 - point1;
		double distance = midPointProjection.Magnitude();
		midPointProjection.Normalize();
		midPointProjection *= distance / 2.0;

		gp_Vec origin = point1 + midPointProjection;

		//yaxis should be the same as vector1, but doing this to eliminate any potential slop from
		//using precision::angular. If lines are colinear and we have no plane, we can't establish zAxis from crossing.
		//we just the absolute axis.
		gp_Vec xAxis = (point1 - origin).Normalized();
		gp_Vec zAxis;
		if (xAxis.IsParallel(vector1, Precision::Angular()))
		{
			if (!xAxis.IsParallel(gp_Vec(0.0, 0.0, 1.0), Precision::Angular()))
				zAxis = gp_Vec(0.0, 0.0, 1.0);
			else
				zAxis = gp_Vec(0.0, 1.0, 0.0);
		}
		else
			zAxis = xAxis.Crossed(vector1).Normalized();
		gp_Vec yAxis = zAxis.Crossed(xAxis).Normalized();
		zAxis = xAxis.Crossed(yAxis).Normalized();

		dimSys = SbMatrix
		(
			xAxis.X(), yAxis.X(), zAxis.X(), origin.X(),
			xAxis.Y(), yAxis.Y(), zAxis.Y(), origin.Y(),
			xAxis.Z(), yAxis.Z(), zAxis.Z(), origin.Z(),
			0.0, 0.0, 0.0, 1.0
		);
		dimSys = dimSys.transpose();

		radius = midPointProjection.Magnitude();
		displayAngle = M_PI;
	}
	else
	{
		Handle(Geom_Curve) heapLine1 = new Geom_Line(vector1Adapter);
		Handle(Geom_Curve) heapLine2 = new Geom_Line(vector2Adapter);

		GeomAPI_ExtremaCurveCurve extrema(heapLine1, heapLine2);

		if (extrema.NbExtrema() < 1)
		{
			return;
		}

		gp_Pnt extremaPoint1, extremaPoint2, dimensionOriginPoint;
		extrema.Points(1, extremaPoint1, extremaPoint2);
		if (extremaPoint1.Distance(extremaPoint2) < Precision::Confusion())
			dimensionOriginPoint = extremaPoint1;
		else
		{
			//find halfway point in between extrema points for dimension origin.
			//求两面法线的最近点的中点
			gp_Vec vec1(extremaPoint1.XYZ());
			gp_Vec vec2(extremaPoint2.XYZ());
			gp_Vec connection(vec2 - vec1);
			Standard_Real distance = connection.Magnitude();
			connection.Normalize();
			connection *= (distance / 2.0);
			dimensionOriginPoint.SetXYZ((vec1 + connection).XYZ());
		}

		gp_Vec thirdPoint(vector2Adapter.getPickPoint());
		gp_Vec originVector(dimensionOriginPoint.XYZ());
		gp_Vec extrema2Vector(extremaPoint2.XYZ());
		radius = (vector1Adapter.getPickPoint() - originVector).Magnitude();
		double legOne = (extrema2Vector - originVector).Magnitude();
		displayAngle = angle;
		if (legOne > Precision::Confusion())
		{
			double legTwo = sqrt(pow(radius, 2) - pow(legOne, 2));
			gp_Vec projectionVector(vector2);
			projectionVector.Normalize();
			projectionVector *= legTwo;
			thirdPoint = extrema2Vector + projectionVector;
			gp_Vec hyp(thirdPoint - originVector);
			hyp.Normalize();
			gp_Vec otherSide(vector1Adapter.getPickPoint() - originVector);
			otherSide.Normalize();
			displayAngle = hyp.Angle(otherSide);
		}

		gp_Vec xAxis = (vector1Adapter.getPickPoint() - originVector).Normalized();
		gp_Vec fakeYAxis = (thirdPoint - originVector).Normalized();
		gp_Vec zAxis = (xAxis.Crossed(fakeYAxis)).Normalized();
		gp_Vec yAxis = zAxis.Crossed(xAxis).Normalized();

		dimSys = SbMatrix
		(
			xAxis.X(), yAxis.X(), zAxis.X(), dimensionOriginPoint.X(),
			xAxis.Y(), yAxis.Y(), zAxis.Y(), dimensionOriginPoint.Y(),
			xAxis.Z(), yAxis.Z(), zAxis.Z(), dimensionOriginPoint.Z(),
			0.0, 0.0, 0.0, 1.0
		);

		dimSys = dimSys.transpose();
	}

	App::Color c(0.0, 0.0, 1.0);

	DimensionAngular *dimension = new DimensionAngular();
	dimension->ref();
	dimension->matrix.setValue(dimSys);
	dimension->radius.setValue(radius);
	dimension->angle.setValue(static_cast<float>(displayAngle));
	dimension->text.setValue(getPrintValue(180 * angle / M_PI,1).toStdString().c_str());
	dimension->dColor.setValue(SbColor(c.r, c.g, c.b));
	dimension->setupDimension();

	View3DInventor* viewer = getInventor();
	if (viewer)
		viewer->addDimension3d(dimension);
	dimension->unref();
	viewer->turn3dDimensionsOn();
}

void TaskMeasureAngular::goDimensionAngularNoTask_new(const VectorAdapter &vector1Adapter, const VectorAdapter &vector2Adapter, gp_Pnt arcOrigin, double trueAngle)
{
	gp_Vec vector1 = vector1Adapter;
	gp_Vec vector2 = vector2Adapter;

	SbMatrix dimSys;
	double radius;
	double displayAngle;//have to fake the angle in the 3d.

	if (vector1.IsParallel(vector2, Precision::Angular()))
	{
		//take first point project it onto second vector.
		Handle(Geom_Curve) heapLine2 = new Geom_Line(vector2Adapter);
		gp_Pnt tempPoint(vector1Adapter.getPickPoint().XYZ());

		GeomAPI_ProjectPointOnCurve projection(tempPoint, heapLine2);
		if (projection.NbPoints() < 1) return;
		gp_Vec newPoint2;
		newPoint2.SetXYZ(projection.Point(1).XYZ());

		//if points are colinear, projection doesn't work and returns the same point.
		//In this case we just use the original point.
		if ((newPoint2 - vector1Adapter.getPickPoint()).Magnitude() < Precision::Confusion())
			newPoint2 = vector2Adapter.getPickPoint();

		//now get midpoint between for dim origin.
		gp_Vec point1 = vector1Adapter.getPickPoint();
		gp_Vec midPointProjection = newPoint2 - point1;
		double distance = midPointProjection.Magnitude();
		midPointProjection.Normalize();
		midPointProjection *= distance / 2.0;

		gp_Vec origin = point1 + midPointProjection;

		//yaxis should be the same as vector1, but doing this to eliminate any potential slop from
		//using precision::angular. If lines are colinear and we have no plane, we can't establish zAxis from crossing.
		//we just the absolute axis.
		gp_Vec xAxis = (point1 - origin).Normalized();
		gp_Vec zAxis;
		if (xAxis.IsParallel(vector1, Precision::Angular()))
		{
			if (!xAxis.IsParallel(gp_Vec(0.0, 0.0, 1.0), Precision::Angular()))
				zAxis = gp_Vec(0.0, 0.0, 1.0);
			else
				zAxis = gp_Vec(0.0, 1.0, 0.0);
		}
		else
			zAxis = xAxis.Crossed(vector1).Normalized();
		gp_Vec yAxis = zAxis.Crossed(xAxis).Normalized();
		zAxis = xAxis.Crossed(yAxis).Normalized();

		dimSys = SbMatrix
		(
			xAxis.X(), yAxis.X(), zAxis.X(), origin.X(),
			xAxis.Y(), yAxis.Y(), zAxis.Y(), origin.Y(),
			xAxis.Z(), yAxis.Z(), zAxis.Z(), origin.Z(),
			0.0, 0.0, 0.0, 1.0
		);
		dimSys = dimSys.transpose();

		radius = midPointProjection.Magnitude();
		displayAngle = M_PI;
	}
	else
	{
		gp_Vec vec1(vector1Adapter.getPickPoint() - gp_Vec(arcOrigin.XYZ()));
		radius = vec1.Magnitude();
		gp_Vec vec2(vector2Adapter.getPickPoint() - gp_Vec(arcOrigin.XYZ()));
		gp_Vec circleNormal = vec1.Crossed(vec2);
		circleNormal.Normalize();
		gp_Vec axisX, axisY, axisZ;
		axisX = vec1.Normalized();
		axisZ = circleNormal;
		axisY = axisZ.Crossed(axisX).Normalized();
		dimSys = SbMatrix(
			axisX.X(), axisY.X(), axisZ.X(), arcOrigin.X(),
			axisX.Y(), axisY.Y(), axisZ.Y(), arcOrigin.Y(),
			axisX.Z(), axisY.Z(), axisZ.Z(), arcOrigin.Z(),
			0,		   0,		  0,		 1
		);
		dimSys = dimSys.transpose();
	}

	App::Color c(0.0, 0.0, 1.0);

	DimensionAngular *dimension = new DimensionAngular();
	auto pickS = (SoPickStyle*)dimension->getPart("pickStyle", true);
	if (pickS) {
		pickS->style.setValue(SoPickStyle::Style::UNPICKABLE);
	}
	dimension->ref();
	dimension->matrix.setValue(dimSys);
	dimension->radius.setValue(radius);
	dimension->angle.setValue(static_cast<float>(/*displayAngle*/trueAngle));
	dimension->text.setValue(getPrintValue(180 * trueAngle / M_PI, 1).toStdString().c_str());
	dimension->dColor.setValue(SbColor(c.r, c.g, c.b));
	dimension->setupDimension();

	View3DInventor* viewer = getInventor();
	if (viewer)
		viewer->addDimension3d(dimension);
	dimension->unref();
	viewer->turn3dDimensionsOn();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SO_KIT_SOURCE(DimensionLinear)

void DimensionLinear::initClass()
{
	SO_KIT_INIT_CLASS(DimensionLinear, SoSeparatorKit, "SeparatorKit");
}

DimensionLinear::DimensionLinear()
{
	SO_KIT_CONSTRUCTOR(DimensionLinear);

	SO_KIT_ADD_CATALOG_ENTRY(transformation, SoTransform, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(annotate, SoAnnotation, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(leftArrow, SoShapeKit, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(rightArrow, SoShapeKit, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(line, SoShapeKit, true, annotate, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(textSep, SoSeparator, true, annotate, "", true);

	SO_KIT_INIT_INSTANCE();

	SO_NODE_ADD_FIELD(rotate, (1.0, 0.0, 0.0, 0.0));//position orientation of the dimension.
	SO_NODE_ADD_FIELD(length, (1.0));//turns into dimension length
	SO_NODE_ADD_FIELD(origin, (0.0, 0.0, 0.0));//static
	SO_NODE_ADD_FIELD(text, ("test"));//dimension text
	SO_NODE_ADD_FIELD(dColor, (1.0, 0.0, 0.0));//dimension color.
}

DimensionLinear::~DimensionLinear()
{

}

SbBool DimensionLinear::affectsState() const
{
	return false;
}

void DimensionLinear::setupDimension()
{
	//transformation
	SoTransform *trans = static_cast<SoTransform *>(getPart("transformation", true));
	trans->translation.connectFrom(&point1);
	//build engine for vector subtraction and length.
	SoCalculator *hyp = new SoCalculator();
	hyp->A.connectFrom(&point1);
	hyp->B.connectFrom(&point2);
	hyp->expression.set1Value(0, "oA = B-A");
	hyp->expression.set1Value(1, "oB = normalize(oA)");
	hyp->expression.set1Value(2, "oa = length(oA)");
	length.connectFrom(&hyp->oa);

	//build engine for rotation.
	SoComposeRotationFromTo *rotationEngine = new SoComposeRotationFromTo();
	rotationEngine->from.setValue(SbVec3f(1.0, 0.0, 0.0));
	rotationEngine->to.connectFrom(&hyp->oB);
	trans->rotation.connectFrom(&rotationEngine->rotation);

	//color
	SoMaterial *material = new SoMaterial;
	material->diffuseColor.connectFrom(&dColor);

	//dimension arrows
	float dimLength = (point2.getValue() - point1.getValue()).length();
	float coneHeight = dimLength * 0.06;
	float coneRadius = coneHeight * 0.3;

	SoCone *cone = new SoCone();
	cone->bottomRadius.setValue(coneRadius);
	cone->height.setValue(coneHeight);

	char lStr[100];
	char rStr[100];
	snprintf(lStr, sizeof(lStr), "translation %.6f 0.0 0.0", coneHeight * 0.5);
	snprintf(rStr, sizeof(rStr), "translation 0.0 -%.6f 0.0", coneHeight * 0.5);

	setPart("leftArrow.shape", cone);
	set("leftArrow.transform", "rotation 0.0 0.0 1.0 1.5707963");
	set("leftArrow.transform", lStr);
	setPart("rightArrow.shape", cone);
	set("rightArrow.transform", "rotation 0.0 0.0 -1.0 1.5707963"); //no constant for PI.
	//have use local here to do the offset because the main is wired up to length of dimension.
	set("rightArrow.localTransform", rStr);

	SoTransform *transform = static_cast<SoTransform *>(getPart("rightArrow.transform", false));
	if (!transform)
		return;//what to do here?
	SoComposeVec3f *vec = new SoComposeVec3f;
	vec->x.connectFrom(&length);
	vec->y.setValue(0.0);
	vec->z.setValue(0.0);
	transform->translation.connectFrom(&vec->vector);

	setPart("leftArrow.material", material);
	setPart("rightArrow.material", material);

	//line
	SoConcatenate *catEngine = new SoConcatenate(SoMFVec3f::getClassTypeId());
	//don't know how to get around having this dummy origin. cat engine wants to connectfrom?
	catEngine->input[0]->connectFrom(&origin);
	catEngine->input[1]->connectFrom(&vec->vector);

	SoVertexProperty *lineVerts = new SoVertexProperty;
	lineVerts->vertex.connectFrom(catEngine->output);

	int lineVertexMap[] = { 0, 1 };
	int lineVertexMapSize(sizeof(lineVertexMap) / sizeof(int));
	SoIndexedLineSet *line = new SoIndexedLineSet;
	line->vertexProperty = lineVerts;
	line->coordIndex.setValues(0, lineVertexMapSize, lineVertexMap);

	setPart("line.shape", line);
	setPart("line.material", material);

	//text
	SoSeparator *textSep = static_cast<SoSeparator *>(getPart("textSep", true));
	if (!textSep)
		return;

	textSep->addChild(material);

	SoCalculator *textVecCalc = new SoCalculator();
	textVecCalc->A.connectFrom(&vec->vector);
	textVecCalc->B.set1Value(0, 0.0, 0.250, 0.0);
	textVecCalc->expression.set1Value(0, "oA = (A / 2) + B");

	SoTransform *textTransform = new SoTransform();
	textTransform->translation.connectFrom(&textVecCalc->oA);
	textSep->addChild(textTransform);

	SoFont *fontNode = new SoFont();
	fontNode->name.setValue("default font");
	fontNode->size.setValue(40);
	textSep->addChild(fontNode);

	SoText2 *textNode = new SoText2();
	textNode->justification = SoText2::CENTER;
	textNode->string.connectFrom(&text);
	textSep->addChild(textNode);

	//this prevents the 2d text from screwing up the bounding box for a viewall
	SoResetTransform *rTrans = new SoResetTransform;
	rTrans->whatToReset = SoResetTransform::BBOX;
	textSep->addChild(rTrans);
}

void DimensionLinear::drawAxis(const SbColor color)
{
	dColor.setValue(color);
	//transformation
	SoTransform *trans = static_cast<SoTransform *>(getPart("transformation", true));
	trans->translation.connectFrom(&point1);
	//build engine for vector subtraction and length.
	SoCalculator *hyp = new SoCalculator();
	hyp->A.connectFrom(&point1);
	hyp->B.connectFrom(&point2);
	hyp->expression.set1Value(0, "oA = B-A");
	hyp->expression.set1Value(1, "oB = normalize(oA)");
	hyp->expression.set1Value(2, "oa = length(oA)");
	length.connectFrom(&hyp->oa);

	//build engine for rotation.
	SoComposeRotationFromTo *rotationEngine = new SoComposeRotationFromTo();
	rotationEngine->from.setValue(SbVec3f(1.0, 0.0, 0.0));
	rotationEngine->to.connectFrom(&hyp->oB);
	trans->rotation.connectFrom(&rotationEngine->rotation);

	//color
	SoMaterial *material = new SoMaterial;
	material->diffuseColor.connectFrom(&dColor);

	//dimension arrows
	float dimLength = (point2.getValue() - point1.getValue()).length();
	float coneHeight = dimLength * 0.2;
	float coneRadius = coneHeight * 0.3;

	SoCone *cone = new SoCone();
	cone->bottomRadius.setValue(coneRadius);
	cone->height.setValue(coneHeight);

	char lStr[100];
	char rStr[100];
	snprintf(lStr, sizeof(lStr), "translation %.6f 0.0 0.0", coneHeight * 0.5);
	snprintf(rStr, sizeof(rStr), "translation 0.0 -%.6f 0.0", coneHeight * 0.5);

	setPart("rightArrow.shape", cone);
	set("rightArrow.transform", "rotation 0.0 0.0 -1.0 1.5707963"); //no constant for PI.
	//have use local here to do the offset because the main is wired up to length of dimension.
	set("rightArrow.localTransform", rStr);

	SoTransform *transform = static_cast<SoTransform *>(getPart("rightArrow.transform", false));
	if (!transform)
		return;//what to do here?
	SoComposeVec3f *vec = new SoComposeVec3f;
	vec->x.connectFrom(&length);
	vec->y.setValue(0.0);
	vec->z.setValue(0.0);
	transform->translation.connectFrom(&vec->vector);

	setPart("rightArrow.material", material);

	//line
	SoConcatenate *catEngine = new SoConcatenate(SoMFVec3f::getClassTypeId());
	//don't know how to get around having this dummy origin. cat engine wants to connectfrom?
	catEngine->input[0]->connectFrom(&origin);
	catEngine->input[1]->connectFrom(&vec->vector);

	SoVertexProperty *lineVerts = new SoVertexProperty;
	lineVerts->vertex.connectFrom(catEngine->output);

	int lineVertexMap[] = { 0, 1 };
	int lineVertexMapSize(sizeof(lineVertexMap) / sizeof(int));
	SoIndexedLineSet *line = new SoIndexedLineSet;
	line->vertexProperty = lineVerts;
	line->coordIndex.setValues(0, lineVertexMapSize, lineVertexMap);

	setPart("line.shape", line);
	setPart("line.material", material);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DimensionControl::DimensionControl(QWidget* parent) : QWidget(parent)
{
	resetButton = new QPushButton(this);

	QPushButton *toggle3dButton = new QPushButton(this);
	QObject::connect(toggle3dButton, SIGNAL(clicked(bool)), this, SLOT(toggle3dSlot(bool)));

	QPushButton *clearAllButton = new QPushButton(this);
	QObject::connect(clearAllButton, SIGNAL(clicked(bool)), this, SLOT(clearAllSlot(bool)));
}

void DimensionControl::toggle3dSlot(bool)
{
	toggle3d();
}

void DimensionControl::clearAllSlot(bool)
{
	eraseAllDimensions();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SteppedSelection::SteppedSelection(const uint &buttonCountIn, QWidget *parent)
	:QWidget(parent)
{
	if (buttonCountIn < 1)
		return;
	for (uint index = 0; index < buttonCountIn; ++index)
	{
		ButtonIconPairType tempPair;

		AbstractButton *button = new AbstractButton(this);
		connect(button, SIGNAL(toggled(bool)), this, SLOT(selectionSlot(bool)));

		QLabel *label = new QLabel(this);
		tempPair.first = button;
		tempPair.second = label;
		buttons.push_back(tempPair);
	}
}

SteppedSelection::~SteppedSelection()
{

}

void SteppedSelection::selectionSlot(bool checked)
{
	AbstractButton *sender = qobject_cast<AbstractButton*>(QObject::sender());
	assert(sender != 0);
	std::vector<ButtonIconPairType>::iterator it;
	for (it = buttons.begin(); it != buttons.end(); ++it)
		if (it->first == sender)
			break;
	assert(it != buttons.end());
}

AbstractButton* SteppedSelection::getButton(const uint& index)
{
	return buttons.at(index).first;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskMeasureLinear::TaskMeasureLinear()
	:buttonSelectedIndex(0)
{
	View3DInventor* viewer = getInventor();
	stepped = new SteppedSelection(2, viewer);
	DimensionControl *control = new DimensionControl(viewer);
	//QObject::connect(control->resetButton, SIGNAL(clicked(bool)), this, SLOT(resetDialogSlot(bool)));
	QObject::connect(stepped->getButton(0), SIGNAL(toggled(bool)), this, SLOT(selection1Slot(bool)));
	QObject::connect(stepped->getButton(1), SIGNAL(toggled(bool)), this, SLOT(selection2Slot(bool)));
	viewer->setEditing(true);
	viewer->setEditingCursor(QCursor(QPixmap(cursor_ruler), 7, 6));
}

void TaskMeasureLinear::selection1Slot(bool checked)
{
	if (!checked)
	{
		return;
	}
	buttonSelectedIndex = 0;

	this->blockConnection(true);
	//we should only be working with 1 entity, but oh well do the loop anyway.
	std::vector<DimSelections::DimSelection>::const_iterator it;
	for (it = selections1.selections.begin(); it != selections1.selections.end(); ++it)
		Selection().addSelection(it->vp,it->subObjectName.c_str()/*it->documentName.c_str(), it->objectName.c_str(), it->subObjectName.c_str()*/);
	this->blockConnection(false);
}

void TaskMeasureLinear::selection2Slot(bool checked)
{
	if (!checked)
		return;
	buttonSelectedIndex = 1;
	this->blockConnection(true);
	std::vector<DimSelections::DimSelection>::const_iterator it;
	for (it = selections2.selections.begin(); it != selections2.selections.end(); ++it)
		Selection().addSelection(it->vp, it->subObjectName.c_str());
	this->blockConnection(false);
}

void TaskMeasureLinear::onSelectionChanged(const SelectionChanges& msg)
{
	if (buttonSelectedIndex == 0)
	{
		if (msg.Type == SelectionChanges::AddSelection)
		{
			DimSelections::DimSelection newSelection;
			newSelection.shapeType = DimSelections::None;
			newSelection.vp = msg.vp;
			newSelection.subObjectName = msg.pSubName;
			newSelection.x = msg.x;
			newSelection.y = msg.y;
			newSelection.z = msg.z;
			selections1.selections.clear();//we only want one item.
			selections1.selections.push_back(newSelection);
			QTimer::singleShot(1000, this, SLOT(selectionClearDelayedSlot()));
			stepped->getButton(1)->setToggle(true);
			return;
		}
	}
	if (buttonSelectedIndex == 1)
	{
		if (msg.Type == SelectionChanges::AddSelection)
		{
			DimSelections::DimSelection newSelection;
			newSelection.shapeType = DimSelections::None;
			newSelection.vp = msg.vp;
			newSelection.subObjectName = msg.pSubName;
			newSelection.x = msg.x;
			newSelection.y = msg.y;
			newSelection.z = msg.z;
			selections2.selections.clear();//we only want one item.
			selections2.selections.push_back(newSelection);
			buildDimension();
			clearSelectionStrings();
			QTimer::singleShot(0, this, SLOT(selectionClearDelayedSlot()));
			stepped->getButton(0)->setToggle(true);
			return;
		}
	}
}

void TaskMeasureLinear::buildDimension() {
	try {
		buildDimension(selections1, selections2);
	}
	catch (...) {
		buttonSelectedIndex = 0;
		clearSelectionStrings();
	}
}

void TaskMeasureLinear::clearSelectionStrings()
{
	selections1.selections.clear();
	selections2.selections.clear();
}

void TaskMeasureLinear::buildDimension(const DimSelections &sel1, const DimSelections &sel2)
{
	if (sel1.selections.size() != 1 || sel2.selections.size() != 1)
		return;

	DimSelections::DimSelection current1 = sel1.selections.at(0);
	DimSelections::DimSelection current2 = sel2.selections.at(0);

	TopoDS_Shape shape1, shape2;
	shape1 = static_cast<ViewProviderPartExt*>(current1.vp)->getShape();
	shape1 = getSubElementShape(shape1, current1.subObjectName.c_str());
	shape2 = static_cast<ViewProviderPartExt*>(current2.vp)->getShape();
	shape2 = getSubElementShape(shape2, current2.subObjectName.c_str());
	if (shape1.IsNull() || shape2.IsNull()) return;
	_Measures.emplace_back(sel1, sel2, true);
	goDimensionLinearNoTask(shape1, shape2);
}

void TaskMeasureLinear::toggle3dSlot(bool)
{
	toggle3d();
}

void TaskMeasureLinear::selectionClearDelayedSlot()
{
	//hack.
	//clearing selections are not working as I hoped. Apparently the observer callback gets called
	//before the actual selection takes place. Resulting in selections being left. this addresses this
	//by being called from the event loop.
	this->blockConnection(true);
	this->blockConnection(false);
}

void TaskMeasureLinear::clearAllSlot(bool)
{
	eraseAllDimensions();
}

TaskMeasureLinear::~TaskMeasureLinear()
{
	View3DInventor* viewer = getInventor();
	if (viewer) {
		viewer->setEditing(false);
		viewer->setEditingCursor(QCursor(Qt::ArrowCursor));
	}
	try
	{
		Selection().clearSelection();
	}
	catch (...)
	{

	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DimensionLinearEvent::DimensionLinearEvent(const Base::Type& s)
	: QEvent(QEvent::Type(QEvent::User + 1)), t(s)
{

}

const Base::Type& DimensionLinearEvent::style() const
{
	return t;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

VectorAdapter::VectorAdapter() : status(false), vector()
{
}

VectorAdapter::VectorAdapter(const TopoDS_Face &faceIn, const gp_Vec &pickedPointIn) :
	status(false), vector(), origin(pickedPointIn)
{
	Handle(Geom_Surface) surface = BRep_Tool::Surface(faceIn);
	if (surface->IsKind(STANDARD_TYPE(Geom_ElementarySurface)))
	{
		Handle(Geom_ElementarySurface) eSurface = Handle(Geom_ElementarySurface)::DownCast(surface);
		gp_Dir direction = eSurface->Axis().Direction();
		vector = direction;
		vector.Normalize();
		if (faceIn.Orientation() == TopAbs_REVERSED)
			vector.Reverse();
		if (surface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)) ||
			surface->IsKind(STANDARD_TYPE(Geom_SphericalSurface))
			)
		{
			origin = eSurface->Axis().Location().XYZ();
			projectOriginOntoVector(pickedPointIn);
		}
		else
			origin = pickedPointIn + vector;
		status = true;
	}
}

VectorAdapter::VectorAdapter(const TopoDS_Edge &edgeIn, const gp_Vec &pickedPointIn) :
	status(false), vector(), origin(pickedPointIn)
{
	TopoDS_Vertex firstVertex = TopExp::FirstVertex(edgeIn, Standard_True);
	TopoDS_Vertex lastVertex = TopExp::LastVertex(edgeIn, Standard_True);
	vector = convert(lastVertex) - convert(firstVertex);
	if (vector.Magnitude() < Precision::Confusion())
		return;
	vector.Normalize();

	status = true;
	projectOriginOntoVector(pickedPointIn);
}

VectorAdapter::VectorAdapter(const TopoDS_Vertex &vertex1In, const TopoDS_Vertex &vertex2In) :
	status(false), vector(), origin()
{
	vector = convert(vertex2In) - convert(vertex1In);
	vector.Normalize();

	//build origin half way.
	gp_Vec tempVector = (convert(vertex2In) - convert(vertex1In));
	double mag = tempVector.Magnitude();
	tempVector.Normalize();
	tempVector *= (mag / 2.0);
	origin = tempVector + convert(vertex1In);

	status = true;
}

VectorAdapter::VectorAdapter(const gp_Vec &vector1, const gp_Vec &vector2) :
	status(false), vector(), origin()
{
	vector = vector2 - vector1;
	vector.Normalize();

	//build origin half way.
	gp_Vec tempVector = vector2 - vector1;
	double mag = tempVector.Magnitude();
	tempVector.Normalize();
	tempVector *= (mag / 2.0);
	origin = tempVector + vector1;

	status = true;
}

void VectorAdapter::projectOriginOntoVector(const gp_Vec &pickedPointIn)
{
	Handle(Geom_Curve) heapLine = new Geom_Line(origin.XYZ(), vector.XYZ());
	gp_Pnt tempPoint(pickedPointIn.XYZ());
	GeomAPI_ProjectPointOnCurve projection(tempPoint, heapLine);
	if (projection.NbPoints() < 1)
		return;
	origin.SetXYZ(projection.Point(1).XYZ());
}

VectorAdapter::operator gp_Lin() const
{
	gp_Pnt tempOrigin;
	tempOrigin.SetXYZ(origin.XYZ());
	return gp_Lin(tempOrigin, gp_Dir(vector));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskMeasureAngular::TaskMeasureAngular()
	: SelectionObserver(true, false)
	, selections1(), selections2(), buttonSelectedIndex(0)
{
	setUpGui();
}

TaskMeasureAngular::~TaskMeasureAngular()
{
	View3DInventor* viewer = getInventor();
	if (viewer) {
		viewer->setEditing(false);
		viewer->setEditingCursor(QCursor(Qt::ArrowCursor));
	}
	try {
		Selection().clearSelection();
	}
	catch (...) { }
}

void TaskMeasureAngular::onSelectionChanged(const SelectionChanges& msg)
{
	if (!msg.vp) return;
	TopoDS_Shape shape = static_cast<ViewProviderPartExt*>(msg.vp)->getShape();
	shape = getSubElementShape(shape, msg.pSubName);
	Matrix4D mat;
	if (shape.IsNull()) return;
	NCollection_Mat4<double> theMat;
	msg.vp->getTransform().GetMat4(theMat);
	auto val = theMat.GetData();
	mat = Matrix4D(val[0],  val[1],  val[2],  val[3]
				 , val[4],  val[5],  val[6],  val[7]
				 , val[8],  val[9],  val[10], val[11]
				 , val[12], val[13], val[14], val[15]);
	mat.inverse();
	DimSelections::DimSelection newSelection;
	newSelection.shapeType = DimSelections::None;
	newSelection.vp = msg.vp;
	newSelection.subObjectName = msg.pSubName;
	gp_Vec pickPoint(msg.x, msg.y, msg.z);
	Vector3d v(msg.x, msg.y, msg.z);
	v = mat * v;
	newSelection.x = v.x;
	newSelection.y = v.y;
	newSelection.z = v.z;
	if (buttonSelectedIndex == 0)
	{
		if (msg.Type == SelectionChanges::AddSelection)
		{
			if (shape.ShapeType() == TopAbs_VERTEX)
			{
				//if we have previous selection it should be only one vertex.
				if (selections1.selections.size() > 1)
					selections1.selections.clear();
				else if (selections1.selections.size() == 1)
				{
					//make sure it is a vertex.
					if (selections1.selections.at(0).shapeType != DimSelections::Vertex)
						selections1.selections.clear();
				}

				newSelection.shapeType = DimSelections::Vertex;
				selections1.selections.push_back(newSelection);
				if (selections1.selections.size() == 1)
					return;
				//here we should have 2 vertices, but will check to make sure.
				assert(selections1.selections.size() == 2);
				assert(selections1.selections.at(0).shapeType == DimSelections::Vertex);
				assert(selections1.selections.at(1).shapeType == DimSelections::Vertex);

				QTimer::singleShot(0, this, SLOT(selectionClearDelayedSlot()));
				stepped->getButton(1)->setToggle(true);
				return;
			}

			//here there should only be one in the selections container. so just clear it.
			selections1.selections.clear();

			if (shape.ShapeType() == TopAbs_EDGE)
			{
				newSelection.shapeType = DimSelections::Edge;
				selections1.selections.push_back(newSelection);
			}

			if (shape.ShapeType() == TopAbs_FACE)
			{
				newSelection.shapeType = DimSelections::Face;
				selections1.selections.push_back(newSelection);
			}

			QTimer::singleShot(0, this, SLOT(selectionClearDelayedSlot()));
			stepped->getButton(1)->setToggle(true);
			return;
		}
	}
	if (buttonSelectedIndex == 1)
	{
		if (msg.Type == SelectionChanges::AddSelection)
		{
			if (shape.ShapeType() == TopAbs_VERTEX)
			{
				//if we have previous selection it should be only one vertex.
				if (selections2.selections.size() > 1)
					selections2.selections.clear();
				else if (selections2.selections.size() == 1)
				{
					//make sure it is a vertex.
					if (selections2.selections.at(0).shapeType != DimSelections::Vertex)
						selections2.selections.clear();
				}

				newSelection.shapeType = DimSelections::Vertex;
				selections2.selections.push_back(newSelection);
				if (selections2.selections.size() == 1)
					return;
				//here we should have 2 vertices, but will check to make sure.
				assert(selections2.selections.size() == 2);
				assert(selections2.selections.at(0).shapeType == DimSelections::Vertex);
				assert(selections2.selections.at(1).shapeType == DimSelections::Vertex);

				buildDimension();
				clearSelection();
				QTimer::singleShot(0, this, SLOT(selectionClearDelayedSlot()));
				stepped->getButton(0)->setToggle(true);
				return;
			}
			//vertices have to be selected in succession. if we get here,clear temp selection.
			selections2.selections.clear();

			if (shape.ShapeType() == TopAbs_EDGE)
			{
				newSelection.shapeType = DimSelections::Edge;
				selections2.selections.push_back(newSelection);
			}

			if (shape.ShapeType() == TopAbs_FACE)
			{
				newSelection.shapeType = DimSelections::Face;
				selections2.selections.push_back(newSelection);
			}

			buildDimension();
			clearSelection();
			QTimer::singleShot(0, this, SLOT(selectionClearDelayedSlot()));
			stepped->getButton(0)->setToggle(true);
			return;
		}
	}
}

void TaskMeasureAngular::selectionClearDelayedSlot()
{
	//hack.
	//clearing selections are not working as I hoped. Apparently the observer callback gets called
	//before the actual selection takes place. Resulting in selections being left. this addresses this
	//by being called from the event loop.
	this->blockConnection(true);
	Selection().clearSelection();
	this->blockConnection(false);
}

VectorAdapter TaskMeasureAngular::buildAdapter(const DimSelections& selection)
{
	Matrix4D mat;
	assert(selection.selections.size() > 0 && selection.selections.size() < 3);
	if (selection.selections.size() == 1)
	{
		DimSelections::DimSelection current = selection.selections.at(0);
		if (current.shapeType == DimSelections::Edge)
		{
			TopoDS_Shape edgeShape = getSubElementShape(static_cast<ViewProviderPartExt*>(current.vp)->getShape(),
														current.subObjectName.c_str());
			if (edgeShape.IsNull())	return VectorAdapter();
			NCollection_Mat4<double> occmat4;
			current.vp->getTransform().GetMat4(occmat4);
			mat = Matrix4D(occmat4[0] , occmat4[1]  , occmat4[2], occmat4[3]
						 , occmat4[4] , occmat4[5]  , occmat4[6], occmat4[7]
						 , occmat4[8] , occmat4[9] , occmat4[10], occmat4[11]
						 , occmat4[12], occmat4[13], occmat4[14], occmat4[15]);
			TopoDS_Edge edge = TopoDS::Edge(edgeShape);
			// make edge orientation so that end of edge closest to pick is head of vector.
			TopoDS_Vertex firstVertex = TopExp::FirstVertex(edge, Standard_True);
			TopoDS_Vertex lastVertex = TopExp::LastVertex(edge, Standard_True);
			if (firstVertex.IsNull() || lastVertex.IsNull())
				return VectorAdapter();
			gp_Vec firstPoint = convert(firstVertex);
			gp_Vec lastPoint = convert(lastVertex);
			Vector3d v(current.x, current.y, current.z);
			v = mat * v;
			gp_Vec pickPoint(v.x, v.y, v.z);
			double firstDistance = (firstPoint - pickPoint).Magnitude();
			double lastDistance = (lastPoint - pickPoint).Magnitude();
			if (lastDistance < firstDistance)
			{
				if (edge.Orientation() == TopAbs_FORWARD)
					edge.Orientation(TopAbs_REVERSED);
				else
					edge.Orientation(TopAbs_FORWARD);
			}
			return VectorAdapter(edge, pickPoint);
		}
		if (current.shapeType == DimSelections::Face)
		{
			TopoDS_Shape faceShape = getSubElementShape(static_cast<ViewProviderPartExt*>(current.vp)->getShape(),
														current.subObjectName.c_str());
			if (faceShape.IsNull())	return VectorAdapter();
			NCollection_Mat4<double> occmat4;
			current.vp->getTransform().GetMat4(occmat4);
			mat = Matrix4D(occmat4[0],  occmat4[1],  occmat4[2],  occmat4[3]
						 , occmat4[4],  occmat4[5],  occmat4[6],  occmat4[7]
						 , occmat4[8],  occmat4[9],  occmat4[10], occmat4[11]
						 , occmat4[12], occmat4[13], occmat4[14], occmat4[15]);
			TopoDS_Face face = TopoDS::Face(faceShape);
			Vector3d v(current.x, current.y, current.z);
			//v = mat * v;
			gp_Vec pickPoint(v.x, v.y, v.z);
			return VectorAdapter(face, pickPoint);
		}
	}
	DimSelections::DimSelection current1 = selection.selections.at(0);
	DimSelections::DimSelection current2 = selection.selections.at(1);
	assert(current1.shapeType == DimSelections::Vertex);
	assert(current2.shapeType == DimSelections::Vertex);
	TopoDS_Shape vertexShape1 = static_cast<ViewProviderPartExt*>(current1.vp)->getShape();
	TopoDS_Shape vertexShape2 = static_cast<ViewProviderPartExt*>(current2.vp)->getShape();
	if (vertexShape1.IsNull())
		return VectorAdapter();
	if (vertexShape2.IsNull())
		return VectorAdapter();

	TopoDS_Vertex vertex1 = TopoDS::Vertex(vertexShape1);
	TopoDS_Vertex vertex2 = TopoDS::Vertex(vertexShape2);

	//build a temp adapter to make sure it is valid.
	return VectorAdapter(convert(vertex2), convert(vertex1));
}

void TaskMeasureAngular::buildDimension() {
	try {
		buildDimension(selections1, selections2);
	}
	catch (...) {
		buttonSelectedIndex = 0;
		clearSelection();
	}
}

void TaskMeasureAngular::buildDimension(const DimSelections &sel1, const DimSelections &sel2)
{
	gp_Pnt pickedPoint1(sel1.selections.at(0).x, sel1.selections.at(0).y, sel1.selections.at(0).z);
	gp_Pnt pickedPoint2(sel2.selections.at(0).x, sel2.selections.at(0).y, sel2.selections.at(0).z);

	//获取两个拓展平面的交线
	TopoDS_Shape faceShape = getSubElementShape(
		static_cast<ViewProviderPartExt*>(sel1.selections.at(0).vp)->getShape(),
		sel1.selections.at(0).subObjectName.c_str());
	TopoDS_Face face = TopoDS::Face(faceShape);
	BRepAdaptor_Surface sur(face);
	
	gp_Dir faceDir1;
	switch (sur.GetType())
	{
	case GeomAbs_SurfaceType::GeomAbs_Plane: faceDir1 = sur.Plane().Axis().Direction(); break;
	case GeomAbs_SurfaceType::GeomAbs_Cylinder: {
		Handle(Geom_Line) centralLine = new Geom_Line(sur.Cylinder().Axis());
		GeomAPI_ProjectPointOnCurve tool(pickedPoint1, centralLine);
		if (tool.NbPoints() < 1) return;
		gp_Vec lineToPickpnt(pickedPoint1.XYZ() - tool.Point(1).XYZ());
		faceDir1 = lineToPickpnt.Normalized();
		break;
	}
	case GeomAbs_SurfaceType::GeomAbs_Sphere: {
		gp_Pnt spherePnt = sur.Sphere().Location();
		gp_Vec spherePntToPickpnt(pickedPoint1.XYZ() - spherePnt.XYZ());
		faceDir1 = spherePntToPickpnt.Normalized();
		break;
	}
	default:return;
	}
	if (face.Orientation() == TopAbs_Orientation::TopAbs_REVERSED)
		faceDir1.Reverse();
	Geom_Plane plane1 = Geom_Plane(pickedPoint1, faceDir1);

	TopoDS_Shape faceShape2 = getSubElementShape(
		static_cast<ViewProviderPartExt*>(sel2.selections.at(0).vp)->getShape(),
		sel2.selections.at(0).subObjectName.c_str());
	TopoDS_Face face2 = TopoDS::Face(faceShape2);
	BRepAdaptor_Surface sur2(face2);
	gp_Dir faceDir2;
	switch (sur2.GetType())
	{
	case GeomAbs_SurfaceType::GeomAbs_Plane: faceDir2 = sur2.Plane().Axis().Direction(); break;
	case GeomAbs_SurfaceType::GeomAbs_Cylinder: {
		Handle(Geom_Line) centralLine = new Geom_Line(sur2.Cylinder().Axis());
		GeomAPI_ProjectPointOnCurve tool(pickedPoint1, centralLine);
		if (tool.NbPoints() < 1) return;
		gp_Vec lineToPickpnt(pickedPoint1.XYZ() - tool.Point(1).XYZ());
		faceDir2 = lineToPickpnt.Normalized();
		break;
	}
	case GeomAbs_SurfaceType::GeomAbs_Sphere: {
		gp_Pnt spherePnt = sur2.Sphere().Location();
		gp_Vec spherePntToPickpnt(pickedPoint1.XYZ() - spherePnt.XYZ());
		faceDir2 = spherePntToPickpnt.Normalized();
		break;
	}
	default:return;
	}
	if (face2.Orientation() == TopAbs_Orientation::TopAbs_REVERSED)
		faceDir2.Reverse();
	Geom_Plane plane2(pickedPoint2, faceDir2);

	const Handle(Geom_Surface) pplane1 = new Geom_Plane(plane1);
	const Handle(Geom_Surface) pplane2 = new Geom_Plane(plane2);
	GeomAPI_IntSS comm(pplane1, pplane2, 0.01);
	auto numofline = comm.NbLines();
	if (numofline < 1) return;
	auto crossline = comm.Line(1);

	Handle(Geom_Curve) p2p = new Geom_Line(
								pickedPoint1,
								gp_Vec(	sel2.selections.at(0).x - sel1.selections.at(0).x,
										sel2.selections.at(0).y - sel1.selections.at(0).y,
										sel2.selections.at(0).z - sel1.selections.at(0).z));

	GeomAPI_ExtremaCurveCurve extrema(crossline, p2p);
	if (extrema.NbExtrema() < 1) return;
	gp_Pnt extreme1, extreme2;
	extrema.Points(1,extreme1, extreme2);
	//从拾取点连线指向标记曲线的圆心的方向
	gp_Vec pointToOrigin(extreme1.XYZ() - extreme2.XYZ());
	pointToOrigin.Normalize();
	//夹角值
	auto f2fAngle = faceDir1.Angle(faceDir2);
	f2fAngle = 3.1415926f - f2fAngle;
	//圆心到直线的偏移值
	double delta = (pickedPoint1.Distance(pickedPoint2) / 2) * tan((3.1415926f - f2fAngle) / 2);
	//拾取点连线中点
	gp_Pnt middleOfPickedPoint((pickedPoint1.XYZ() + pickedPoint2.XYZ()) / 2);
	gp_Trsf trsfToCirOrigin;
	trsfToCirOrigin.SetTranslation(pointToOrigin * delta);
	//标记曲线的圆心
	gp_Pnt cirOrigin = middleOfPickedPoint.Transformed(trsfToCirOrigin);

	//build adapters.
	VectorAdapter adapt1 = buildAdapter(sel1);
	VectorAdapter adapt2 = buildAdapter(sel2);

	if (!adapt1.isValid() || !adapt2.isValid())
	{
		return;
	}
	_Measures.emplace_back(sel1, sel2, false);

	goDimensionAngularNoTask_new(adapt1, adapt2, cirOrigin, f2fAngle);
}

void TaskMeasureAngular::clearSelection()
{
	selections1.selections.clear();
	selections2.selections.clear();
}

void TaskMeasureAngular::setUpGui()
{
	View3DInventor* viewer = getInventor();
	stepped = new SteppedSelection(2, viewer);
	DimensionControl *control = new DimensionControl(viewer);

	QObject::connect(stepped->getButton(0), SIGNAL(toggled(bool)), this, SLOT(selection1Slot(bool)));
	QObject::connect(stepped->getButton(1), SIGNAL(toggled(bool)), this, SLOT(selection2Slot(bool)));
	viewer->setEditing(true);
	viewer->setEditingCursor(QCursor(QPixmap(cursor_angular), 7, 6));
}

void TaskMeasureAngular::selection1Slot(bool checked)
{
	if (checked)
	{
		buttonSelectedIndex = 0;
		std::vector<DimSelections::DimSelection>::const_iterator it;
		for (it = selections1.selections.begin(); it != selections1.selections.end(); ++it)
			Selection().addSelection(it->vp, it->subObjectName.c_str());
	}
}

void TaskMeasureAngular::selection2Slot(bool checked)
{
	if (checked)
		buttonSelectedIndex = 1;
	std::vector<DimSelections::DimSelection>::const_iterator it;
	for (it = selections2.selections.begin(); it != selections2.selections.end(); ++it)
		Selection().addSelection(it->vp, it->subObjectName.c_str());
}

void TaskMeasureAngular::resetDialogSlot(bool)
{
	clearSelection();
	this->blockConnection(true);
	Selection().clearSelection();
	this->blockConnection(false);
}

void TaskMeasureAngular::toggle3dSlot(bool)
{
	toggle3d();
}

void TaskMeasureAngular::toggleDeltaSlot(bool)
{
}

void TaskMeasureAngular::clearAllSlot(bool)
{
	eraseAllDimensions();
}

QDialogButtonBox::StandardButtons TaskMeasureAngular::getStandardButtons() const
{
	return QDialogButtonBox::Close;
}

bool TaskMeasureAngular::isAllowedAlterDocument(void) const 
{ 
	return false; 
}

bool TaskMeasureAngular::needsFullSpace() const 
{ 
	return false; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SO_KIT_SOURCE(DimensionAngular)

void DimensionAngular::initClass()
{
	SO_KIT_INIT_CLASS(DimensionAngular, SoSeparatorKit, "SeparatorKit");
}

DimensionAngular::DimensionAngular()
{
	SO_KIT_CONSTRUCTOR(DimensionAngular);

	SO_KIT_ADD_CATALOG_ENTRY(transformation, SoMatrixTransform, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(annotate, SoAnnotation, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(arrow1, SoShapeKit, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(arrow2, SoShapeKit, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(arcSep, SoSeparator, true, annotate, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(textSep, SoSeparator, true, annotate, "", true);

	SO_KIT_INIT_INSTANCE();

	SO_NODE_ADD_FIELD(radius, (10.0));
	SO_NODE_ADD_FIELD(angle, (1.0));
	SO_NODE_ADD_FIELD(text, ("test"));//dimension text
	SO_NODE_ADD_FIELD(dColor, (1.0, 0.0, 0.0));//dimension color.
	SO_NODE_ADD_FIELD(matrix, (1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0));
}

DimensionAngular::~DimensionAngular()
{

}

SbBool DimensionAngular::affectsState() const
{
	return false;
}


void DimensionAngular::setupDimension()
{
	//transformation
	SoMatrixTransform *trans = static_cast<SoMatrixTransform *>(getPart("transformation", true));
	trans->matrix.connectFrom(&matrix);

	//color
	SoMaterial *material = new SoMaterial;
	material->ref();
	material->diffuseColor.connectFrom(&dColor);

	// calculate arrow positions
	SoTransform *arrow1Transform = new SoTransform();
	SoComposeVec3f *arrow1Compose = new SoComposeVec3f();
	arrow1Compose->x.connectFrom(&radius);
	arrow1Compose->y.setValue(0.0f);
	arrow1Compose->z.setValue(0.0f);
	const float* constFloat = arrow1Compose->x.getValues(0);
	auto PositionX1 = *constFloat;
	arrow1Transform->translation.connectFrom(&arrow1Compose->vector);
	setPart("arrow1.transform", arrow1Transform);

	SoComposeRotation *arrow2Rotation = new SoComposeRotation();
	arrow2Rotation->angle.connectFrom(&angle); 
	arrow2Rotation->axis.setValue(0.0f, 0.0f, 1.0f);
	SoTransform *arrow2Transform = new SoTransform();
	arrow2Transform->rotation.connectFrom(&arrow2Rotation->rotation);
	SoCalculator *arrow2LocationCalc = new SoCalculator();
	arrow2LocationCalc->a.connectFrom(&angle);
	arrow2LocationCalc->b.connectFrom(&radius);
	arrow2LocationCalc->expression.set1Value(0, "oa = cos(a) * b"); //xLocation
	arrow2LocationCalc->expression.set1Value(1, "ob = sin(a) * b"); //yLocation
	SoComposeVec3f *arrow2Compose = new SoComposeVec3f();
	arrow2Compose->x.connectFrom(&arrow2LocationCalc->oa);
	arrow2Compose->y.connectFrom(&arrow2LocationCalc->ob);
	arrow2Compose->z.setValue(0.0f);
	arrow2Transform->translation.connectFrom(&arrow2Compose->vector);

	// calculate distance between the 2 arrows
	constFloat = arrow2Compose->x.getValues(0);
	auto PositionX2 = *constFloat;
	constFloat = arrow2Compose->y.getValues(0);
	auto PositionY2 = *constFloat;
	float distance = sqrt((PositionX2 - PositionX1) * (PositionX2 - PositionX1) + PositionY2 * PositionY2);

	// dimension arrows
	// the cone size must be scaled with the distance
	// we use the same factors as for linear dimensions
	float coneHeight = distance * 0.06;
	float coneRadius = coneHeight * 0.3;

	SoCone* cone = new SoCone();
	cone->bottomRadius.setValue(coneRadius);
	cone->height.setValue(coneHeight);

	// set arrows, their precision and rotation
	char str1[100];
	char str2[100];
	snprintf(str1, sizeof(str1), "translation 0.0 %.6f 0.0", coneHeight * 0.5);
	snprintf(str2, sizeof(str2), "translation 0.0 -%.6f 0.0", coneHeight * 0.5);

	setPart("arrow1.shape", cone);
	set("arrow1.localTransform", "rotation 0.0 0.0 1.0 3.1415927");
	set("arrow1.localTransform", str1);
	setPart("arrow2.shape", cone);
	set("arrow2.transform", "rotation 0.0 0.0 1.0 0.0");
	set("arrow2.localTransform", str2);

	// now the position
	setPart("arrow1.transform", arrow1Transform);
	setPart("arrow2.transform", arrow2Transform);

	// finally the material
	setPart("arrow1.material", material);
	setPart("arrow2.material", material);

	ArcEngine *arcEngine = new ArcEngine();
	arcEngine->angle.connectFrom(&angle);
	arcEngine->radius.connectFrom(&radius);
	arcEngine->deviation.setValue(0.1f);

	SoCoordinate3 *coordinates = new SoCoordinate3();
	coordinates->point.connectFrom(&arcEngine->points);

	SoLineSet *lineSet = new SoLineSet();
	lineSet->ref();
	lineSet->vertexProperty.setValue(coordinates);
	lineSet->numVertices.connectFrom(&arcEngine->pointCount);
	lineSet->startIndex.setValue(0);

	SoSeparator *arcSep = static_cast<SoSeparator *>(getPart("arcSep", true));
	if (arcSep) {
		arcSep->addChild(material);
		arcSep->addChild(lineSet);
	}

	//text
	SoSeparator *textSep = static_cast<SoSeparator *>(getPart("textSep", true));
	if (!textSep)
		return;

	textSep->addChild(material);

	SoCalculator *textVecCalc = new SoCalculator();
	textVecCalc->a.connectFrom(&angle);
	textVecCalc->b.connectFrom(&radius);
	textVecCalc->expression.set1Value(0, "oa = a / 2.0");
	textVecCalc->expression.set1Value(1, "ob = cos(oa) * b"); //x
	textVecCalc->expression.set1Value(2, "oc = sin(oa) * b"); //y

	SoComposeVec3f *textLocation = new SoComposeVec3f();
	textLocation->x.connectFrom(&textVecCalc->ob);
	textLocation->y.connectFrom(&textVecCalc->oc);
	textLocation->z.setValue(0.0);


	SoTransform *textTransform = new SoTransform();
	textTransform->translation.connectFrom(&textLocation->vector);
	textSep->addChild(textTransform);

	SoFont *fontNode = new SoFont();
	fontNode->name.setValue("default font");
	fontNode->size.setValue(40);
	textSep->addChild(fontNode);

	SoText2 *textNode = new SoText2();
	textNode->justification = SoText2::CENTER;
	textNode->string.connectFrom(&text);
	textSep->addChild(textNode);

	//this prevents the 2d text from screwing up the bounding box for a viewall
	SoResetTransform *rTrans = new SoResetTransform;
	rTrans->whatToReset = SoResetTransform::BBOX;
	textSep->addChild(rTrans);

	lineSet->unref();
	material->unref();
}

void DimensionAngular::setupDimension(gp_Trsf picked1, gp_Trsf picked2, SbVec3f origin)
{
	NCollection_Mat4<float> mat;
	picked1.GetMat4(mat);
	SoSFMatrix somat;
	somat.setValue(
		mat[0], mat[1], mat[2], mat[3],
		mat[4], mat[5], mat[6], mat[7],
		mat[8], mat[9], mat[10], mat[11],
		mat[12], mat[13], mat[14], mat[15]);
	SoMatrixTransform *trans = static_cast<SoMatrixTransform *>(getPart("transformation", true));
	trans->matrix.connectFrom(&somat);

	auto pickPnt1 = picked1.TranslationPart();
	auto pickPnt2 = picked2.TranslationPart();
	auto dirPnt1 = picked1.GetRotation();
	auto dirPnt2 = picked2.GetRotation();

	//color
	SoMaterial *material = new SoMaterial;
	material->ref();
	material->diffuseColor.connectFrom(&dColor);

	float coneHeight = (SbVec3f(pickPnt1.X(), pickPnt1.Y(), pickPnt1.Z()) - origin).length() * 0.06;
	float coneRadius = coneHeight * 0.3;

	SoCone* cone = new SoCone();
	cone->bottomRadius.setValue(coneRadius);
	cone->height.setValue(coneHeight);

	// set arrows, their precision and rotation
	char str1[100];
	char str2[100];
	snprintf(str1, sizeof(str1), "translation 0.0 %.6f 0.0", coneHeight * 0.5);
	snprintf(str2, sizeof(str2), "translation 0.0 -%.6f 0.0", coneHeight * 0.5);

	setPart("arrow1.shape", cone);
	set("arrow1.localTransform", "rotation 0.0 0.0 1.0 3.1415927");
	set("arrow1.localTransform", str1);

	setPart("arrow2.shape", cone);
	set("arrow2.transform", "rotation 0.0 0.0 1.0 0.0");
	set("arrow2.localTransform", str2);

	SoTransform *arrow1Transform = new SoTransform();
	SoComposeVec3f *arrow1Compose = new SoComposeVec3f();
	arrow1Compose->x.setValue(pickPnt1.X());
	arrow1Compose->y.setValue(pickPnt1.Y());
	arrow1Compose->z.setValue(pickPnt1.Z());
	arrow1Transform->translation.connectFrom(&arrow1Compose->vector);
	SoComposeRotation* arrow1Rot = new SoComposeRotation();
	gp_Vec axis;
	Standard_Real theAngle;
	dirPnt1.GetVectorAndAngle(axis, theAngle);
	arrow1Rot->angle.setValue(theAngle);
	arrow1Rot->axis.setValue(axis.X(), axis.Y(), axis.Z());
	arrow1Transform->rotation.connectFrom(&arrow1Rot->rotation);
	setPart("arrow1.transform", arrow1Transform);

	SoTransform *arrow2Transform = new SoTransform();
	SoComposeVec3f *arrow2Compose = new SoComposeVec3f();
	arrow2Compose->x.setValue(pickPnt2.X());
	arrow2Compose->y.setValue(pickPnt2.Y());
	arrow2Compose->z.setValue(pickPnt2.Z());
	arrow2Transform->translation.connectFrom(&arrow2Compose->vector);
	SoComposeRotation* arrow2Rot = new SoComposeRotation();
	gp_Vec axis2;
	Standard_Real theAngle2;
	dirPnt2.GetVectorAndAngle(axis2, theAngle2);
	arrow2Rot->angle.setValue(theAngle2);
	arrow2Rot->axis.setValue(axis2.X(), axis2.Y(), axis2.Z());
	arrow2Transform->rotation.connectFrom(&arrow2Rot->rotation);
	setPart("arrow2.transform", arrow2Transform);


	SbVec3f p;
	p.setValue(picked1.TranslationPart().X(), picked1.TranslationPart().Y(), picked1.TranslationPart().Z());
	radius.setValue((p - origin).length());
	ArcEngine *arcEngine = new ArcEngine();
	arcEngine->angle.connectFrom(&angle);
	arcEngine->radius.connectFrom(&radius);
	arcEngine->deviation.setValue(0.1f);

	SoCoordinate3 *coordinates = new SoCoordinate3();
	coordinates->point.connectFrom(&arcEngine->points);

	SoLineSet *lineSet = new SoLineSet();
	lineSet->ref();
	lineSet->vertexProperty.setValue(coordinates);
	lineSet->numVertices.connectFrom(&arcEngine->pointCount);
	lineSet->startIndex.setValue(0);

	SoSeparator *arcSep = static_cast<SoSeparator *>(getPart("arcSep", true));
	if (arcSep) {
		arcSep->addChild(material);
		arcSep->addChild(lineSet);
	}

	//text
	SoSeparator *textSep = static_cast<SoSeparator *>(getPart("textSep", true));
	if (!textSep)
		return;

	textSep->addChild(material);

	SoCalculator *textVecCalc = new SoCalculator();
	textVecCalc->a.connectFrom(&angle);
	textVecCalc->b.connectFrom(&radius);
	textVecCalc->expression.set1Value(0, "oa = a / 2.0");
	textVecCalc->expression.set1Value(1, "ob = cos(oa) * b"); //x
	textVecCalc->expression.set1Value(2, "oc = sin(oa) * b"); //y

	SoComposeVec3f *textLocation = new SoComposeVec3f();
	textLocation->x.connectFrom(&textVecCalc->ob);
	textLocation->y.connectFrom(&textVecCalc->oc);
	textLocation->z.setValue(0.0);


	SoTransform *textTransform = new SoTransform();
	textTransform->translation.connectFrom(&textLocation->vector);
	textSep->addChild(textTransform);

	SoFont *fontNode = new SoFont();
	fontNode->name.setValue("default font");
	fontNode->size.setValue(40);
	textSep->addChild(fontNode);

	SoText2 *textNode = new SoText2();
	textNode->justification = SoText2::CENTER;
	textNode->string.connectFrom(&text);
	textSep->addChild(textNode);

	//this prevents the 2d text from screwing up the bounding box for a viewall
	SoResetTransform *rTrans = new SoResetTransform;
	rTrans->whatToReset = SoResetTransform::BBOX;
	textSep->addChild(rTrans);

	lineSet->unref();
	material->unref();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SO_ENGINE_SOURCE(ArcEngine)

ArcEngine::ArcEngine()
{
	SO_ENGINE_CONSTRUCTOR(ArcEngine);

	SO_ENGINE_ADD_INPUT(radius, (10.0));
	SO_ENGINE_ADD_INPUT(angle, (1.0));
	SO_ENGINE_ADD_INPUT(deviation, (0.25));

	SO_ENGINE_ADD_OUTPUT(points, SoMFVec3f);
	SO_ENGINE_ADD_OUTPUT(pointCount, SoSFInt32);
}

void ArcEngine::initClass()
{
	SO_ENGINE_INIT_CLASS(ArcEngine, SoEngine, "Engine");
}

void ArcEngine::evaluate()
{
	if (radius.getValue() < std::numeric_limits<float>::epsilon() ||
		angle.getValue() < std::numeric_limits<float>::epsilon() ||
		deviation.getValue() < std::numeric_limits<float>::epsilon())
	{
		defaultValues();
		return;
	}

	float deviationAngle(acos((radius.getValue() - deviation.getValue()) / radius.getValue()));
	std::vector<SbVec3f> tempPoints;
	int segmentCount;
	if (deviationAngle >= angle.getValue())
		segmentCount = 1;
	else
	{
		segmentCount = static_cast<int>(angle.getValue() / deviationAngle) + 1;
		if (segmentCount < 2)
		{
			defaultValues();
			return;
		}
	}
	float angleIncrement = angle.getValue() / static_cast<float>(segmentCount);
	for (int index = 0; index < segmentCount + 1; ++index)
	{
		SbVec3f currentNormal(1.0, 0.0, 0.0);
		float currentAngle = index * angleIncrement;
		SbRotation rotation(SbVec3f(0.0, 0.0, 1.0), currentAngle);
		rotation.multVec(currentNormal, currentNormal);
		tempPoints.push_back(currentNormal * radius.getValue());
	}
	int tempCount = tempPoints.size(); //for macro.
	SO_ENGINE_OUTPUT(points, SoMFVec3f, setNum(tempCount));
	SO_ENGINE_OUTPUT(pointCount, SoSFInt32, setValue(tempCount));
	std::vector<SbVec3f>::const_iterator it;
	for (it = tempPoints.begin(); it != tempPoints.end(); ++it)
	{
		int currentIndex = it - tempPoints.begin(); //for macro.
		SbVec3f temp(*it); //for macro
		SO_ENGINE_OUTPUT(points, SoMFVec3f, set1Value(currentIndex, temp));
	}

}

void ArcEngine::defaultValues()
{
	//just some non-failing info.
	SO_ENGINE_OUTPUT(points, SoMFVec3f, setNum(2));
	SbVec3f point1(10.0, 0.0, 0.0);
	SO_ENGINE_OUTPUT(points, SoMFVec3f, set1Value(0, point1));
	SbVec3f point2(7.07f, 7.07f, 0.0);
	SO_ENGINE_OUTPUT(points, SoMFVec3f, set1Value(1, point2));
	SO_ENGINE_OUTPUT(pointCount, SoSFInt32, setValue(2));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskMeasureArea::TaskMeasureArea()
	: SelectionObserver(true, false), selection()
{
	button = new AbstractButton(this);
	View3DInventor* viewer = getInventor();
	viewer->setEditing(true);
	viewer->setEditingCursor(QCursor(QPixmap(cursor_area), 7, 6));
}

void TaskMeasureArea::onSelectionChanged(const SelectionChanges& msg)
{
	if (!msg.vp) return;
	if (msg.Type == SelectionChanges::AddSelection)
	{
		TopoDS_Shape shape = static_cast<ViewProviderPartExt*>(msg.vp)->getShape();
		shape = getSubElementShape(shape, msg.pSubName);
		if (shape.IsNull()) return;
		if (shape.ShapeType() != TopAbs_ShapeEnum::TopAbs_FACE) return;
		Vector3d vec(msg.x, msg.y, msg.z);
		buildDimension(shape, vec);
		//Selection().clearSelection();
	}
}

void TaskMeasureArea::buildDimension(TopoDS_Shape shape, Vector3d pickedPoint)
{
	GProp_GProps props;
	BRepGProp::SurfaceProperties(shape, props);
	double area = props.Mass();//取得面积
	TopoDS_Face face = TopoDS::Face(shape);
	if (face.Orientation() == TopAbs_REVERSED) face.Reverse();
	BRepGProp_Face analysisFace(face);
	Standard_Real umin, umax, vmin, vmax;
	analysisFace.Bounds(umin, umax, vmin, vmax);//获取拓扑面的参数范围
	Standard_Real midU, midV;
	midU = (umax - umin) / 2;//拓扑面的参数中点
	midV = (vmax - vmin) / 2;
	gp_Vec norm;
	gp_Pnt midPoint;
	analysisFace.Normal(midU, midV, midPoint, norm);
	norm.Normalize();
	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(pickedPoint.x, pickedPoint.y, pickedPoint.z), gp_Dir(1.0,0.0,0.0)), 0.78539815);
	norm.Transform(trsf);
	Standard_Real delta;
	if (midU > 0 && midV > 0) delta = midU > midV ? midU : midV;
	else if (midU > 0 && midV < 0) delta = midU > -midV ? midU : -midV;
	else if (midU < 0 && midV > 0) delta = -midU > midV ? -midU : midV;
	else if (midU < 0 && midV < 0) delta = -midU > -midV ? -midU : -midV;
	norm.Scale(delta);
	gp_Pnt newPoint(pickedPoint.x, pickedPoint.y, pickedPoint.z);
	gp_Trsf trsf2;
	trsf2.SetTranslation(norm);
	newPoint.Transform(trsf2);
	DimensionArea* dimension = new DimensionArea();
	dimension->point2.setValue(pickedPoint.x, pickedPoint.y, pickedPoint.z);
	dimension->point1.setValue(newPoint.X(), newPoint.Y(), newPoint.Z());
	dimension->area.setValue(area);
	dimension->dColor.setValue(0.0, 0.86, 0.41);
	dimension->setupDimension();

	QString value = getPrintValue(area, 2);
	dimension->text.setValue(value.toStdString().c_str());

	View3DInventor* viewer = getInventor();
	if (!viewer)
		return;
	viewer->addDimension3d(dimension);
}

TaskMeasureArea::~TaskMeasureArea()
{
	View3DInventor* viewer = getInventor();
	if (viewer) {
		viewer->setEditing(false);
		viewer->setEditingCursor(QCursor(Qt::ArrowCursor));
	}
	delete button;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SO_KIT_SOURCE(DimensionArea)

void DimensionArea::initClass()
{
	SO_KIT_INIT_CLASS(DimensionArea, SoSeparatorKit, "SeparatorKit");
}

DimensionArea::DimensionArea()
{
	SO_KIT_CONSTRUCTOR(DimensionArea);

	SO_KIT_ADD_CATALOG_ENTRY(transformation, SoTransform, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(annotate, SoAnnotation, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(Arrow, SoShapeKit, true, topSeparator, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(line, SoShapeKit, true, annotate, "", true);
	SO_KIT_ADD_CATALOG_ENTRY(textSep, SoSeparator, true, annotate, "", true);

	SO_KIT_INIT_INSTANCE();

	SO_NODE_ADD_FIELD(rotate, (1.0, 0.0, 0.0, 0.0));//position orientation of the dimension.
	SO_NODE_ADD_FIELD(length, (1.0));//turns into dimension length
	SO_NODE_ADD_FIELD(origin, (0.0, 0.0, 0.0));//static
	SO_NODE_ADD_FIELD(text, ("test"));//dimension text
	SO_NODE_ADD_FIELD(dColor, (1.0, 0.0, 0.0));//dimension color.
}

DimensionArea::~DimensionArea()
{

}

SbBool DimensionArea::affectsState() const
{
	return false;
}

void DimensionArea::setupDimension()
{
	//transformation
	SoTransform *trans = static_cast<SoTransform *>(getPart("transformation", true));
	trans->translation.connectFrom(&point1);
	//build engine for vector subtraction and length.
	SoCalculator *hyp = new SoCalculator();
	hyp->A.connectFrom(&point1);
	hyp->B.connectFrom(&point2);
	hyp->expression.set1Value(0, "oA = B-A");
	hyp->expression.set1Value(1, "oB = normalize(oA)");
	hyp->expression.set1Value(2, "oa = length(oA)");
	length.connectFrom(&hyp->oa);

	//build engine for rotation.
	SoComposeRotationFromTo *rotationEngine = new SoComposeRotationFromTo();
	rotationEngine->from.setValue(SbVec3f(1.0, 0.0, 0.0));
	rotationEngine->to.connectFrom(&hyp->oB);
	trans->rotation.connectFrom(&rotationEngine->rotation);

	//color
	SoMaterial *material = new SoMaterial;
	material->diffuseColor.connectFrom(&dColor);

	//dimension arrows
	float dimLength = (point2.getValue() - point1.getValue()).length();
	float coneHeight = dimLength * 0.06;
	float coneRadius = coneHeight * 0.2;

	SoCone *cone = new SoCone();
	cone->bottomRadius.setValue(coneRadius);
	cone->height.setValue(coneHeight);

	char lStr[100];
	char rStr[100];
	snprintf(lStr, sizeof(lStr), "translation %.6f 0.0 0.0", coneHeight * 0.5);
	snprintf(rStr, sizeof(rStr), "translation 0.0 -%.6f 0.0", coneHeight * 0.5);

	setPart("Arrow.shape", cone);
	set("Arrow.transform", "rotation 0.0 0.0 -1.0 1.5707963");
	set("Arrow.localTransform", rStr);

	SoTransform *transform = static_cast<SoTransform *>(getPart("Arrow.transform", false));
	if (!transform)
		return;//what to do here?
	SoComposeVec3f *vec = new SoComposeVec3f;
	vec->x.connectFrom(&length);
	vec->y.setValue(0.0);
	vec->z.setValue(0.0);
	transform->translation.connectFrom(&vec->vector);

	setPart("Arrow.material", material);

	//line
	SoConcatenate *catEngine = new SoConcatenate(SoMFVec3f::getClassTypeId());
	//don't know how to get around having this dummy origin. cat engine wants to connectfrom?
	catEngine->input[0]->connectFrom(&origin);
	catEngine->input[1]->connectFrom(&vec->vector);

	SoVertexProperty *lineVerts = new SoVertexProperty;
	lineVerts->vertex.connectFrom(catEngine->output);

	int lineVertexMap[] = { 0, 1 };
	int lineVertexMapSize(sizeof(lineVertexMap) / sizeof(int));
	SoIndexedLineSet *line = new SoIndexedLineSet;
	line->vertexProperty = lineVerts;
	line->coordIndex.setValues(0, lineVertexMapSize, lineVertexMap);

	setPart("line.shape", line);
	setPart("line.material", material);

	//text
	SoSeparator *textSep = static_cast<SoSeparator *>(getPart("textSep", true));
	if (!textSep)
		return;

	textSep->addChild(material);

	SoCalculator *textVecCalc = new SoCalculator();
	textVecCalc->A.connectFrom(&vec->vector);
	textVecCalc->B.set1Value(0, 0.0, 0.250, 0.0);
	textVecCalc->expression.set1Value(0, "oA = B");

	SoTransform *textTransform = new SoTransform();
	textTransform->translation.connectFrom(&textVecCalc->oA);
	textSep->addChild(textTransform);

	SoFont *fontNode = new SoFont();
	fontNode->name.setValue(" ");
	fontNode->size.setValue(40);
	textSep->addChild(fontNode);

	SoText2 *textNode = new SoText2();
	textNode->justification = SoText2::CENTER;
	textNode->string.connectFrom(&text);
	textSep->addChild(textNode);

	//this prevents the 2d text from screwing up the bounding box for a viewall
	SoResetTransform *rTrans = new SoResetTransform;
	rTrans->whatToReset = SoResetTransform::BBOX;
	textSep->addChild(rTrans);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskMeasureRadius::TaskMeasureRadius()
	: SelectionObserver(true, false)
{
	View3DInventor* viewer = getInventor();
	viewer->setEditing(true);
	viewer->setEditingCursor(QCursor(QPixmap(cursor_radius), 7, 6));
}

void TaskMeasureRadius::onSelectionChanged(const SelectionChanges& msg)
{
	if (!msg.vp) return;
	if (msg.Type == SelectionChanges::AddSelection)
	{
		TopoDS_Shape shape = static_cast<ViewProviderPartExt*>(msg.vp)->getShape();
		shape = getSubElementShape(shape, msg.pSubName);
		if (shape.IsNull()) return;
		if (shape.ShapeType() == TopAbs_ShapeEnum::TopAbs_EDGE)
		{
			auto edge = TopoDS::Edge(shape);
			BRepAdaptor_Curve curve(edge);
			if (curve.GetType() == GeomAbs_CurveType::GeomAbs_Circle)
			{
				auto r = curve.Circle().Radius();
				auto o = curve.Circle().Location();
				gp_Pnt pickPnt(msg.x, msg.y, msg.z);
				buildDimension(o, pickPnt, r);
				return;
			}
		}
	}
	Selection().clearSelection();
}

void TaskMeasureRadius::buildDimension(const gp_Pnt o, const gp_Pnt pnt, const Standard_Real R)
{
	DimensionLinear *dimension = new DimensionLinear();
	dimension->point1.setValue(SbVec3f(o.X(), o.Y(), o.Z()));
	dimension->point2.setValue(SbVec3f(pnt.X(), pnt.Y(), pnt.Z()));
	dimension->setupDimension();

	//写入测量值
	dimension->text.setValue(getPrintValue(static_cast<double>(R), 0).toStdString().c_str());
	dimension->dColor.setValue(1.0f,0.0f,1.0f);
	View3DInventor* viewer = getInventor();
	if (viewer)
	{
		viewer->addDimension3d(dimension);
	}
}

TaskMeasureRadius::~TaskMeasureRadius()
{
	View3DInventor* viewer = getInventor();
	if (viewer) {
		viewer->setEditing(false);
		viewer->setEditingCursor(QCursor(Qt::ArrowCursor));
	}
}

View3DInventor* TaskMeasure::getInventor()
{
	auto window = QApplication::activeWindow();
	if (MainViewer* mainViewer = dynamic_cast<MainViewer*>(window)) {
		return mainViewer->getInventor();
	}
	return nullptr;
}