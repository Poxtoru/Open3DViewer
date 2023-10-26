/***************************************************************************
 *   Copyright (c) 2013 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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


#ifndef GUI_GLPAINTER_H
#define GUI_GLPAINTER_H
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <Open3DViewer\DLLConfig.h>
#include <Open3DViewer/View3DInventor.h>
#include <Open3DViewer/BaseClass.h>
#include <Open3DViewer/QtOpenGL.h>
#include <QPoint>

class QPaintDevice;
class View3DInventor;

//class ViewInevntor;
class GLPainter
{
public:
    GLPainter();
    virtual ~GLPainter();

    bool begin(QPaintDevice * device);
    bool end();
    bool isActive() const;

    /** @name Setter methods */
    //@{
    void setLineWidth(float);
    void setPointSize(float);
    void setColor(float, float, float, float=0);
    void setLogicOp(GLenum);
    void resetLogicOp();
    void setDrawBuffer(GLenum);
    void setLineStipple(GLint factor, GLushort pattern);
    void resetLineStipple();
    //@}

    /** @name Draw routines */
    //@{
    void drawRect (int x, int y, int w, int h);
    void drawLine (int x1, int y1, int x2, int y2);
    void drawPoint(int x, int y);
    //@}

private:
    QtGLWidget* viewer;
    GLfloat depthrange[2];
    GLdouble projectionmatrix[16];
    GLint width, height;
    bool logicOp;
    bool lineStipple;
};

class GLGraphicsItem : public Base::BaseClass
{
    TYPESYSTEM_HEADER();

public:
    GLGraphicsItem()
    {
    }
    virtual ~GLGraphicsItem()
    {
    }
    virtual void paintGL() = 0;
};

class Rubberband : public GLGraphicsItem
{
    View3DInventor* viewer;
    int x_old, y_old, x_new, y_new;
    float rgb_r, rgb_g, rgb_b, rgb_a;
    bool working, stipple;

public:
    Rubberband(View3DInventor* v);
    Rubberband();
    ~Rubberband();
    void setWorking(bool on);
    void setLineStipple(bool on);
    //bool isWorking();
    void setViewer(View3DInventor* v);
    void setCoords(int x1, int y1, int x2, int y2);
    void setColor(float r, float g, float b, float a);
    void paintGL();
};

namespace Gui {

	class Polyline : public GLGraphicsItem
	{
		View3DInventor* viewer;
		std::vector<QPoint> _cNodeVector;
		int x_new, y_new;
		float rgb_r, rgb_g, rgb_b, rgb_a, line;
		bool working, closed, stippled;
		GLPainter p;

	public:
		Polyline(View3DInventor* v);
		Polyline();
		~Polyline();
		void setWorking(bool on);
		bool isWorking() const;
		void setViewer(View3DInventor* v);
		void setCoords(int x, int y);
		void setColor(int r, int g, int b, int a = 0);
		void setLineWidth(float l);
		void setClosed(bool c);
		void setCloseStippled(bool c);
		void addNode(const QPoint& p);
		void popNode();
		void clear();
		void paintGL();
	};

}
#endif  // GUI_GLPAINTER_H

