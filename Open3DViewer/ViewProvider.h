/***************************************************************************
 *   Copyright (c) 2004 J¨¹rgen Riegel <juergen.riegel@web.de>              *
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

#ifndef VIEWPROVIDER_H
#define VIEWPROVIDER_H

#include <map>
#include <vector>
#include <string>
#include <bitset>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <QColor>
#include <gp_Trsf.hxx>
#include <Open3DViewer\DLLConfig.h>
#include <Open3DViewer/BaseClass.h>
#include <Open3DViewer/Placement.h>

class SbVec2s;
class SbVec3f;
class SoNode;
class SoPath;
class SoSeparator;
class SoEvent;
class SoSwitch;
class SoTransform;
class SbMatrix;
class SoEventCallback;
class SoPickedPoint;
class SoDetail;
class SoFullPath;
class QString;
class QMenu;
class QObject;

enum ViewStatus {
	UpdateData = 0,
	Detach = 1,
	isRestoring = 2,
	UpdatingView = 3,
	TouchDocument = 4,
};

void DLLExport coinRemoveAllChildren(SoGroup *node);

class DLLExport ViewProvider : public Base::BaseClass
{
public:
	/// constructor.
	ViewProvider();
	/// destructor.
	virtual ~ViewProvider();

	/// Adds a new display mask mode
	void addDisplayMaskMode(SoNode *node, const char* type);
	/// Activates the display mask mode \a type
	void setDisplayMaskMode(const char* type);
	/// Get the node to the display mask mode \a type
	SoNode* getDisplayMaskMode(const char* type) const;
	/// Returns a list of added display mask modes
	std::vector<std::string> getDisplayMaskModes() const;
	void setDefaultMode(int);
	//void setStatus(ViewStatus pos, bool on) { StatusBits.set((size_t)pos, on); }
	virtual void setRenderCacheMode(int);
	virtual SoSeparator* getRoot(void) const;
	virtual bool isSelectable(void) const;
	virtual std::string getElement(const SoDetail *) const;
	virtual bool getElementPicked(const SoPickedPoint *, std::string &subname) const;
	virtual SoDetail* getDetail(const char *) const;
	virtual bool getDetailPath(const char *subname, SoFullPath *pPath, bool append, SoDetail *&det) const;
	virtual void hide(void);
	virtual void setOverrideMode(const std::string &mode);
	virtual void setShapeColor(double r, double g, double b);
	void updateTransform(const Placement& from);
	const gp_Trsf getTransform();
	void visible();
	bool isVisible();
	virtual void setFaceColor(const int idx, const QColor color);
protected:
	/// The root Separator of the ViewProvider
	SoSeparator *pcRoot;
	/// this is transformation for the provider
	SoTransform *pcTransform;
	const char* sPixmap;
	/// this is the mode switch, all the different viewing modes are collected here
	SoSwitch    *pcModeSwitch;
	/// The root separator for annotations
	SoSeparator *pcAnnotation;
	std::string overrideMode;
	//std::bitset<32> StatusBits;
	virtual void setModeSwitch();
	


private:
	int _iActualMode;
	int _iEditMode;
	int viewOverrideMode;
	std::string _sCurrentMode;
	std::map<std::string, int> _sDisplayMaskModes;
	int lastMode;
};

#endif