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

#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <gp_Quaternion.hxx>
#include <Open3DViewer\ViewProvider.h>
#include <Open3DViewer\SoFCUnifiedSelection.h>

void coinRemoveAllChildren(SoGroup *group) {
	if (!group)
		return;
	int count = group->getNumChildren();
	if (!count)
		return;
	SbBool autonotify = group->enableNotify(FALSE);
	for (; count > 0; --count)
		group->removeChild(count - 1);
	group->enableNotify(autonotify);
	group->touch();
}


ViewProvider::ViewProvider()
	: pcAnnotation(0)
	, overrideMode("As Is")
	, _iActualMode(-1)
	, _iEditMode(-1)
	, viewOverrideMode(-1)
	, lastMode(-1)
{
	//setStatus(UpdateData, true);

	pcRoot = new SoSelectionRoot(true);
	pcRoot->ref();
	pcModeSwitch = new SoSwitch();
	pcModeSwitch->ref();
	pcTransform = new SoTransform();
	pcTransform->ref();
	pcRoot->addChild(pcTransform);
	pcRoot->addChild(pcModeSwitch);
	//sPixmap = "px";
	pcModeSwitch->whichChild = _iActualMode;
	//pcRoot->addChild(new SoCone);
	setRenderCacheMode(0);
}

void ViewProvider::setRenderCacheMode(int mode) 
{
	pcRoot->renderCaching =
		mode == 0 ? SoSeparator::AUTO : (mode == 1 ? SoSeparator::ON : SoSeparator::OFF);
}

bool ViewProvider::getElementPicked(const SoPickedPoint *pp, std::string &subname) const {
	if (!isSelectable()) return false;
	subname = getElement(pp ? pp->getDetail() : 0);
	return true;
}

void ViewProvider::hide(void)
{
	if (pcModeSwitch->whichChild.getValue() >= 0) {
		lastMode = pcModeSwitch->whichChild.getValue();
		pcModeSwitch->whichChild = -1;
	}
}

void ViewProvider::visible()
{
	if (pcModeSwitch->whichChild.getValue() == -1)
	{
		pcModeSwitch->whichChild = lastMode;
	}
}

bool ViewProvider::getDetailPath(const char *subname, SoFullPath *pPath, bool append, SoDetail *&det) const {
	if (pcRoot->findChild(pcModeSwitch) < 0) {
		return false;
	}
	if (append) {
		pPath->append(pcRoot);
		pPath->append(pcModeSwitch);
	}
	det = getDetail(subname);
	return true;
}

void ViewProvider::setOverrideMode(const std::string &mode)
{
	if (mode == "As Is") {
		viewOverrideMode = -1;
		overrideMode = mode;
	}
	else {
		std::map<std::string, int>::const_iterator it = _sDisplayMaskModes.find(mode);
		if (it == _sDisplayMaskModes.end())
			return; //view style not supported
		viewOverrideMode = (*it).second;
		overrideMode = mode;
	}
	if (mode == "As Is")
		lastMode = 0;
	else
		lastMode = viewOverrideMode;
	if (pcModeSwitch->whichChild.getValue() != -1)
		setModeSwitch();
}

void ViewProvider::setModeSwitch()
{
	if (viewOverrideMode == -1)
		pcModeSwitch->whichChild = _iActualMode;
	else if (viewOverrideMode < pcModeSwitch->getNumChildren())
	{
		pcModeSwitch->whichChild = viewOverrideMode;
		//_iActualMode = viewOverrideMode;
	}
	else
		return;
}

void ViewProvider::setDefaultMode(int val)
{
	_iActualMode = val;
}

void ViewProvider::updateTransform(const Placement& from)
{
	float q0 = (float)from.getRotation().getValue()[0];
	float q1 = (float)from.getRotation().getValue()[1];
	float q2 = (float)from.getRotation().getValue()[2];
	float q3 = (float)from.getRotation().getValue()[3];
	float px = (float)from.getPosition().x;
	float py = (float)from.getPosition().y;
	float pz = (float)from.getPosition().z;
	pcTransform->rotation.setValue(q0, q1, q2, q3);
	pcTransform->translation.setValue(px, py, pz);
	pcTransform->center.setValue(0.0f, 0.0f, 0.0f);
	pcTransform->scaleFactor.setValue(1.0f, 1.0f, 1.0f);
}

const gp_Trsf ViewProvider::getTransform()
{
	float q0, q1, q2, q3;
	q0 = pcTransform->rotation.getValue().getValue()[0];
	q1 = pcTransform->rotation.getValue().getValue()[1];
	q2 = pcTransform->rotation.getValue().getValue()[2];
	q3 = pcTransform->rotation.getValue().getValue()[3];
	float px, py, pz;
	px = pcTransform->translation.getValue().getValue()[0];
	py = pcTransform->translation.getValue().getValue()[1];
	pz = pcTransform->translation.getValue().getValue()[2];
	gp_Trsf trsf;
	trsf.SetRotation(gp_Quaternion(q0,q1,q2,q3));
	trsf.SetTranslation(gp_Vec(px, py, pz));
	return trsf;
}

void ViewProvider::addDisplayMaskMode(SoNode *node, const char* type)
{
	_sDisplayMaskModes[type] = pcModeSwitch->getNumChildren();
	pcModeSwitch->addChild(node);
}

void ViewProvider::setDisplayMaskMode(const char* type)
{
	std::map<std::string, int>::const_iterator it = _sDisplayMaskModes.find(type);
	if (it != _sDisplayMaskModes.end())
		_iActualMode = it->second;
	else
		_iActualMode = -1;
	setModeSwitch();
}

SoNode* ViewProvider::getDisplayMaskMode(const char* type) const
{
	std::map<std::string, int>::const_iterator it = _sDisplayMaskModes.find(type);
	if (it != _sDisplayMaskModes.end()) {
		return pcModeSwitch->getChild(it->second);
	}

	return 0;
}

std::vector<std::string> ViewProvider::getDisplayMaskModes() const
{
	std::vector<std::string> types;
	for (std::map<std::string, int>::const_iterator it = _sDisplayMaskModes.begin();
		it != _sDisplayMaskModes.end(); ++it)
		types.push_back(it->first);
	return types;
}

SoSeparator* ViewProvider::getRoot(void) const 
{ 
	return pcRoot; 
}

bool ViewProvider::isSelectable(void) const 
{ 
	return true; 
}

std::string ViewProvider::getElement(const SoDetail *) const 
{ 
	return std::string(); 
}

SoDetail* ViewProvider::getDetail(const char *) const 
{ 
	return 0; 
}

void ViewProvider::setShapeColor(double r, double g, double b) 
{ 
	this->setShapeColor(r, g, b); 
}

bool ViewProvider::isVisible() 
{ 
	return !(pcModeSwitch->whichChild.getValue() == -1); 
}

void ViewProvider::setFaceColor(const int idx, const QColor color) 
{ 
	this->setFaceColor(idx, color); 
}


ViewProvider::~ViewProvider()
{
	coinRemoveAllChildren(pcRoot);
	pcRoot->unref();
	pcTransform->unref();
	pcModeSwitch->unref();
	if (pcAnnotation)
		pcAnnotation->unref();
}