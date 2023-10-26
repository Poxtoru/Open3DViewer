/***************************************************************************
 *   Copyright (c) 2011 J¨¹rgen Riegel <juergen.riegel@web.de>              *
 *   Copyright (c) 2011 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
#include <assert.h>
#include <string>
#include <boost/algorithm/string/predicate.hpp>
#include <boost_bind_bind.hpp>
#include <Open3DViewer\Selection.h>
#include <Open3DViewer\Tools.h>
#include <Open3DViewer\ViewProvider.h>
#include <Open3DViewer\View3DInventor.h>

namespace bp = boost::placeholders;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SelectionGateFilterExternal::SelectionGateFilterExternal(const char *docName, const char *objName) {
	if (docName) {
		DocName = docName;
		if (objName)
			ObjName = objName;
	}
}

bool SelectionGateFilterExternal::allow(const char*) {
	if (DocName.size())
		notAllowedReason = "Cannot select external object";
	else if (ObjName.size() )
		notAllowedReason = "Cannot select self";
	else
		return true;
	return false;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SelectionObserver::SelectionObserver(bool attach, int resolve)
	:resolve(resolve), blockSelection(false)
{
	if (attach)
		attachSelection();
}

SelectionObserver::SelectionObserver(const ViewProvider *vp, bool attach, int resolve)
	:resolve(resolve), blockSelection(false)
{
	if (attach)
		attachSelection();
}


SelectionObserver::~SelectionObserver()
{
	detachSelection();
}

bool SelectionObserver::blockConnection(bool block)
{
	bool ok = blockSelection;
	if (block)
		blockSelection = true;
	else
		blockSelection = false;
	return ok;
}

bool SelectionObserver::isConnectionBlocked() const
{
	return blockSelection;
}

bool SelectionObserver::isConnectionAttached() const
{
	return connectSelection.connected();
}

void SelectionObserver::attachSelection()
{
	if (!connectSelection.connected()) {
		auto &signal = resolve > 1 ? Selection().signalSelectionChanged3 :
			resolve ? Selection().signalSelectionChanged2 :
			Selection().signalSelectionChanged;
		connectSelection = signal.connect(boost::bind
		(&SelectionObserver::_onSelectionChanged, this, bp::_1));

		if (filterDocName.size()) {
			Selection().addSelectionGate(
				new SelectionGateFilterExternal(filterDocName.c_str(), filterObjName.c_str()));
		}
	}
}

void SelectionObserver::_onSelectionChanged(const SelectionChanges& msg) {
	try {
		if (blockSelection)
			return;
		onSelectionChanged(msg);
	}
	catch (...) {
	}
}

void SelectionObserver::detachSelection()
{
	if (connectSelection.connected()) {
		connectSelection.disconnect();
		if (filterDocName.size())
			Selection().rmvSelectionGate();
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//**************************************************************************
// Construction/Destruction

/**
 * A constructor.
 * A more elaborate description of the constructor.
 */
SelectionSingleton::SelectionSingleton()
	:CurrentPreselection(SelectionChanges::ClrSelection)
	, _needPickedList(false)
{
	hx = 0;
	hy = 0;
	hz = 0;
	ActiveGate = 0;
	gateResolve = 1;
	
	signalSelectionChanged.connect(boost::bind(&SelectionSingleton::slotSelectionChanged, this, bp::_1));
}

/**
 * A destructor.
 * A more elaborate description of the destructor.
 */
SelectionSingleton::~SelectionSingleton()
{
}

SelectionSingleton* SelectionSingleton::_pcSingleton = NULL;

SelectionSingleton& SelectionSingleton::instance(void)
{
	if (_pcSingleton == NULL)
		_pcSingleton = new SelectionSingleton;
	return *_pcSingleton;
}

void SelectionSingleton::destruct(void)
{
	if (_pcSingleton != NULL)
		delete _pcSingleton;
	_pcSingleton = 0;
}

void SelectionSingleton::slotSelectionChanged(const SelectionChanges& msg) {
	if (msg.Type == SelectionChanges::SetPreselectSignal ||
		msg.Type == SelectionChanges::ShowSelection ||
		msg.Type == SelectionChanges::HideSelection)
		return;

	if (msg.pSubName != "") {
		if (!msg.vp) return;
		SelectionChanges msg2(msg.Type, msg.vp,	msg.pSubName, msg.x, msg.y, msg.z);
		try {
			msg2.pOriginalMsg = &msg;
			signalSelectionChanged3(msg2);
			signalSelectionChanged2(msg2);
		}
		catch (...) {
		}
	}
	else {
		try {
			signalSelectionChanged3(msg);
			signalSelectionChanged2(msg);
		}
		catch (...) {
		}
	}
}

// add a SelectionGate to control what is selectable
void SelectionSingleton::addSelectionGate(SelectionGate *gate, int resolve)
{
	if (ActiveGate)
		rmvSelectionGate();

	ActiveGate = gate;
	gateResolve = resolve;
}

void SelectionSingleton::rmvSelectionGate(void)
{
	if (ActiveGate) {
		delete ActiveGate;
		ActiveGate = nullptr;

		//View3DInventor::getInventor()->restoreOverrideCursor();
	}
}

int SelectionSingleton::setPreselect(ViewProvider* vp, const char* pSubName, float x, float y, float z, int signal)
{
	if (!vp)
	{
		rmvPreselect();
		return 0;
	}

	if (!pSubName) pSubName = "";

	if (/*SubName == pSubName*/vp == viewprovider) {
		// MovePreselect is likely going to slow down large scene rendering.
		// Disable it for now.
#if 0
		if (hx != x || hy != y || hz != z) {
			hx = x;
			hy = y;
			hz = z;
			SelectionChanges Chng(SelectionChanges::MovePreselect,
				DocName, FeatName, SubName, std::string(), x, y, z);
			notify(Chng);
		}
#endif
		return -1;
	}

	rmvPreselect();

	if (ActiveGate && signal != 1) {
		//if (!View3DInventor::getInventor())
		//	return 0;
		//std::pair<std::string, std::string> elementName;

		//const char *subelement = pSubName;
		//if (gateResolve) {
		//	auto &newElementName = elementName.first;
		//	auto &oldElementName = elementName.second;
		//	pObject = App::GeoFeature::resolveElement(pObject, pSubName, elementName);
		//	if (!pObject)
		//		return 0;
		//	if (gateResolve > 1)
		//		subelement = newElementName.size() ? newElementName.c_str() : oldElementName.c_str();
		//	else
		//		subelement = oldElementName.c_str();
		//}
		//if (!ActiveGate->allow(pObject->getDocument(), pObject, subelement)) {
		//	return 0;
		//}
		//View3DInventor::getInventor()->restoreOverrideCursor();
	}

	SubName = pSubName;
	hx = x;
	hy = y;
	hz = z;

	// set up the change object
	SelectionChanges Chng(signal == 1 ? SelectionChanges::SetPreselectSignal : SelectionChanges::SetPreselect,
		vp, SubName, x, y, z, signal);

	if (Chng.Type == SelectionChanges::SetPreselect) {
		CurrentPreselection = Chng;
	}

	notify(Chng);

	if (signal == 1 && vp) {
		Chng.Type = SelectionChanges::SetPreselect;
		CurrentPreselection = Chng;
		notify(std::move(Chng));
	}

	// It is possible the preselect is removed during notification
	return vp ? 1 : 0;
}

void SelectionSingleton::rmvPreselect(bool signal)
{
	//if (DocName == "")
	//	return;

	if (signal) {
		SelectionChanges Chng(SelectionChanges::RmvPreselectSignal, viewprovider, SubName);
		notify(std::move(Chng));
		return;
	}

	SelectionChanges Chng(SelectionChanges::RmvPreselect, viewprovider, SubName);

	// reset the current preselection
	CurrentPreselection = SelectionChanges();

	viewprovider = 0;
	SubName = "";
	hx = 0;
	hy = 0;
	hz = 0;

	//if (ActiveGate && View3DInventor::getInventor()) {
	//	View3DInventor::getInventor()->restoreOverrideCursor();
	//}


	// notify observing objects
	notify(std::move(Chng));

}

bool SelectionSingleton::isSelected(ViewProvider* vp, const char* pSubName, int resolve) const
{
	_SelObj sel;
	return checkSelection(vp, pSubName, resolve, sel, &_SelList) > 0;
}

int SelectionSingleton::checkSelection(ViewProvider* vp, const char *pSubName, int resolve,
	_SelObj &sel, const std::list<_SelObj> *selList) const
{
	//sel.pDoc = getDocument(pDocName);
	//if (!sel.pDoc) {
	//	if (!selList)
	//		FC_ERR("Cannot find document");
	//	return -1;
	//}
	//pDocName = sel.pDoc->getName();
	//sel.DocName = pDocName;
	//
	//if (pObjectName)
	//	sel.pObject = sel.pDoc->getObject(pObjectName);
	//else
	//	sel.pObject = 0;
	//if (!sel.pObject) {
	//	if (!selList)
	//		FC_ERR("Object not found");
	//	return -1;
	//}
	//if (sel.pObject->testStatus(App::ObjectStatus::Remove))
	//	return -1;
	if (!vp) return -1;
	sel.vp = vp;
	if (pSubName)
		sel.SubName = pSubName;
	pSubName = sel.SubName.size() ? sel.SubName.c_str() : 0;
	if (!selList)
		selList = &_SelList;

	if (!pSubName)
		pSubName = "";

	for (auto &s : *selList) {
		if (s.vp == vp) {
			if (s.SubName == pSubName)
				return 1;
			if (resolve > 1 /*&& boost::starts_with(s.SubName, prefix)*/)
				return 1;
		}
	}
	if (resolve == 1) {
		for (auto &s : *selList) {
			if (s.vp != sel.vp)
				continue;
			if (!pSubName[0])
				return 1;
			if (s.elementName.first.size()) {
				if (s.elementName.first == sel.elementName.first)
					return 1;
			}
			else if (s.SubName == sel.elementName.second)
				return 1;
		}
	}
	return 0;
}

bool SelectionSingleton::needPickedList() const {
	return _needPickedList;
}

bool SelectionSingleton::addSelection(ViewProvider* vp,	const char* pSubName, float x, float y, float z,
	const std::vector<SelObj> *pickedList, bool clearPreselect)
{
	if (pickedList) {
		_PickedList.clear();
		for (const auto &sel : *pickedList) {
			_PickedList.emplace_back();
			auto &s = _PickedList.back();
			s.vp = sel.vp;
			s.SubName = sel.SubName;
			s.x = sel.x;
			s.y = sel.y;
			s.z = sel.z;
		}
		notify(SelectionChanges(SelectionChanges::PickedListChanged));
	}

	_SelObj temp;
	int ret = checkSelection(vp, pSubName, 0, temp);
	if (ret != 0)
		return false;

	temp.x = x;
	temp.y = y;
	temp.z = z;

	_SelList.push_back(temp);

	if (clearPreselect)
		rmvPreselect();

	SelectionChanges Chng(SelectionChanges::AddSelection,
		temp.vp, temp.SubName, x, y, z);

	notify(std::move(Chng));

	//getMainWindow()->updateActions();

	rmvPreselect(true);

	// There is a possibility that some observer removes or clears selection
	// inside signal handler, hence the check here
	return isSelected(temp.vp, temp.SubName.c_str());
}

void SelectionSingleton::notify(SelectionChanges &&Chng) {
	if (Notifying) {
		NotificationQueue.push_back(std::move(Chng));
		return;
	}
	Base::FlagToggler<bool> flag(Notifying);
	NotificationQueue.push_back(std::move(Chng));
	while (NotificationQueue.size()) {
		const auto &msg = NotificationQueue.front();
		bool notify;
		switch (msg.Type) {
		case SelectionChanges::AddSelection:
			notify = isSelected(msg.vp, msg.pSubName, 0);
			break;
		case SelectionChanges::RmvSelection:
			notify = !isSelected(msg.vp, msg.pSubName, 0);
			break;
		case SelectionChanges::SetPreselect:
			notify = CurrentPreselection.Type == SelectionChanges::SetPreselect
				&& CurrentPreselection.vp == msg.vp;
			break;
		case SelectionChanges::RmvPreselect:
			notify = CurrentPreselection.Type == SelectionChanges::ClrSelection;
			break;
		default:
			notify = true;
		}
		if (notify) {
			Notify(msg);
			try {
				signalSelectionChanged(msg);
			}
			catch (...) {
				// reported by code analyzers
				//Base::Console().Warning("notify: Unexpected boost exception\n");
			}
		}
		NotificationQueue.pop_front();
	}
}

void SelectionSingleton::rmvSelection(ViewProvider* vp, const char* pSubName,
	const std::vector<SelObj> *pickedList)
{
	if (pickedList) {
		_PickedList.clear();
		for (const auto &sel : *pickedList) {
			_PickedList.emplace_back();
			auto &s = _PickedList.back();
			s.vp = sel.vp;
			s.SubName = sel.SubName;
			s.x = sel.x;
			s.y = sel.y;
			s.z = sel.z;
		}
		notify(SelectionChanges(SelectionChanges::PickedListChanged));
	}

	if (!vp) return;

	_SelObj temp;
	int ret = checkSelection(vp, pSubName, 0, temp);
	if (ret < 0)
		return;

	std::vector<SelectionChanges> changes;
	for (auto It = _SelList.begin(), ItNext = It; It != _SelList.end(); It = ItNext) {
		++ItNext;
		if (It->vp != temp.vp)
			continue;
		// if no subname is specified, remove all subobjects of the matching object
		if (temp.SubName.size()) {
			// otherwise, match subojects with common prefix, separated by '.'
			if (!boost::starts_with(It->SubName, temp.SubName) ||
				(It->SubName.length() != temp.SubName.length() && It->SubName[temp.SubName.length() - 1] != '.'))
				continue;
		}

		//It->log(true);

		changes.emplace_back(SelectionChanges::RmvSelection, It->vp, It->SubName);

		// destroy the _SelObj item
		_SelList.erase(It);
	}

	// NOTE: It can happen that there are nested calls of rmvSelection()
	// so that it's not safe to invoke the notifications inside the loop
	// as this can invalidate the iterators and thus leads to undefined
	// behaviour.
	// So, the notification is done after the loop, see also #0003469
	if (changes.size()) {
		for (auto &Chng : changes) {
			//FC_LOG("Rmv Selection " << Chng.pDocName << '#' << Chng.pObjectName << '.' << Chng.pSubName);
			notify(std::move(Chng));
		}
		//getMainWindow()->updateActions();
	}
}

const char *SelectionSingleton::getSelectedElement(ViewProvider *vp, const char* pSubName) const
{
	if (!vp) return 0;

	for (std::list<_SelObj>::const_iterator It = _SelList.begin(); It != _SelList.end(); ++It) {
		if (It->vp == vp) {
			auto len = It->SubName.length();
			if (!len)
				return "";
			if (pSubName && strncmp(pSubName, It->SubName.c_str(), It->SubName.length()) == 0) {
				if (pSubName[len] == 0 || pSubName[len - 1] == '.')
					return It->SubName.c_str();
			}
		}
	}
	return 0;
}

void SelectionSingleton::clearSelection(ViewProvider* vp,bool clearPreSelect)
{
	// Because the introduction of external editing, it is best to make
	// clearSelection(0) behave as clearCompleteSelection(), which is the same
	// behavior of python Selection.clearSelection(None)
	if (/*!pDocName || !pDocName[0] || strcmp(pDocName, "*")*/vp == 0) {
		clearCompleteSelection(clearPreSelect);
		return;
	}

	if (_PickedList.size()) {
		_PickedList.clear();
		notify(SelectionChanges(SelectionChanges::PickedListChanged));
	}

	if (clearPreSelect)
		rmvPreselect();

	bool touched = false;
	for (auto it = _SelList.begin(); it != _SelList.end();) {
		if (it->vp == vp) {
			touched = true;
			it = _SelList.erase(it);
		}
		else {
			++it;
		}
	}

	if (!touched)
		return;

	notify(SelectionChanges(SelectionChanges::ClrSelection, vp));
}

void SelectionSingleton::setSelection(ViewProvider* vp, const std::vector<ViewProvider*>& sel)
{
	if (!vp)
		return;

	if (_PickedList.size()) {
		_PickedList.clear();
		notify(SelectionChanges(SelectionChanges::PickedListChanged));
	}

	bool touched = false;
	for (auto eachvp : sel) {
		if (!eachvp)
			continue;
		_SelObj temp;
		int ret = checkSelection(eachvp, 0, 0, temp);
		if (ret != 0)
			continue;
		touched = true;
		_SelList.push_back(temp);
	}

	if (touched) {
		//_SelStackForward.clear();
		notify(SelectionChanges(SelectionChanges::SetSelection, vp));
		//getMainWindow()->updateActions();
	}
}

void SelectionSingleton::clearCompleteSelection(bool clearPreSelect)
{
	if (_PickedList.size()) {
		_PickedList.clear();
		notify(SelectionChanges(SelectionChanges::PickedListChanged));
	}

	if (clearPreSelect)
		rmvPreselect();

	if (_SelList.empty())
		return;

	//if (!logDisabled)
	//	Application::Instance->macroManager()->addLine(MacroManager::Cmt,
	//		clearPreSelect ? "Gui.Selection.clearSelection()"
	//		: "Gui.Selection.clearSelection(False)");

	_SelList.clear();

	SelectionChanges Chng(SelectionChanges::ClrSelection);

	//FC_LOG("Clear selection");

	notify(std::move(Chng));
	//getMainWindow()->updateActions();
}

std::vector<SelectionSingleton::SelObj> SelectionSingleton::getSelection(bool single) const
{
	std::vector<SelObj> temp;
	if (single) temp.reserve(1);
	SelObj tempSelObj;

	std::map<ViewProvider*, std::set<std::string> > objMap;

	for (auto &sel : _SelList) {
		if (single && temp.size()) {
			temp.clear();
			break;
		}
		tempSelObj.vp = sel.vp;
		tempSelObj.SubName = sel.SubName.c_str();
		tempSelObj.x = sel.x;
		tempSelObj.y = sel.y;
		tempSelObj.z = sel.z;

		temp.push_back(tempSelObj);
	}

	return temp;
}