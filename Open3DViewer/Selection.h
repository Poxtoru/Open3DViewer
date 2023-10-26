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
#ifndef SELECTION_H
#define SELECTION_H

#include <string>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <boost_signals2.hpp>
#include <Open3DViewer/Observer.h>
#include <Open3DViewer\DLLConfig.h>

class ViewProvider;

class DLLExport strCache
{
public:
	strCache() {};
	strCache(const char* str):subName(str) {};
	std::string subName;
};

class DLLExport SelectionChanges
{
public:
	enum MsgType {
		AddSelection,
		RmvSelection,
		SetSelection,
		ClrSelection,
		SetPreselect, // to signal observer the preselect has changed
		RmvPreselect,
		SetPreselectSignal, // to request 3D view to change preselect
		PickedListChanged,
		ShowSelection, // to show a selection
		HideSelection, // to hide a selection
		RmvPreselectSignal, // to request 3D view to remove preselect
		MovePreselect, // to signal observer the mouse movement when preselect
	};

	SelectionChanges(MsgType type = ClrSelection,
		ViewProvider* vpr = 0,
		const char *subName = 0, 
		float x = 0, float y = 0, float z = 0, int subtype = 0)
		: Type(type), SubType(subtype)
		, x(x), y(y), z(z)
	{
		vp = vpr;
		pSubName = subName;
	}

	SelectionChanges(MsgType type,
		ViewProvider* vpr,
		const std::string &subName,
		float x = 0, float y = 0, float z = 0, int subtype = 0)
		: Type(type), SubType(subtype)
		, x(x), y(y), z(z),
		str(subName.c_str())
	{
		vp = vpr;
		pSubName = str.subName.c_str();
	}

	SelectionChanges &operator=(const SelectionChanges &other) {
		Type = other.Type;
		SubType = other.SubType;
		x = other.x;
		y = other.y;
		z = other.z;
		TypeName = other.TypeName;
		vp = other.vp;
		pSubName = other.pSubName;
		pOriginalMsg = other.pOriginalMsg;
		return *this;
	}

	SelectionChanges(const SelectionChanges &other) {
		*this = other;
	}

	MsgType Type;
	int SubType;
	ViewProvider* vp;
	const char* pSubName;
	float x;
	float y;
	float z;
	strCache str;
	std::string TypeName;

	// Original selection message in case resolve!=0
	const SelectionChanges *pOriginalMsg = 0;
};

/** SelectionGate
 * The selection gate allows or disallows selection of certain types.
 * It has to be registered to the selection.
 */
class DLLExport SelectionGate
{
public:
	virtual ~SelectionGate() {}
	virtual bool allow(const char*) = 0;

	/**
	 * @brief notAllowedReason is a string that sets the message to be
	 * displayed in statusbar for cluing the user on why is the selection not
	 * allowed. Set this variable in allow() implementation. Enclose the
	 * literal into QT_TR_NOOP() for translatability.
	 */
	std::string notAllowedReason;
};

/** SelectionGateFilterExternal
 * The selection gate disallows any external object
 */
class DLLExport SelectionGateFilterExternal : public SelectionGate
{
public:
	SelectionGateFilterExternal(const char *docName, const char *objName = 0);
	virtual bool allow(const char*) override;
private:
	std::string DocName;
	std::string ObjName;
};

class DLLExport SelectionObserver
{
public:
	SelectionObserver(bool attach = true, int resolve = 1);
	SelectionObserver(const ViewProvider *vp, bool attach = true, int resolve = 1);

	virtual ~SelectionObserver();

	bool blockConnection(bool block);
	bool isConnectionBlocked() const;
	bool isConnectionAttached() const;

	/** Attaches to the selection. */
	void attachSelection();
	/** Detaches from the selection. */
	void detachSelection();

private:
	virtual void onSelectionChanged(const SelectionChanges& msg) = 0;
	void _onSelectionChanged(const SelectionChanges& msg);

private:
	typedef boost::signals2::connection Connection;
	Connection connectSelection;
	std::string filterDocName;
	std::string filterObjName;
	int resolve;
	bool blockSelection;
};

class DLLExport SelectionSingleton : public Subject<const SelectionChanges&>
{
public:
	struct SelObj {
		const char* SubName;
		ViewProvider* vp;
		float x, y, z;
	};

	struct _SelObj {
		ViewProvider* vp;
		std::string SubName;
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		//bool logged = false;

		std::pair<std::string, std::string> elementName;
	};
	mutable std::list<_SelObj> _SelList;
	mutable std::list<_SelObj> _PickedList;
	/// Add to selection
	bool addSelection(ViewProvider* vp, const char* pSubName = 0,
		float x = 0, float y = 0, float z = 0, const std::vector<SelObj> *pickedList = 0, bool clearPreSelect = true);
	//bool addSelection2(const char* pDocName, const char* pObjectName = 0, const char* pSubName = 0,
	//	float x = 0, float y = 0, float z = 0)
	//{
	//	return addSelection(pDocName, pObjectName, pSubName, x, y, z, false);
	//}
	bool isSelected(ViewProvider* vp, const char* pSubName = 0, int resolve = 1) const;
	int checkSelection(ViewProvider* vp, const char *pSubName, int resolve, _SelObj &sel, const std::list<_SelObj> *selList = 0) const;
	void clearCompleteSelection(bool clearPreSelect = true);

	/// signal on new object
	boost::signals2::signal<void(const SelectionChanges& msg)> signalSelectionChanged;

	/// signal on selection change with resolved object
	boost::signals2::signal<void(const SelectionChanges& msg)> signalSelectionChanged2;
	/// signal on selection change with resolved object and sub element map
	boost::signals2::signal<void(const SelectionChanges& msg)> signalSelectionChanged3;

	static SelectionSingleton& instance(void);
	static void destruct(void);
	void addSelectionGate(SelectionGate *gate, int resolve = 1);
	void rmvSelectionGate(void);
	void rmvPreselect(bool signal = false);
	int setPreselect(ViewProvider* vp, const char* pSubName, float x, float y, float z, int signal=0);
	bool needPickedList() const;
	void rmvSelection(ViewProvider* vp, const char* pSubName = 0,
		const std::vector<SelObj> *pickedList = 0);
	const char *getSelectedElement(ViewProvider*, const char* pSubName) const;
	void clearSelection(ViewProvider* vp = 0, bool clearPreSelect = true);
	void setSelection(ViewProvider* vp, const std::vector<ViewProvider*>&);
	std::vector<SelObj> getSelection(bool single = false) const;

protected:
	/// Construction
	SelectionSingleton();
	/// Destruction
	virtual ~SelectionSingleton();

	void slotSelectionChanged(const SelectionChanges& msg);

	SelectionChanges CurrentPreselection;
	bool _needPickedList;
	static SelectionSingleton* _pcSingleton;
	void notify(SelectionChanges &&Chng);
	void notify(const SelectionChanges &Chng) { notify(SelectionChanges(Chng)); }

	float hx, hy, hz;
	SelectionGate *ActiveGate;
	int gateResolve;
	std::deque<SelectionChanges> NotificationQueue;
	bool Notifying = false;


	ViewProvider* viewprovider;
	std::string SubName;
};



/// Get the global instance
inline SelectionSingleton& Selection(void)
{
	return SelectionSingleton::instance();
}



#endif // !SELECTION_H
