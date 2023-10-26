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

#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoTranslation.h>
#include <QtWidgets/QGraphicsPixmapItem>
#include <QBitmap>
#include <QException>
#include <QMessageBox>
#include <QEventLoop>
#include <QTimer>
#include <Open3DViewer\View3DInventor.h>
#include <Open3DViewer\SoFCUnifiedSelection.h>
#include <Open3DViewer\ViewProvider.h>
#include <Open3DViewer\SoFCBoundingBox.h>
#include <Open3DViewer\SoFCUnifiedSelection.h>
#include <Open3DViewer\SoFCSelectionAction.h>
#include <Open3DViewer\GestureNavigationStyle.h>
#include <Open3DViewer\SoMouseWheelEvent.h>

 /*** zoom-style cursor ******/

#define ZOOM_WIDTH 16
#define ZOOM_HEIGHT 16
#define ZOOM_BYTES ((ZOOM_WIDTH + 7) / 8) * ZOOM_HEIGHT
#define ZOOM_HOT_X 5
#define ZOOM_HOT_Y 7

static unsigned char zoom_bitmap[ZOOM_BYTES] =
{
  0x00, 0x0f, 0x80, 0x1c, 0x40, 0x38, 0x20, 0x70,
  0x90, 0xe4, 0xc0, 0xcc, 0xf0, 0xfc, 0x00, 0x0c,
  0x00, 0x0c, 0xf0, 0xfc, 0xc0, 0xcc, 0x90, 0xe4,
  0x20, 0x70, 0x40, 0x38, 0x80, 0x1c, 0x00, 0x0f
};

static unsigned char zoom_mask_bitmap[ZOOM_BYTES] =
{
 0x00,0x0f,0x80,0x1f,0xc0,0x3f,0xe0,0x7f,0xf0,0xff,0xf0,0xff,0xf0,0xff,0x00,
 0x0f,0x00,0x0f,0xf0,0xff,0xf0,0xff,0xf0,0xff,0xe0,0x7f,0xc0,0x3f,0x80,0x1f,
 0x00,0x0f
};

/*** pan-style cursor *******/

#define PAN_WIDTH 16
#define PAN_HEIGHT 16
#define PAN_BYTES ((PAN_WIDTH + 7) / 8) * PAN_HEIGHT
#define PAN_HOT_X 7
#define PAN_HOT_Y 7

static unsigned char pan_bitmap[PAN_BYTES] =
{
  0xc0, 0x03, 0x60, 0x02, 0x20, 0x04, 0x10, 0x08,
  0x68, 0x16, 0x54, 0x2a, 0x73, 0xce, 0x01, 0x80,
  0x01, 0x80, 0x73, 0xce, 0x54, 0x2a, 0x68, 0x16,
  0x10, 0x08, 0x20, 0x04, 0x40, 0x02, 0xc0, 0x03
};

static unsigned char pan_mask_bitmap[PAN_BYTES] =
{
 0xc0,0x03,0xe0,0x03,0xe0,0x07,0xf0,0x0f,0xe8,0x17,0xdc,0x3b,0xff,0xff,0xff,
 0xff,0xff,0xff,0xff,0xff,0xdc,0x3b,0xe8,0x17,0xf0,0x0f,0xe0,0x07,0xc0,0x03,
 0xc0,0x03
};

/*** rotate-style cursor ****/

#define ROTATE_WIDTH 16
#define ROTATE_HEIGHT 16
#define ROTATE_BYTES ((ROTATE_WIDTH + 7) / 8) * ROTATE_HEIGHT
#define ROTATE_HOT_X 6
#define ROTATE_HOT_Y 8

static unsigned char rotate_bitmap[ROTATE_BYTES] = {
  0xf0, 0xef, 0x18, 0xb8, 0x0c, 0x90, 0xe4, 0x83,
  0x34, 0x86, 0x1c, 0x83, 0x00, 0x81, 0x00, 0xff,
  0xff, 0x00, 0x81, 0x00, 0xc1, 0x38, 0x61, 0x2c,
  0xc1, 0x27, 0x09, 0x30, 0x1d, 0x18, 0xf7, 0x0f
};

static unsigned char rotate_mask_bitmap[ROTATE_BYTES] = {
 0xf0,0xef,0xf8,0xff,0xfc,0xff,0xfc,0xff,0x3c,0xfe,0x1c,0xff,0x00,0xff,0x00,
 0xff,0xff,0x00,0xff,0x00,0xff,0x38,0x7f,0x3c,0xff,0x3f,0xff,0x3f,0xff,0x1f,
 0xf7,0x0f
};

class ViewerEventFilter : public QWidget
{
public:
	ViewerEventFilter() {}
	~ViewerEventFilter() {}

	bool eventFilter(QObject* obj, QEvent* event) {

#ifdef GESTURE_MESS
		if (obj->isWidgetType()) {
			View3DInventorViewer* v = dynamic_cast<View3DInventorViewer*>(obj);
			if (v) {
				/* Internally, Qt seems to set up the gestures upon showing the
				 * widget (but after this event is processed), thus invalidating
				 * our settings. This piece takes care to retune gestures on the
				 * next event after the show event.
				 */
				if (v->winGestureTuneState == View3DInventorViewer::ewgtsNeedTuning) {
					try {
						WinNativeGestureRecognizerPinch::TuneWindowsGestures(v);
						v->winGestureTuneState = View3DInventorViewer::ewgtsTuned;
					}
					catch (Base::Exception &e) {
						Base::Console().Warning("Failed to TuneWindowsGestures. Error: %s\n", e.what());
						v->winGestureTuneState = View3DInventorViewer::ewgtsDisabled;
					}
					catch (...) {
						Base::Console().Warning("Failed to TuneWindowsGestures. Unknown error.\n");
						v->winGestureTuneState = View3DInventorViewer::ewgtsDisabled;
					}
				}
				if (event->type() == QEvent::Show && v->winGestureTuneState == View3DInventorViewer::ewgtsTuned)
					v->winGestureTuneState = View3DInventorViewer::ewgtsNeedTuning;

			}
		}
#endif

		// Bug #0000607: Some mice also support horizontal scrolling which however might
		// lead to some unwanted zooming when pressing the MMB for panning.
		// Thus, we filter out horizontal scrolling.
		if (event->type() == QEvent::Wheel) {
			QWheelEvent* we = static_cast<QWheelEvent*>(event);
#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
			if (qAbs(we->angleDelta().x()) > qAbs(we->angleDelta().y()))
				return true;
#else
			if (we->orientation() == Qt::Horizontal)
				return true;
#endif
		}
		else if (event->type() == QEvent::KeyPress) {
			QKeyEvent* ke = static_cast<QKeyEvent*>(event);
			if (ke->matches(QKeySequence::SelectAll)) {
				static_cast<View3DInventor*>(obj)->selectAll();
				return true;
			}
		}
		else if (event->type() == QDragEnterEvent::DragEnter)
		{
			event->ignore();
			return false;
		}
		else if (event->type() == QDropEvent::Drop)
		{
			event->ignore();
			return false;
		}
		return false;
	}
};

class SpaceNavigatorDevice : public Quarter::InputDevice {
public:
	SpaceNavigatorDevice(void) {}
	virtual ~SpaceNavigatorDevice() {}
	virtual const SoEvent* translateEvent(QEvent* event) {

		//if (event->type() == Spaceball::MotionEvent::MotionEventType) {
		//	Spaceball::MotionEvent* motionEvent = static_cast<Spaceball::MotionEvent*>(event);
		//	if (!motionEvent) {
		//		//Base::Console().Log("invalid spaceball motion event\n");
		//		return NULL;
		//	}
		//	motionEvent->setHandled(true);
		//	float xTrans, yTrans, zTrans;
		//	xTrans = static_cast<float>(motionEvent->translationX());
		//	yTrans = static_cast<float>(motionEvent->translationY());
		//	zTrans = static_cast<float>(motionEvent->translationZ());
		//	SbVec3f translationVector(xTrans, yTrans, zTrans);
		//	static float rotationConstant(.0001f);
		//	SbRotation xRot, yRot, zRot;
		//	xRot.setValue(SbVec3f(1.0, 0.0, 0.0), static_cast<float>(motionEvent->rotationX()) * rotationConstant);
		//	yRot.setValue(SbVec3f(0.0, 1.0, 0.0), static_cast<float>(motionEvent->rotationY()) * rotationConstant);
		//	zRot.setValue(SbVec3f(0.0, 0.0, 1.0), static_cast<float>(motionEvent->rotationZ()) * rotationConstant);
		//	SoMotion3Event* motion3Event = new SoMotion3Event;
		//	motion3Event->setTranslation(translationVector);
		//	motion3Event->setRotation(xRot * yRot * zRot);
		//	motion3Event->setPosition(this->mousepos);
		//	return motion3Event;
		//}

		return NULL;
	}
};

View3DInventor* View3DInventor::inventorInstance = nullptr;

View3DInventor::View3DInventor(QWidget* parent, const QtGLWidget* sharewidget)
	: inherited(parent, sharewidget), SelectionObserver(false, 0),
	editViewProvider(0), navigation(0),
	renderType(Native), framebuffer(0), axisCross(0), axisGroup(0), editing(false), redirected(false),
	allowredir(false), overrideMode("As Is")
{
	inventorInstance = this;
	init();
}

View3DInventor::View3DInventor(const QtGLFormat& format, QWidget* parent, const QtGLWidget* sharewidget)
	: inherited(format, parent, sharewidget), SelectionObserver(false, 0),
	editViewProvider(0), navigation(0),
	renderType(Native), framebuffer(0), axisCross(0), axisGroup(0), editing(false), redirected(false),
	allowredir(false), overrideMode("As Is")
{
	inventorInstance = this;
	init();
}

void View3DInventor::init()
{
	static bool _cacheModeInited;
	if (!_cacheModeInited) {
		_cacheModeInited = true;
		pcViewProviderRoot = 0;
		setRenderCache(-1);
	}

	this->setAcceptDrops(true);
	shading = true;
	vboEnabled = false;

	attachSelection();
	this->setClearWindow(false);
	initialize();

	SoOrthographicCamera* cam = new SoOrthographicCamera;
	cam->position = SbVec3f(0, 0, 1);
	cam->height = 1;
	cam->nearDistance = 0.5;
	cam->farDistance = 1.5;

	// setup light sources
	SoDirectionalLight* hl = this->getHeadlight();
	backlight = new SoDirectionalLight();
	backlight->ref();
	backlight->setName("backlight");
	backlight->direction.setValue(-hl->direction.getValue());
	backlight->on.setValue(true);

	// Set up background scenegraph with image in it.
	backgroundroot = new SoSeparator;
	backgroundroot->ref();
	this->backgroundroot->addChild(cam);

	// Background stuff
	pcBackGround = new SoFCBackgroundGradient;
	pcBackGround->ref();

	this->foregroundroot = new SoSeparator;
	this->foregroundroot->ref();

	SoLightModel* lm = new SoLightModel;
	lm->model = SoLightModel::BASE_COLOR;

	SoBaseColor* bc = new SoBaseColor;
	bc->rgb = SbColor(1, 1, 0);

	//cam = new SoOrthographicCamera;
	//cam->position = SbVec3f(0, 0, 5);
	//cam->height = 10;
	//cam->nearDistance = 0;
	//cam->farDistance = 10;

	this->foregroundroot->addChild(cam);
	this->foregroundroot->addChild(lm);
	this->foregroundroot->addChild(bc);

	selectionRoot = new SoFCUnifiedSelection();
	selectionRoot->applySettings();

	//pcViewProviderRoot = new SoSeparator();
	pcViewProviderRoot = selectionRoot;
	pcViewProviderRoot->ref();
	setSceneGraph(pcViewProviderRoot);

	// Event callback node
	pEventCallback = new SoEventCallback();
	pEventCallback->setUserData(this);
	pEventCallback->ref();
	pcViewProviderRoot->addChild(pEventCallback);
	pEventCallback->addEventCallback(SoEvent::getClassTypeId(), handleEventCB, this);

	dimensionRoot = new SoSeparator;
	pcViewProviderRoot->addChild(dimensionRoot);
	dimensionRoot->addChild(new SoSwitch()); //first one will be for the 3d dimensions.
	dimensionRoot->addChild(new SoSwitch()); //second one for the delta dimensions.

	originAxisRoot = new SoSeparator;
	originAxisRoot->ref();
	pcViewProviderRoot->addChild(originAxisRoot);

	pcGroupOnTop = new SoSeparator;
	pcGroupOnTop->ref();
	pcViewProviderRoot->addChild(pcGroupOnTop);

	auto pcGroupOnTopPickStyle = new SoPickStyle;
	pcGroupOnTopPickStyle->style = SoPickStyle::UNPICKABLE;
	pcGroupOnTopPickStyle->setOverride(true);
	pcGroupOnTop->addChild(pcGroupOnTopPickStyle);

	coin_setenv("COIN_SEPARATE_DIFFUSE_TRANSPARENCY_OVERRIDE", "1", TRUE);
	auto pcOnTopMaterial = new SoMaterial;
	pcOnTopMaterial->transparency = 0.5;
	pcOnTopMaterial->diffuseColor.setIgnored(true);
	pcOnTopMaterial->setOverride(true);
	pcGroupOnTop->addChild(pcOnTopMaterial);

	auto selRoot = new SoSelectionRoot;
	selRoot->selectionStyle = SoSelectionRoot::PassThrough;
	pcGroupOnTopSel = selRoot;
	pcGroupOnTopSel->setName("GroupOnTopSel");
	pcGroupOnTopSel->ref();
	pcGroupOnTop->addChild(pcGroupOnTopSel);
	selRoot = new SoSelectionRoot;
	selRoot->selectionStyle = SoSelectionRoot::PassThrough;
	pcGroupOnTopPreSel = selRoot;
	pcGroupOnTopPreSel->setName("GroupOnTopPreSel");
	pcGroupOnTopPreSel->ref();
	pcGroupOnTop->addChild(pcGroupOnTopPreSel);

	pcClipPlane = 0;

	pcEditingRoot = new SoSeparator;
	pcEditingRoot->ref();
	pcEditingRoot->setName("EditingRoot");
	pcEditingTransform = new SoTransform;
	pcEditingTransform->ref();
	pcEditingTransform->setName("EditingTransform");
	restoreEditingRoot = false;
	pcEditingRoot->addChild(pcEditingTransform);
	pcViewProviderRoot->addChild(pcEditingRoot);

	vpGroup = new SoGroup;
	vpGroup->ref();
	pcViewProviderRoot->addChild(vpGroup);

	getSoRenderManager()->getGLRenderAction()->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND);

	// Settings
	setSeekTime(0.4f);

	if (isSeekValuePercentage() == false)
		setSeekValueAsPercentage(true);

	setSeekDistance(100);
	setViewing(false);

	setBackgroundColor(QColor(236, 236, 236));
	setGradientBackground(true);

	// set some callback functions for user interaction
	addStartCallback(interactionStartCB);
	addFinishCallback(interactionFinishCB);

	//filter a few qt events
	viewerEventFilter = new ViewerEventFilter;
	installEventFilter(viewerEventFilter);
	//getEventFilter()->registerInputDevice(new SpaceNavigatorDevice);
	getEventFilter()->registerInputDevice(new GesturesDevice(this));

	this->winGestureTuneState = View3DInventor::ewgtsDisabled;
	try {
		this->grabGesture(Qt::PanGesture);
		this->grabGesture(Qt::PinchGesture);
	}
	catch (...) {}

	//create the cursors
	createStandardCursors(devicePixelRatio());

	connect(this, &View3DInventor::devicePixelRatioChanged,
		this, &View3DInventor::createStandardCursors);
}

/// adds an ViewProvider to the view, e.g. from a feature
void View3DInventor::addViewProvider(ViewProvider* pcProvider)
{
	SoSeparator* root = pcProvider->getRoot();

	if (root)
	{
		//pcViewProviderRoot->addChild(root);
		vpGroup->addChild(root);
		_CoinMap[root] = pcProvider;
	}

	pcProvider->setOverrideMode(this->getOverrideMode());
	_ViewProviderSet.insert(pcProvider);
}

void View3DInventor::onSelectionChanged(const SelectionChanges &_Reason)
{
	//if (!getDocument())
	//	return;

	SelectionChanges Reason(_Reason);

	//if (!Reason.pSubName && !(*Reason.pSubName))
	//	return;

	switch (Reason.Type) {
	case SelectionChanges::ShowSelection:
	case SelectionChanges::HideSelection:
		if (Reason.Type == SelectionChanges::ShowSelection)
			Reason.Type = SelectionChanges::AddSelection;
		else
			Reason.Type = SelectionChanges::RmvSelection;
		// fall through
	case SelectionChanges::SetPreselect:
		if (Reason.SubType != 2) // 2 means it is triggered from tree view
			break;
		// fall through
	case SelectionChanges::RmvPreselect:
	case SelectionChanges::RmvPreselectSignal:
	case SelectionChanges::SetSelection:
	case SelectionChanges::AddSelection:
	case SelectionChanges::RmvSelection:
	case SelectionChanges::ClrSelection:
		checkGroupOnTop(Reason);
		break;
	case SelectionChanges::SetPreselectSignal:
		break;
	default:
		return;
	}

	if (Reason.Type == SelectionChanges::RmvPreselect ||
		Reason.Type == SelectionChanges::RmvPreselectSignal)
	{
		//Hint: do not create a tmp. instance of SelectionChanges
		SelectionChanges selChanges(SelectionChanges::RmvPreselect);
		SoFCHighlightAction cAct(selChanges);
		cAct.apply(pcViewProviderRoot);
	}
	else {
		SoFCSelectionAction cAct(Reason);
		cAct.apply(pcViewProviderRoot);
	}
}


void View3DInventor::setRenderCache(int mode)
{
	static int canAutoCache = -1;

	if (mode < 0) {
		// Work around coin bug of unmatched call of
		// SoGLLazyElement::begin/endCaching() when on top rendering
		// transparent object with SORTED_OBJECT_SORTED_TRIANGLE_BLEND
		// transparency type.
		//
		// For more details see:
		// https://forum.freecadweb.org/viewtopic.php?f=18&t=43305&start=10#p412537
		coin_setenv("COIN_AUTO_CACHING", "0", TRUE);

		//int setting = ViewParams::instance()->getRenderCache();
		if (mode == -2) {
			if (pcViewProviderRoot /*&& setting != 1*/)
				pcViewProviderRoot->renderCaching = SoSeparator::ON;
			mode = 2;
		}
		else {
			if (pcViewProviderRoot)
				pcViewProviderRoot->renderCaching = SoSeparator::AUTO;
			//mode = setting;
		}
	}

	if (canAutoCache < 0) {
		const char *env = coin_getenv("COIN_AUTO_CACHING");
		canAutoCache = env ? atoi(env) : 1;
	}

	// If coin auto cache is disabled, do not use 'Auto' render cache mode, but
	// fallback to 'Distributed' mode.
	if (!canAutoCache && mode != 2)
		mode = 1;

	auto caching = mode == 0 ? SoSeparator::AUTO :
		(mode == 1 ? SoSeparator::ON :
			SoSeparator::OFF);

	SoFCSeparator::setCacheMode(caching);
}

void View3DInventor::setCameraType(SoType t)
{
	SIM::Coin3D::Quarter::SoQTQuarterAdaptor::setCameraType(t);

	if (t.isDerivedFrom(SoPerspectiveCamera::getClassTypeId())) {
		// When doing a viewAll() for an orthographic camera and switching
		// to perspective the scene looks completely strange because of the
		// heightAngle. Setting it to 45 deg also causes an issue with a too
		// close camera but we don't have this other ugly effect.
		SoCamera* cam = this->getSoRenderManager()->getCamera();

		if (cam == 0) return;

		static_cast<SoPerspectiveCamera*>(cam)->heightAngle = (float)(M_PI / 4.0);
	}
}

void View3DInventor::setBacklight(SbBool on)
{
	this->backlight->on = on;
}

SoDirectionalLight* View3DInventor::getBacklight(void) const
{
	return this->backlight;
}

void View3DInventor::turnAllDimensionsOn()
{
	//dimensionRoot->whichChild = SO_SWITCH_ALL;
}

void View3DInventor::turnAllDimensionsOff()
{
	//dimensionRoot->whichChild = SO_SWITCH_NONE;
}

void View3DInventor::eraseAllDimensions()
{
	coinRemoveAllChildren(static_cast<SoSwitch*>(dimensionRoot->getChild(0)));
	coinRemoveAllChildren(static_cast<SoSwitch*>(dimensionRoot->getChild(1)));
}

void View3DInventor::turn3dDimensionsOn()
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(0))->whichChild = SO_SWITCH_ALL;
}

void View3DInventor::turn3dDimensionsOff()
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(0))->whichChild = SO_SWITCH_NONE;
}

void View3DInventor::addDimension3d(SoNode* node)
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(0))->addChild(node);
}

void View3DInventor::addDimensionDelta(SoNode* node)
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(1))->addChild(node);
}

void View3DInventor::turnDeltaDimensionsOn()
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(1))->whichChild = SO_SWITCH_ALL;
}

void View3DInventor::turnDeltaDimensionsOff()
{
	static_cast<SoSwitch*>(dimensionRoot->getChild(1))->whichChild = SO_SWITCH_NONE;
}

View3DInventor::~View3DInventor()
{
	coinRemoveAllChildren(originAxisRoot);

	// to prevent following OpenGL error message: "Texture is not valid in the current context. Texture has not been destroyed"
	//aboutToDestroyGLContext();

	// It can happen that a document has several MDI views and when the about to be
	// closed 3D view is in edit mode the corresponding view provider must be restored
	// because otherwise it might be left in a broken state
	// See https://forum.freecadweb.org/viewtopic.php?f=3&t=39720
	//if (restoreEditingRoot) {
	//	resetEditingRoot(false);
	//}

	// cleanup
	this->backgroundroot->unref();
	this->backgroundroot = 0;
	this->foregroundroot->unref();
	this->foregroundroot = 0;
	this->pcBackGround->unref();
	this->pcBackGround = 0;

	setSceneGraph(0);
	this->pEventCallback->unref();
	this->pEventCallback = 0;
	// Note: It can happen that there is still someone who references
	// the root node but isn't destroyed when closing this viewer so
	// that it prevents all children from being deleted. To reduce this
	// likelihood we explicitly remove all child nodes now.
	coinRemoveAllChildren(this->pcViewProviderRoot);
	this->pcViewProviderRoot->unref();
	this->pcViewProviderRoot = 0;
	this->backlight->unref();
	this->backlight = 0;

	this->pcGroupOnTop->unref();
	this->pcGroupOnTopPreSel->unref();
	this->pcGroupOnTopSel->unref();

	this->pcEditingRoot->unref();
	this->pcEditingTransform->unref();

	if (this->pcClipPlane)
		this->pcClipPlane->unref();

	delete this->navigation;

	// Note: When closing the application the main window doesn't exist any more.
	//if (getMainWindow())
	//	getMainWindow()->setPaneText(2, QLatin1String(""));

	detachSelection();

	removeEventFilter(viewerEventFilter);
	delete viewerEventFilter;

	// In the init() function we have overridden the default SoGLRenderAction with our
	// own instance of SoBoxSelectionRenderAction and SoRenderManager destroyed the default.
	// But it does this only once so that now we have to explicitly destroy our instance in
	// order to free the memory.
	SoGLRenderAction* glAction = this->getSoRenderManager()->getGLRenderAction();
	this->getSoRenderManager()->setGLRenderAction(nullptr);
	glAction = nullptr;
	delete glAction;
	//for (auto vpr : _CoinMap) {
	//	delete vpr.second;
	//	vpr.second = nullptr;
	//}
}

SoSeparator* View3DInventor::getRoot() const
{
	return pcViewProviderRoot;
}

void View3DInventor::setGradientBackground(bool on)
{
	if (on && backgroundroot->findChild(pcBackGround) == -1)
		backgroundroot->addChild(pcBackGround);
	else if (!on && backgroundroot->findChild(pcBackGround) != -1)
		backgroundroot->removeChild(pcBackGround);
}

void View3DInventor::interactionStartCB(void*, SoQTQuarterAdaptor* viewer)
{
	SoGLRenderAction* glra = viewer->getSoRenderManager()->getGLRenderAction();
	SoFCInteractiveElement::set(glra->getState(), viewer->getSceneGraph(), true);
}

void View3DInventor::interactionFinishCB(void*, SoQTQuarterAdaptor* viewer)
{
	SoGLRenderAction* glra = viewer->getSoRenderManager()->getGLRenderAction();
	SoFCInteractiveElement::set(glra->getState(), viewer->getSceneGraph(), false);
	viewer->redraw();
}

void View3DInventor::handleEventCB(void* ud, SoEventCallback* n)
{
	View3DInventor* that = reinterpret_cast<View3DInventor*>(ud);
	SoGLRenderAction* glra = that->getSoRenderManager()->getGLRenderAction();
	SoAction* action = n->getAction();
	SoGLRenderActionElement::set(action->getState(), glra);
	SoGLWidgetElement::set(action->getState(), qobject_cast<QtGLWidget*>(that->getGLWidget()));
}

void View3DInventor::createStandardCursors(double dpr)
{
	QBitmap cursor = QBitmap::fromData(QSize(ROTATE_WIDTH, ROTATE_HEIGHT), rotate_bitmap);
	QBitmap mask = QBitmap::fromData(QSize(ROTATE_WIDTH, ROTATE_HEIGHT), rotate_mask_bitmap);
#if defined(Q_OS_WIN32)
	cursor.setDevicePixelRatio(dpr);
	mask.setDevicePixelRatio(dpr);
#else
	Q_UNUSED(dpr)
#endif
		spinCursor = QCursor(cursor, mask, ROTATE_HOT_X, ROTATE_HOT_Y);

	cursor = QBitmap::fromData(QSize(ZOOM_WIDTH, ZOOM_HEIGHT), zoom_bitmap);
	mask = QBitmap::fromData(QSize(ZOOM_WIDTH, ZOOM_HEIGHT), zoom_mask_bitmap);
#if defined(Q_OS_WIN32)
	cursor.setDevicePixelRatio(dpr);
	mask.setDevicePixelRatio(dpr);
#endif
	zoomCursor = QCursor(cursor, mask, ZOOM_HOT_X, ZOOM_HOT_Y);

	cursor = QBitmap::fromData(QSize(PAN_WIDTH, PAN_HEIGHT), pan_bitmap);
	mask = QBitmap::fromData(QSize(PAN_WIDTH, PAN_HEIGHT), pan_mask_bitmap);
#if defined(Q_OS_WIN32)
	cursor.setDevicePixelRatio(dpr);
	mask.setDevicePixelRatio(dpr);
#endif
	panCursor = QCursor(cursor, mask, PAN_HOT_X, PAN_HOT_Y);
}

ViewProvider* View3DInventor::getViewProviderByPathFromHead(SoPath * path) const
{
	for (int i = 0; i < path->getLength(); i++) {
		SoNode *node = path->getNode(i);
		if (node->isOfType(SoSeparator::getClassTypeId())) {
			auto it = _CoinMap.find(static_cast<SoSeparator*>(node));
			if (it != _CoinMap.end())
				return it->second;
		}
	}

	return 0;
}

GLenum View3DInventor::getInternalTextureFormat() const
{
#undef HAVE_QT5_OPENGL
#if defined(HAVE_QT5_OPENGL)
	ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath
	("User parameter:BaseApp/Preferences/View");
	std::string format = hGrp->GetASCII("InternalTextureFormat", "Default");

	if (format == "GL_RGB") {
		return GL_RGB;
	}
	else if (format == "GL_RGBA") {
		return GL_RGBA;
	}
	else if (format == "GL_RGB8") {
		return GL_RGB8;
	}
	else if (format == "GL_RGBA8") {
		return GL_RGBA8;
	}
	else if (format == "GL_RGB10") {
		return GL_RGB10;
	}
	else if (format == "GL_RGB10_A2") {
		return GL_RGB10_A2;
	}
	else if (format == "GL_RGB16") {
		return GL_RGB16;
	}
	else if (format == "GL_RGBA16") {
		return GL_RGBA16;
	}
	else if (format == "GL_RGB32F") {
		return GL_RGB32F_ARB;
	}
	else if (format == "GL_RGBA32F") {
		return GL_RGBA32F_ARB;
	}
	else {
		QOpenGLFramebufferObjectFormat fboFormat;
		return fboFormat.internalTextureFormat();
	}
#else
	//return GL_RGBA;
	return GL_RGB;
#endif
}

void View3DInventor::renderScene(void)
{
	// Must set up the OpenGL viewport manually, as upon resize
	// operations, Coin won't set it up until the SoGLRenderAction is
	// applied again. And since we need to do glClear() before applying
	// the action..
	const SbViewportRegion vp = this->getSoRenderManager()->getViewportRegion();
	SbVec2s origin = vp.getViewportOriginPixels();
	SbVec2s size = vp.getViewportSizePixels();
	glViewport(origin[0], origin[1], size[0], size[1]);

	const QColor col = this->backgroundColor();
	glClearColor(col.redF(), col.greenF(), col.blueF(), 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

#if defined(ENABLE_GL_DEPTH_RANGE)
	// using 90% of the z-buffer for the background and the main node
	glDepthRange(0.1, 1.0);
#endif

	// Render our scenegraph with the image.
	SoGLRenderAction* glra = this->getSoRenderManager()->getGLRenderAction();
	SoState* state = glra->getState();
	SoGLWidgetElement::set(state, qobject_cast<QtGLWidget*>(this->getGLWidget()));
	SoGLRenderActionElement::set(state, glra);
	SoGLVBOActivatedElement::set(state, this->vboEnabled);
	glra->apply(this->backgroundroot);

	navigation->updateAnimation();

	if (!this->shading) {
		state->push();
		SoLightModelElement::set(state, selectionRoot, SoLightModelElement::BASE_COLOR);
		SoOverrideElement::setLightModelOverride(state, selectionRoot, true);
	}

	try {
		// Render normal scenegraph.
		inherited::actualRedraw();
	}
	catch (QException) {
		// FIXME: If this exception appears then the background and camera position get broken somehow. (Werner 2006-02-01)
		for (std::set<ViewProvider*>::iterator it = _ViewProviderSet.begin(); it != _ViewProviderSet.end(); ++it)
			(*it)->hide();

		inherited::actualRedraw();
		QMessageBox::warning(parentWidget(), QObject::tr("Out of memory"),
			QObject::tr("Not enough memory available to display the data."));
	}

	if (!this->shading) {
		state->pop();
	}

#if defined (ENABLE_GL_DEPTH_RANGE)
	// using 10% of the z-buffer for the foreground node
	glDepthRange(0.0, 0.1);
#endif

	// Render overlay front scenegraph.
	glra->apply(this->foregroundroot);

	if (this->axiscrossEnabled) {
		this->drawAxisCross();
	}

#if defined (ENABLE_GL_DEPTH_RANGE)
	// using the main portion of z-buffer again (for frontbuffer highlighting)
	glDepthRange(0.1, 1.0);
#endif

	// Immediately reschedule to get continuous spin animation.
	if (this->isAnimating()) {
		this->getSoRenderManager()->scheduleRedraw();
	}

#if 0 // this breaks highlighting of edges
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
#endif

	printDimension();
	navigation->redraw();

	for (std::list<GLGraphicsItem*>::iterator it = this->graphicsItems.begin(); it != this->graphicsItems.end(); ++it)
		(*it)->paintGL();

	//fps rendering
	//if (fpsEnabled) {
	//	std::stringstream stream;
	//	stream.precision(1);
	//	stream.setf(std::ios::fixed | std::ios::showpoint);
	//	stream << framesPerSecond[0] << " ms / " << framesPerSecond[1] << " fps";
	//	draw2DString(stream.str().c_str(), SbVec2s(10, 10), SbVec2f(0.1f, 0.1f));
	//}

	//if (naviCubeEnabled)
	//	naviCube->drawNaviCube();

#if 0 // this breaks highlighting of edges
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
#endif
}

void View3DInventor::renderFramebuffer()
{
	const SbViewportRegion vp = this->getSoRenderManager()->getViewportRegion();
	SbVec2s size = vp.getViewportSizePixels();

	glDisable(GL_LIGHTING);
	glViewport(0, 0, size[0], size[1]);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);

	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, this->framebuffer->texture());
	glColor3f(1.0, 1.0, 1.0);

	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(-1.0, -1.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(1.0f, -1.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(1.0f, 1.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(-1.0f, 1.0f);
	glEnd();

	printDimension();
	navigation->redraw();

	for (std::list<GLGraphicsItem*>::iterator it = this->graphicsItems.begin(); it != this->graphicsItems.end(); ++it)
		(*it)->paintGL();

	//if (naviCubeEnabled)
		//naviCube->drawNaviCube();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}

void View3DInventor::animatedViewAll(int steps, int ms)
{
	SoCamera* cam = this->getSoRenderManager()->getCamera();
	if (!cam)
		return;

	SbVec3f campos = cam->position.getValue();
	SbRotation camrot = cam->orientation.getValue();
	SbViewportRegion vp = this->getSoRenderManager()->getViewportRegion();
	SoGetBoundingBoxAction action(vp);
	action.apply(this->getSoRenderManager()->getSceneGraph());
	SbBox3f box = action.getBoundingBox();

#if (COIN_MAJOR_VERSION >= 3)
	float aspectRatio = vp.getViewportAspectRatio();
#endif

	if (box.isEmpty())
		return;

	SbSphere sphere;
	sphere.circumscribe(box);
	if (sphere.getRadius() == 0)
		return;

	SbVec3f direction, pos;
	camrot.multVec(SbVec3f(0, 0, -1), direction);

	bool isOrthographic = false;
	float height = 0;
	float diff = 0;

	if (cam->isOfType(SoOrthographicCamera::getClassTypeId())) {
		isOrthographic = true;
		height = static_cast<SoOrthographicCamera*>(cam)->height.getValue();
#if (COIN_MAJOR_VERSION >= 3)
		if (aspectRatio < 1.0f)
			diff = sphere.getRadius() * 2 - height * aspectRatio;
		else
#endif
			diff = sphere.getRadius() * 2 - height;
		pos = (box.getCenter() - direction * sphere.getRadius());
	}
	else if (cam->isOfType(SoPerspectiveCamera::getClassTypeId())) {
		float movelength = sphere.getRadius() / float(tan(static_cast<SoPerspectiveCamera*>
			(cam)->heightAngle.getValue() / 2.0));
		pos = box.getCenter() - direction * movelength;
	}

	QEventLoop loop;
	QTimer timer;
	timer.setSingleShot(true);
	QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));

	for (int i = 0; i < steps; i++) {
		float s = float(i) / float(steps);

		if (isOrthographic) {
			float camHeight = height + diff * s;
			static_cast<SoOrthographicCamera*>(cam)->height.setValue(camHeight);
		}

		SbVec3f curpos = campos * (1.0f - s) + pos * s;
		cam->position.setValue(curpos);
		timer.start(clamp<int>(ms, 0, 5000));
		loop.exec(QEventLoop::ExcludeUserInputEvents);
	}
}

void View3DInventor::actualRedraw()
{
	switch (renderType) {
	case Native:
		renderScene();
		break;
	case Framebuffer:
		renderFramebuffer();
		break;
	case Image:
		//renderGLImage();
		break;
	}
}

void View3DInventor::setSeekMode(SbBool on)
{
	// Overrides this method to make sure any animations are stopped
	// before we go into seek mode.

	// Note: this method is almost identical to the setSeekMode() in the
	// SoQtFlyViewer and SoQtPlaneViewer, so migrate any changes.

	//if (this->isAnimating()) {
	//	this->stopAnimating();
	//}

	inherited::setSeekMode(on);
	navigation->setViewingMode(on ? NavigationStyle::SEEK_WAIT_MODE :
		(this->isViewing() ?
			NavigationStyle::IDLE : NavigationStyle::INTERACT));
}

void View3DInventor::afterRealizeHook(void)
{
	inherited::afterRealizeHook();
	this->setCursorRepresentation(navigation->getViewingMode());
}

void View3DInventor::setCursorRepresentation(int modearg)
{
	// There is a synchronization problem between Qt and SoQt which
	// happens when popping up a context-menu. In this case the
	// Qt::WA_UnderMouse attribute is reset and never set again
	// even if the mouse is still in the canvas. Thus, the cursor
	// won't be changed as long as the user doesn't leave and enter
	// the canvas. To fix this we explicitly set Qt::WA_UnderMouse
	// if the mouse is inside the canvas.
	QWidget* glWindow = this->getGLWidget();

	// When a widget is added to the QGraphicsScene and the user
	// hovered over it the 'WA_SetCursor' attribute is set to the
	// GL widget but never reset and thus would cause that the
	// cursor on this widget won't be set.
	if (glWindow)
		glWindow->setAttribute(Qt::WA_SetCursor, false);

	if (glWindow && glWindow->rect().contains(QCursor::pos()))
		glWindow->setAttribute(Qt::WA_UnderMouse);

	switch (modearg) {
	case NavigationStyle::IDLE:
	case NavigationStyle::INTERACT:
		if (isEditing())
			this->getWidget()->setCursor(this->editCursor);
		else
			this->getWidget()->setCursor(QCursor(Qt::ArrowCursor));
		break;

	case NavigationStyle::DRAGGING:
	case NavigationStyle::SPINNING:
		this->getWidget()->setCursor(spinCursor);
		break;

	case NavigationStyle::ZOOMING:
		this->getWidget()->setCursor(zoomCursor);
		break;

	case NavigationStyle::SEEK_MODE:
	case NavigationStyle::SEEK_WAIT_MODE:
	case NavigationStyle::BOXZOOM:
		this->getWidget()->setCursor(Qt::CrossCursor);
		break;

	case NavigationStyle::PANNING:
		this->getWidget()->setCursor(panCursor);
		break;

	case NavigationStyle::SELECTION:
		this->getWidget()->setCursor(Qt::PointingHandCursor);
		break;

	default:
		assert(0);
		break;
	}
}

void View3DInventor::addGraphicsItem(GLGraphicsItem* item)
{
	this->graphicsItems.push_back(item);
}

void View3DInventor::setRenderType(const RenderType type)
{
	renderType = type;

	glImage = QImage();
	if (type != Framebuffer) {
		delete framebuffer;
		framebuffer = 0;
	}

	switch (type) {
	case Native:
		break;
	case Framebuffer:
		if (!framebuffer) {
			const SbViewportRegion vp = this->getSoRenderManager()->getViewportRegion();
			SbVec2s size = vp.getViewportSizePixels();
			int width = size[0];
			int height = size[1];

			QtGLWidget* gl = static_cast<QtGLWidget*>(this->getWidget());
			gl->makeCurrent();
#if !defined(HAVE_QT5_OPENGL)
			framebuffer = new QOpenGLFramebufferObject(width, height, QOpenGLFramebufferObject::Depth);
			renderToFramebuffer(framebuffer);
#else
			QOpenGLFramebufferObjectFormat fboFormat;
			fboFormat.setSamples(/*getNumSamples()*/4);
			fboFormat.setAttachment(QtGLFramebufferObject::Depth);
			QtGLFramebufferObject* fbo = new QtGLFramebufferObject(width, height, fboFormat);
			if (fbo->format().samples() > 0) {
				renderToFramebuffer(fbo);
				framebuffer = new QtGLFramebufferObject(fbo->size());
				// this is needed to be able to render the texture later
				QOpenGLFramebufferObject::blitFramebuffer(framebuffer, fbo);
				delete fbo;
			}
			else {
				renderToFramebuffer(fbo);
				framebuffer = fbo;
			}
#endif
		}
		break;
	case Image:
	{
		//glImage = grabFramebuffer();
	}
	break;
	}
}

void View3DInventor::renderToFramebuffer(QOpenGLFramebufferObject* fbo)
{
	static_cast<QtGLWidget*>(this->getWidget())->makeCurrent();
	fbo->bind();
	int width = fbo->size().width();
	int height = fbo->size().height();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);

	const QColor col = this->backgroundColor();
	glViewport(0, 0, width, height);
	glClearColor(col.redF(), col.greenF(), col.blueF(), col.alphaF());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// If on then transparent areas may shine through opaque areas
	//glDepthRange(0.1,1.0);

	SoBoxSelectionRenderAction gl(SbViewportRegion(width, height));
	// When creating a new GL render action we have to copy over the cache context id
	// For further details see init().
	uint32_t id = this->getSoRenderManager()->getGLRenderAction()->getCacheContext();
	gl.setCacheContext(id);
	gl.setTransparencyType(SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND);

	if (!this->shading) {
		SoLightModelElement::set(gl.getState(), selectionRoot, SoLightModelElement::BASE_COLOR);
		SoOverrideElement::setLightModelOverride(gl.getState(), selectionRoot, true);
	}

	gl.apply(this->backgroundroot);
	// The render action of the render manager has set the depth function to GL_LESS
	// while creating a new render action has it set to GL_LEQUAL. So, in order to get
	// the exact same result set it explicitly to GL_LESS.
	glDepthFunc(GL_LESS);
	gl.apply(this->getSoRenderManager()->getSceneGraph());
	gl.apply(this->foregroundroot);

	if (this->axiscrossEnabled) {
		this->drawAxisCross();
	}

	fbo->release();
}

void View3DInventor::removeGraphicsItem(GLGraphicsItem* item)
{
	this->graphicsItems.remove(item);
}

void View3DInventor::boxZoom(const SbBox2s& box)
{
	navigation->boxZoom(box);
}

void View3DInventor::setViewing(SbBool enable)
{
	if (this->isViewing() == enable) {
		return;
	}

	navigation->setViewingMode(enable ?
		NavigationStyle::IDLE : NavigationStyle::INTERACT);
	inherited::setViewing(enable);
}

SbBool View3DInventor::processSoEventBase(const SoEvent* const ev)
{
	return inherited::processSoEvent(ev);
}

void View3DInventor::printDimension()
{
	SoCamera* cam = getSoRenderManager()->getCamera();
	if (!cam) return; // no camera there

	SoType t = getSoRenderManager()->getCamera()->getTypeId();
	if (t.isDerivedFrom(SoOrthographicCamera::getClassTypeId())) {
		const SbViewportRegion& vp = getSoRenderManager()->getViewportRegion();
		const SbVec2s& size = vp.getWindowSize();
		short dimX, dimY;
		size.getValue(dimX, dimY);

		float fHeight = static_cast<SoOrthographicCamera*>(getSoRenderManager()->getCamera())->height.getValue();
		float fWidth = fHeight;

		if (dimX > dimY)
			fWidth *= ((float)dimX) / ((float)dimY);
		else if (dimX < dimY)
			fHeight *= ((float)dimY) / ((float)dimX);

		// Translate screen units into user's unit schema
		//Quantity qWidth(Base::Quantity::MilliMetre);
		//Base::Quantity qHeight(Base::Quantity::MilliMetre);
		//qWidth.setValue(fWidth);
		//qHeight.setValue(fHeight);
		//QString wStr = Base::UnitsApi::schemaTranslate(qWidth);
		//QString hStr = Base::UnitsApi::schemaTranslate(qHeight);

		// Create final string and update window
		//QString dim = QString::fromLatin1("%1 x %2")
		//	.arg(wStr, hStr);
		//getMainWindow()->setPaneText(2, dim);
	}
	//else
	//	getMainWindow()->setPaneText(2, QLatin1String(""));
}

void View3DInventor::checkGroupOnTop(const SelectionChanges &Reason) {
	if (Reason.Type == SelectionChanges::SetSelection || Reason.Type == SelectionChanges::ClrSelection) {
		clearGroupOnTop();
		if (Reason.Type == SelectionChanges::ClrSelection)
			return;
	}
	if (Reason.Type == SelectionChanges::RmvPreselect ||
		Reason.Type == SelectionChanges::RmvPreselectSignal)
	{
		SoSelectionElementAction action(SoSelectionElementAction::None, true);
		action.apply(pcGroupOnTopPreSel);
		coinRemoveAllChildren(pcGroupOnTopPreSel);
		objectsOnTopPreSel.clear();
		return;
	}
	if (!Reason.vp)
		return;
	if (Reason.Type == SelectionChanges::RmvSelection) {
		auto &objs = objectsOnTop;
		auto pcGroup = pcGroupOnTopSel;
		auto it = objs.find(Reason.vp);
		if (it == objs.end())
			return;
		int index = pcGroup->findChild(it->second);
		if (index >= 0) {
			auto node = static_cast<SoFCPathAnnotation*>(it->second);
			SoSelectionElementAction action(node->getDetail() ?
				SoSelectionElementAction::Remove : SoSelectionElementAction::None, true);
			auto path = node->getPath();
			SoTempPath tmpPath(2 + (path ? path->getLength() : 0));
			tmpPath.ref();
			tmpPath.append(pcGroup);
			tmpPath.append(node);
			tmpPath.append(node->getPath());
			action.setElement(node->getDetail());
			action.apply(&tmpPath);
			tmpPath.unrefNoDelete();
			pcGroup->removeChild(index);
		}
		objs.erase(it);
		return;
	}

	auto &objs = Reason.Type == SelectionChanges::SetPreselect ? objectsOnTopPreSel : objectsOnTop;
	auto pcGroup = Reason.Type == SelectionChanges::SetPreselect ? pcGroupOnTopPreSel : pcGroupOnTopSel;

	if (objs.find(Reason.vp) != objs.end())
		return;
	auto vp = Reason.vp;
	if (!vp || !vp->isSelectable() /*|| !vp->isShow()*/)
		return;
	int onTop;
	// onTop==2 means on top only if whole object is selected,
	// onTop==3 means on top only if some sub-element is selected
	// onTop==1 means either
	if (Selection().needPickedList())
		onTop = 1;
	else
		onTop = 0;
	if (Reason.Type == SelectionChanges::SetPreselect) {
		SoHighlightElementAction action;
		action.setHighlighted(true);
		action.setColor(selectionRoot->colorHighlight.getValue());
		action.apply(pcGroupOnTopPreSel);
		if (!onTop)
			onTop = 2;
	}
	else {
		if (!onTop)
			return;
		SoSelectionElementAction action(SoSelectionElementAction::All);
		action.setColor(selectionRoot->colorSelection.getValue());
		action.apply(pcGroupOnTopSel);
	}
	if (onTop == 2 || onTop == 3) {
		auto subname = Reason.pSubName;
		if (subname && *subname) {
			size_t len = strlen(subname);
			if (subname[len - 1] == '.') {
				// ending with '.' means whole object selection
				if (onTop == 3)
					return;
			}
			else if (onTop == 2)
				return;
		}
		else if (onTop == 3)
			return;
	}

	SoTempPath path(10);
	path.ref();

	SoDetail *det = 0;
	if (vp->getDetailPath(Reason.pSubName, &path, true, det) && path.getLength()) {
		SoFCPathAnnotation* node = new SoFCPathAnnotation;
		node->setPath(&path);
		pcGroup->addChild(node);
		if (det) {
			SoSelectionElementAction action(SoSelectionElementAction::Append, true);
			action.setElement(det);
			SoTempPath tmpPath(path.getLength() + 2);
			tmpPath.ref();
			tmpPath.append(pcGroup);
			tmpPath.append(node);
			tmpPath.append(&path);
			action.apply(&tmpPath);
			tmpPath.unrefNoDelete();
			node->setDetail(det);
			det = 0;
		}
		objs[Reason.vp] = node;
	}
	delete det;
	path.unrefNoDelete();
}

void View3DInventor::clearGroupOnTop() {
	if (objectsOnTop.size() || objectsOnTopPreSel.size()) {
		objectsOnTop.clear();
		objectsOnTopPreSel.clear();
		SoSelectionElementAction action(SoSelectionElementAction::None, true);
		action.apply(pcGroupOnTopPreSel);
		action.apply(pcGroupOnTopSel);
		coinRemoveAllChildren(pcGroupOnTopSel);
		coinRemoveAllChildren(pcGroupOnTopPreSel);
	}
}

void View3DInventor::initialize()
{
	//navigation = new CADNavigationStyle();
	navigation = new GestureNavigationStyle();
	//delete ptr;
	//navi = nullptr;
	navigation->setViewer(this);

	this->axiscrossEnabled = true;
	this->axiscrossSize = 10;
}

void View3DInventor::selectAll()
{
	std::vector<ViewProvider*> vps;

	for (std::set<ViewProvider*>::iterator it = _ViewProviderSet.begin(); it != _ViewProviderSet.end(); ++it) {
		ViewProvider* vp = static_cast<ViewProvider*>(*it);
		if (vp) vps.push_back(vp);
	}

	if (!vps.empty())
		Selection().setSelection(vps.front(), vps);
}

void View3DInventor::setNavigationType(Base::Type t)
{
	if (t.isBad())
		return;

	this->winGestureTuneState = View3DInventor::ewgtsNeedTuning; //triggers enable/disable rotation gesture when preferences change

	if (this->navigation && this->navigation->getTypeId() == t)
		return; // nothing to do

	Base::BaseClass* base = static_cast<Base::BaseClass*>(t.createInstance());
	if (!base)
		return;

	if (!base->getTypeId().isDerivedFrom(NavigationStyle::getClassTypeId())) {
		delete base;
		return;
	}

	NavigationStyle* ns = static_cast<NavigationStyle*>(base);
	if (this->navigation) {
		ns->operator = (*this->navigation);
		delete this->navigation;
	}
	this->navigation = ns;
	this->navigation->setViewer(this);
}

NavigationStyle* View3DInventor::navigationStyle() const
{
	return this->navigation;
}

void View3DInventor::startSelection(View3DInventor::SelectionMode mode)
{
	navigation->startSelection(NavigationStyle::SelectionMode(mode));
}

void View3DInventor::stopSelection()
{
	navigation->stopSelection();
}

bool View3DInventor::isSelecting() const
{
	return navigation->isSelecting();
}

bool View3DInventor::processSoEvent(const SoEvent* ev)
{
	if (isRedirectedToSceneGraph()) {
		SbBool processed = inherited::processSoEvent(ev);

		if (!processed)
			processed = navigation->processEvent(ev);

		return processed;
	}

	if (ev->getTypeId().isDerivedFrom(SoKeyboardEvent::getClassTypeId())) {
		// filter out 'Q' and 'ESC' keys
		const SoKeyboardEvent* const ke = static_cast<const SoKeyboardEvent*>(ev);

		switch (ke->getKey()) {
		case SoKeyboardEvent::ESCAPE:
		case SoKeyboardEvent::Q: // ignore 'Q' keys (to prevent app from being closed)
			return inherited::processSoEvent(ev);
		default:
			break;
		}
	}
	return navigation->processEvent(ev);
}

void View3DInventor::setCameraOrientation(const SbRotation& rot, SbBool moveTocenter)
{
	navigation->setCameraOrientation(rot, moveTocenter);
}

void View3DInventor::setAnimationEnabled(const SbBool enable)
{
	navigation->setAnimationEnabled(enable);
}

SbBool View3DInventor::isAnimationEnabled(void) const
{
	return navigation->isAnimationEnabled();
}

SbBool View3DInventor::isAnimating(void) const
{
	return navigation->isAnimating();
}

void View3DInventor::startAnimating(const SbVec3f& axis, float velocity)
{
	navigation->startAnimating(axis, velocity);
}

void View3DInventor::stopAnimating(void)
{
	navigation->stopAnimating();
}

void View3DInventor::setPopupMenuEnabled(const SbBool on)
{
	navigation->setPopupMenuEnabled(on);
}

SbBool View3DInventor::isPopupMenuEnabled(void) const
{
	return navigation->isPopupMenuEnabled();
}

void View3DInventor::setCursorEnabled(SbBool /*enable*/)
{
	this->setCursorRepresentation(navigation->getViewingMode());
}

void View3DInventor::viewAll()
{
	SbViewportRegion vp = this->getSoRenderManager()->getViewportRegion();
	SoGetBoundingBoxAction action(vp);
	action.apply(this->getSoRenderManager()->getSceneGraph());
	SbBox3f box = action.getBoundingBox();

	if (box.isEmpty())
		return;

	SbSphere sphere;
	sphere.circumscribe(box);
	if (sphere.getRadius() == 0)
		return;

	// in the scene graph we may have objects which we want to exclude
	// when doing a fit all. Such objects must be part of the group
	// SoSkipBoundingGroup.
	SoSearchAction sa;
	sa.setType(SoSkipBoundingGroup::getClassTypeId());
	sa.setInterest(SoSearchAction::ALL);
	sa.apply(this->getSoRenderManager()->getSceneGraph());
	const SoPathList& pathlist = sa.getPaths();

	for (int i = 0; i < pathlist.getLength(); i++) {
		SoPath* path = pathlist[i];
		SoSkipBoundingGroup* group = static_cast<SoSkipBoundingGroup*>(path->getTail());
		group->mode = SoSkipBoundingGroup::EXCLUDE_BBOX;
	}

	// Set the height angle to 45 deg
	SoCamera* cam = this->getSoRenderManager()->getCamera();

	if (cam && cam->getTypeId().isDerivedFrom(SoPerspectiveCamera::getClassTypeId()))
		static_cast<SoPerspectiveCamera*>(cam)->heightAngle = (float)(M_PI / 4.0);

	float aspectratio = getSoRenderManager()->getViewportRegion().getViewportAspectRatio();
	action.apply(vpGroup);
	auto bbox = action.getBoundingBox();
	if (cam)
		cam->viewBoundingBox(bbox, aspectratio, 1.0);

	auto size = this->getSoRenderManager()->getViewportRegion().getViewportSizePixels();

	for (int i = 0; i < pathlist.getLength(); i++) {
		SoPath* path = pathlist[i];
		SoSkipBoundingGroup* group = static_cast<SoSkipBoundingGroup*>(path->getTail());
		group->mode = SoSkipBoundingGroup::INCLUDE_BBOX;
	}
}

void View3DInventor::viewAll(float factor)
{
	SoCamera* cam = this->getSoRenderManager()->getCamera();

	if (!cam) return;

	if (factor <= 0.0f) return;

	if (factor != 1.0f) {
		SoSearchAction sa;
		sa.setType(SoSkipBoundingGroup::getClassTypeId());
		sa.setInterest(SoSearchAction::ALL);
		sa.apply(this->getSoRenderManager()->getSceneGraph());
		const SoPathList& pathlist = sa.getPaths();

		for (int i = 0; i < pathlist.getLength(); i++) {
			SoPath* path = pathlist[i];
			SoSkipBoundingGroup* group = static_cast<SoSkipBoundingGroup*>(path->getTail());
			group->mode = SoSkipBoundingGroup::EXCLUDE_BBOX;
		}

		SoGetBoundingBoxAction action(this->getSoRenderManager()->getViewportRegion());
		action.apply(this->getSoRenderManager()->getSceneGraph());
		SbBox3f box = action.getBoundingBox();
		float minx, miny, minz, maxx, maxy, maxz;
		box.getBounds(minx, miny, minz, maxx, maxy, maxz);

		for (int i = 0; i < pathlist.getLength(); i++) {
			SoPath* path = pathlist[i];
			SoSkipBoundingGroup* group = static_cast<SoSkipBoundingGroup*>(path->getTail());
			group->mode = SoSkipBoundingGroup::INCLUDE_BBOX;
		}

		SoCube* cube = new SoCube();
		cube->width = factor * (maxx - minx);
		cube->height = factor * (maxy - miny);
		cube->depth = factor * (maxz - minz);

		// fake a scenegraph with the desired bounding size
		SoSeparator* graph = new SoSeparator();
		graph->ref();
		SoTranslation* tr = new SoTranslation();
		tr->translation.setValue(box.getCenter());

		graph->addChild(tr);
		graph->addChild(cube);
		cam->viewAll(graph, this->getSoRenderManager()->getViewportRegion());
		graph->unref();
	}
	else {
		viewAll();
	}
}

void View3DInventor::viewSelection()
{
	if (Selection().getSelection().size() == 0) return;
	std::vector<SbBox3f> boxes;
	for (auto &sel : Selection().getSelection()) {
		auto vp = sel.vp;
		if (!vp)
			continue;
		SoTempPath path(20);
		path.ref();
		SoDetail *det = 0;
		vp->getDetailPath(sel.SubName, &path, true, det);
		SoGetBoundingBoxAction action(getSoRenderManager()->getViewportRegion());
		action.apply(&path);
		auto bbox = action.getBoundingBox();
		boxes.emplace_back(bbox);
	}

	std::vector<gp_Pnt> points;
	for (auto b : boxes) {
		float X, Y, Z;
		b.getMin().getValue(X, Y, Z);
		points.emplace_back(gp_Pnt(X, Y, Z));
		b.getMax().getValue(X, Y, Z);
		points.emplace_back(gp_Pnt(X, Y, Z));
	}
	gp_Pnt minp, maxp;
	double dis = 0;
	for (int i = 0; i < points.size(); i++) {
		for (int j = i + 1; j < points.size(); j++) {
			double p2p = points[i].Distance(points[j]);
			if (p2p > dis) {
				dis = p2p;
				if (/*points[i].Distance(gp_Pnt(0, 0, 0)) < points[j].Distance(gp_Pnt(0, 0, 0))*/
					points[i].Z() < points[j].Z()) {
					minp = points[i];
					maxp = points[j];
				}
				else {
					minp = points[j];
					maxp = points[i];
				}

			}
		}
	}
	SbBox3f newbox(minp.X(), minp.Y(), minp.Z(), maxp.X(), maxp.Y(), maxp.Z());

	//gp_Vec min, max = gp_Vec(0, 0, 0);
	//for (auto it = boxes.begin(); it != boxes.end(); it++)
	//{
	//	float X, Y, Z;
	//	(*it).getMin().getValue(X, Y, Z);
	//	gp_Vec point(X, Y, Z);
	//	if (min.Magnitude() < point.Magnitude()) min.SetXYZ(point.XYZ());

	//	(*it).getMax().getValue(X, Y, Z);
	//	point = gp_Vec(X, Y, Z);
	//	if (max.Magnitude() < point.Magnitude()) max.SetXYZ(point.XYZ());
	//}
	//SbBox3f box(min.X(), min.Y(), min.Z(), max.X(), max.Y(), max.Z());

	SoCamera* cam = this->getSoRenderManager()->getCamera();
#if (COIN_MAJOR_VERSION >= 4)
	float aspectratio = getSoRenderManager()->getViewportRegion().getViewportAspectRatio();
	switch (cam->viewportMapping.getValue()) {
	case SoCamera::CROP_VIEWPORT_FILL_FRAME:
	case SoCamera::CROP_VIEWPORT_LINE_FRAME:
	case SoCamera::CROP_VIEWPORT_NO_FRAME:
		aspectratio = 1.0f;
		break;
	default:
		break;
	}
	cam->viewBoundingBox(newbox, aspectratio, 1.0);
#else
	SoTempPath path(2);
	path.ref();
	auto pcGroup = new SoGroup;
	pcGroup->ref();
	auto pcTransform = new SoTransform;
	pcGroup->addChild(pcTransform);
	pcTransform->translation = box.getCenter();
	auto *pcCube = new SoCube;
	pcGroup->addChild(pcCube);
	float sizeX, sizeY, sizeZ;
	box.getSize(sizeX, sizeY, sizeZ);
	pcCube->width = sizeX;
	pcCube->height = sizeY;
	pcCube->depth = sizeZ;
	path.append(pcGroup);
	path.append(pcCube);
	cam->viewAll(&path, getSoRenderManager()->getViewportRegion());
	path.unrefNoDelete();
	pcGroup->unref();
#endif
}

void View3DInventor::setSceneGraph(SoNode* root)
{
	inherited::setSceneGraph(root);
	if (!root) {
		_ViewProviderSet.clear();
		_ViewProviderMap.clear();
		editViewProvider = 0;
	}

	SoSearchAction sa;
	sa.setNode(this->backlight);
	//we want the rendered scene with all lights and cameras, viewer->getSceneGraph would return
	//the geometry scene only
	SoNode* scene = this->getSoRenderManager()->getSceneGraph();
	if (scene && scene->getTypeId().isDerivedFrom(SoSeparator::getClassTypeId())) {
		sa.apply(scene);
		if (!sa.getPath())
			static_cast<SoSeparator*>(scene)->insertChild(this->backlight, 0);
	}
}

bool View3DInventor::pickPoint(const SbVec2s& pos, SbVec3f& point, SbVec3f& norm) const
{
	// attempting raypick in the event_cb() callback method
	SoRayPickAction rp(getSoRenderManager()->getViewportRegion());
	rp.setPoint(pos);
	rp.apply(getSoRenderManager()->getSceneGraph());
	SoPickedPoint* Point = rp.getPickedPoint();

	if (Point) {
		point = Point->getObjectPoint();
		norm = Point->getObjectNormal();
		return true;
	}

	return false;
}

SoPickedPoint* View3DInventor::pickPoint(const SbVec2s& pos) const
{
	SoRayPickAction rp(getSoRenderManager()->getViewportRegion());
	rp.setPoint(pos);
	rp.apply(getSoRenderManager()->getSceneGraph());

	// returns a copy of the point
	SoPickedPoint* pick = rp.getPickedPoint();
	//return (pick ? pick->copy() : 0); // needs the same instance of CRT under MS Windows
	return (pick ? new SoPickedPoint(*pick) : 0);
}

const SoPickedPoint* View3DInventor::getPickedPoint(SoEventCallback* n) const
{
	if (selectionRoot) {
		auto ret = selectionRoot->getPickedList(n->getAction(), true);
		if (ret.size()) return ret[0].pp;
		return nullptr;
	}
	return n->getPickedPoint();
}

SbBool View3DInventor::pubSeekToPoint(const SbVec2s& pos)
{
	return this->seekToPoint(pos);
}

void View3DInventor::pubSeekToPoint(const SbVec3f& pos)
{
	this->seekToPoint(pos);
}

void View3DInventor::addEventCallback(SoType eventtype, SoEventCallbackCB* cb, void* userdata)
{
	pEventCallback->addEventCallback(eventtype, cb, userdata);
}

void View3DInventor::removeEventCallback(SoType eventtype, SoEventCallbackCB* cb, void* userdata)
{
	pEventCallback->removeEventCallback(eventtype, cb, userdata);
}

void View3DInventor::setAxisCross(bool on)
{
	SoNode* scene = getSceneGraph();
	SoSeparator* sep = static_cast<SoSeparator*>(scene);

	if (on) {
		if (!axisGroup) {
			axisCross = new SoShapeScale;
			SoAxisCrossKit* axisKit = new SoAxisCrossKit();
			axisKit->set("xAxis.appearance.drawStyle", "lineWidth 2");
			axisKit->set("yAxis.appearance.drawStyle", "lineWidth 2");
			axisKit->set("zAxis.appearance.drawStyle", "lineWidth 2");
			axisCross->setPart("shape", axisKit);
			axisCross->scaleFactor = 1.0f;
			axisGroup = new SoSkipBoundingGroup;
			axisGroup->addChild(axisCross);

			sep->addChild(axisGroup);
		}
	}
	else {
		if (axisGroup) {
			sep->removeChild(axisGroup);
			axisGroup = 0;
		}
	}
}

bool View3DInventor::hasAxisCross(void)
{
	return axisGroup;
}

void View3DInventor::setFeedbackSize(const int size)
{
	if (size < 1) {
		return;
	}

	this->axiscrossSize = size;

	if (this->isFeedbackVisible() && this->isViewing()) {
		this->getSoRenderManager()->scheduleRedraw();
	}
}

/*!
  Return the size of the feedback axis cross. Default is 10.
*/
int View3DInventor::getFeedbackSize(void) const
{
	return this->axiscrossSize;
}

SbBool View3DInventor::isFeedbackVisible(void) const
{
	return this->axiscrossEnabled;
}

void View3DInventor::setFeedbackVisibility(const SbBool enable)
{
	if (enable == this->axiscrossEnabled) {
		return;
	}

	this->axiscrossEnabled = enable;

	if (this->isViewing()) {
		this->getSoRenderManager()->scheduleRedraw();
	}
}

static GLubyte xbmp[] = { 0x11,0x11,0x0a,0x04,0x0a,0x11,0x11 };
static GLubyte ybmp[] = { 0x04,0x04,0x04,0x04,0x0a,0x11,0x11 };
static GLubyte zbmp[] = { 0x1f,0x10,0x08,0x04,0x02,0x01,0x1f };

void View3DInventor::drawAxisCross(void)
{
	// FIXME: convert this to a superimposition scenegraph instead of
	// OpenGL calls. 20020603 mortene.

	// Store GL state.
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	GLfloat depthrange[2];
	glGetFloatv(GL_DEPTH_RANGE, depthrange);
	GLdouble projectionmatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projectionmatrix);

	glDepthFunc(GL_ALWAYS);
	glDepthMask(GL_TRUE);
	glDepthRange(0, 0);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_BLEND); // Kills transparency.

	// Set the viewport in the OpenGL canvas. Dimensions are calculated
	// as a percentage of the total canvas size.
	SbVec2s view = this->getSoRenderManager()->getSize();
	const int pixelarea =
		int(float(this->axiscrossSize) / 100.0f * std::min(view[0], view[1]));
#if 0 // middle of canvas
	SbVec2s origin(view[0] / 2 - pixelarea / 2, view[1] / 2 - pixelarea / 2);
#endif // middle of canvas
#if 1 // lower right of canvas
	SbVec2s origin(view[0] - pixelarea, 0);
#endif // lower right of canvas
	glViewport(origin[0], origin[1], pixelarea, pixelarea);

	// Set up the projection matrix.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const float NEARVAL = 0.1f;
	const float FARVAL = 10.0f;
	const float dim = NEARVAL * float(tan(M_PI / 8.0)); // FOV is 45 deg (45/360 = 1/8)
	glFrustum(-dim, dim, -dim, dim, NEARVAL, FARVAL);


	// Set up the model matrix.
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	SbMatrix mx;
	SoCamera* cam = this->getSoRenderManager()->getCamera();

	// If there is no camera (like for an empty scene, for instance),
	// just use an identity rotation.
	if (cam) {
		mx = cam->orientation.getValue();
	}
	else {
		mx = SbMatrix::identity();
	}

	mx = mx.inverse();
	mx[3][2] = -3.5; // Translate away from the projection point (along z axis).
	glLoadMatrixf((float*)mx);


	// Find unit vector end points.
	SbMatrix px;
	glGetFloatv(GL_PROJECTION_MATRIX, (float*)px);
	SbMatrix comb = mx.multRight(px);

	SbVec3f xpos;
	comb.multVecMatrix(SbVec3f(1, 0, 0), xpos);
	xpos[0] = (1 + xpos[0]) * view[0] / 2;
	xpos[1] = (1 + xpos[1]) * view[1] / 2;
	SbVec3f ypos;
	comb.multVecMatrix(SbVec3f(0, 1, 0), ypos);
	ypos[0] = (1 + ypos[0]) * view[0] / 2;
	ypos[1] = (1 + ypos[1]) * view[1] / 2;
	SbVec3f zpos;
	comb.multVecMatrix(SbVec3f(0, 0, 1), zpos);
	zpos[0] = (1 + zpos[0]) * view[0] / 2;
	zpos[1] = (1 + zpos[1]) * view[1] / 2;


	// Render the cross.
	{
		glLineWidth(2.0);

		enum { XAXIS, YAXIS, ZAXIS };
		int idx[3] = { XAXIS, YAXIS, ZAXIS };
		float val[3] = { xpos[2], ypos[2], zpos[2] };

		// Bubble sort.. :-}
		if (val[0] < val[1]) {
			std::swap(val[0], val[1]);
			std::swap(idx[0], idx[1]);
		}

		if (val[1] < val[2]) {
			std::swap(val[1], val[2]);
			std::swap(idx[1], idx[2]);
		}

		if (val[0] < val[1]) {
			std::swap(val[0], val[1]);
			std::swap(idx[0], idx[1]);
		}

		assert((val[0] >= val[1]) && (val[1] >= val[2])); // Just checking..

		for (int i = 0; i < 3; i++) {
			glPushMatrix();

			if (idx[i] == XAXIS) {                        // X axis.
				if (stereoMode() != Quarter::SoQTQuarterAdaptor::MONO)
					glColor3f(0.500f, 0.5f, 0.5f);
				else
					glColor3f(0.500f, 0.125f, 0.125f);
			}
			else if (idx[i] == YAXIS) {                   // Y axis.
				glRotatef(90, 0, 0, 1);

				if (stereoMode() != Quarter::SoQTQuarterAdaptor::MONO)
					glColor3f(0.400f, 0.4f, 0.4f);
				else
					glColor3f(0.125f, 0.500f, 0.125f);
			}
			else {                                        // Z axis.
				glRotatef(-90, 0, 1, 0);

				if (stereoMode() != Quarter::SoQTQuarterAdaptor::MONO)
					glColor3f(0.300f, 0.3f, 0.3f);
				else
					glColor3f(0.125f, 0.125f, 0.500f);
			}

			this->drawArrow();
			glPopMatrix();
		}
	}

	// Render axis notation letters ("X", "Y", "Z").
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, view[0], 0, view[1], -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	GLint unpack;
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &unpack);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	if (stereoMode() != Quarter::SoQTQuarterAdaptor::MONO)
		glColor3fv(SbVec3f(1.0f, 1.0f, 1.0f).getValue());
	else
		glColor3fv(SbVec3f(0.0f, 0.0f, 0.0f).getValue());

	glRasterPos2d(xpos[0], xpos[1]);
	glBitmap(8, 7, 0, 0, 0, 0, xbmp);
	glRasterPos2d(ypos[0], ypos[1]);
	glBitmap(8, 7, 0, 0, 0, 0, ybmp);
	glRasterPos2d(zpos[0], zpos[1]);
	glBitmap(8, 7, 0, 0, 0, 0, zbmp);

	glPixelStorei(GL_UNPACK_ALIGNMENT, unpack);
	glPopMatrix();

	// Reset original state.

	// FIXME: are these 3 lines really necessary, as we push
	// GL_ALL_ATTRIB_BITS at the start? 20000604 mortene.
	glDepthRange(depthrange[0], depthrange[1]);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(projectionmatrix);

	glPopAttrib();
}

void View3DInventor::drawArrow(void)
{
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glEnd();
	glDisable(GL_CULL_FACE);
	glBegin(GL_TRIANGLES);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, +0.5f / 4.0f, 0.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, -0.5f / 4.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, 0.0f, +0.5f / 4.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, 0.0f, -0.5f / 4.0f);
	glEnd();
	glBegin(GL_QUADS);
	glVertex3f(1.0f - 1.0f / 3.0f, +0.5f / 4.0f, 0.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, 0.0f, +0.5f / 4.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, -0.5f / 4.0f, 0.0f);
	glVertex3f(1.0f - 1.0f / 3.0f, 0.0f, -0.5f / 4.0f);
	glEnd();
}

void View3DInventor::customEvent(QEvent * e)
{
	if (e->type() == QEvent::User) {
		NavigationStyleEvent* se = static_cast<NavigationStyleEvent*>(e);
		if (navigation->getClassTypeId() == se->style())
			return;
		setNavigationType(se->style());
	}
	QObject::customEvent(e);
}

void View3DInventor::doPopMenuAction(Base::Type type)
{
	Base::BaseClass* base = static_cast<Base::BaseClass*>(type.createInstance());
}

void View3DInventor::setGradientBackgroundColor(const SbColor& fromColor,
	const SbColor& toColor)
{
	pcBackGround->setColorGradient(fromColor, toColor);
}

void View3DInventor::setGradientBackgroundColor(const SbColor& fromColor,
	const SbColor& toColor,
	const SbColor& midColor)
{
	pcBackGround->setColorGradient(fromColor, toColor, midColor);
}

bool View3DInventor::hasGradientBackground() const
{
	return (backgroundroot->findChild(pcBackGround) != -1);
}

void View3DInventor::setBackGround()
{
	pcViewProviderRoot->addChild(backgroundroot);
}

void View3DInventor::setEditing(SbBool edit)
{
	this->editing = edit;
	this->getWidget()->setCursor(QCursor(Qt::ArrowCursor));
	this->editCursor = QCursor();
}

void View3DInventor::setOverrideMode(const std::string& mode)
{
	if (mode == overrideMode)
		return;

	overrideMode = mode;

	std::vector<ViewProvider*> views;
	for (auto vp : _ViewProviderSet)
		views.push_back(vp);

	if (mode == "No Shading") {
		this->shading = false;
		std::string flatLines = "Flat Lines";
		for (auto view : views)
			view->setOverrideMode(flatLines);
		this->getSoRenderManager()->setRenderMode(SoRenderManager::AS_IS);
	}
	else if (mode == "Hidden Line") {
		this->shading = true;
		std::string shaded = "Shaded";
		for (auto view : views)
			view->setOverrideMode(shaded);
		this->getSoRenderManager()->setRenderMode(SoRenderManager::HIDDEN_LINE);
	}
	else {
		this->shading = true;
		for (auto view : views)
			view->setOverrideMode(mode);
		this->getSoRenderManager()->setRenderMode(SoRenderManager::AS_IS);
	}
}

void View3DInventor::updateOverrideMode(const std::string& mode)
{
	if (mode == overrideMode)
		return;

	overrideMode = mode;

	if (mode == "No Shading") {
		this->shading = false;
		this->getSoRenderManager()->setRenderMode(SoRenderManager::AS_IS);
	}
	else if (mode == "Hidden Line") {
		this->shading = true;
		this->getSoRenderManager()->setRenderMode(SoRenderManager::HIDDEN_LINE);
	}
	else {
		this->shading = true;
		this->getSoRenderManager()->setRenderMode(SoRenderManager::AS_IS);
	}
}

void View3DInventor::removeAllViewprovider()
{
	//if (_CoinMap.size() == 0) return;
	if (_ViewProviderSet.size() == 0) return;
	for (auto pair : _CoinMap) {
		vpGroup->removeChild(pair.first);
		//delete pair.second;
		//pair.second = nullptr;
	}
	_CoinMap.clear();

	for (auto vp : _ViewProviderSet) {
		if (vp) delete vp;
	}
	_ViewProviderSet.clear();
}

void View3DInventor::removeViewprovider(ViewProvider* vp)
{
	for (auto iter = _ViewProviderSet.begin(); iter != _ViewProviderSet.end(); iter++) {
		if (*iter == vp) {
			for (auto it = _CoinMap.begin(); it != _CoinMap.end(); it++) {
				if (it->second == vp) {
					vpGroup->removeChild(it->first);
					_CoinMap.erase(it);
					_ViewProviderSet.erase(vp);
					return;
				}
			}
		}
	}
}

void View3DInventor::setEnabledVBO(bool on)
{
	vboEnabled = on;
}

bool View3DInventor::isEnabledVBO() const
{
	return vboEnabled;
}

void View3DInventor::setEditingCursor(const QCursor& cursor)
{
	this->getWidget()->setCursor(cursor);
	this->editCursor = this->getWidget()->cursor();
}

SbBool View3DInventor::isEditing() const
{
	return this->editing;
}

SbBool View3DInventor::isRedirectedToSceneGraph() const
{ 
	return this->redirected; 
}

std::string View3DInventor::getOverrideMode() const
{ 
	return overrideMode; 
}

const std::set<ViewProvider*> View3DInventor::getVps() const
{ 
	return _ViewProviderSet; 
};