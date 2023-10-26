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
#pragma once
#ifndef VIEWINVENTOR_H
#define VIEWINVENTOR_H

#include <Open3DViewer\DLLConfig.h>

#include <set>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/SbRotation.h>
#include <Inventor/nodes/SoClipPlane.h>
#include <Quarter/eventhandlers/EventFilter.h>
#include <Quarter\SoQTQuarterAdaptor.h>

#include <Open3DViewer/QtOpenGL.h>
#include <Open3DViewer/SoTouchEvents.h>
#include <Open3DViewer/SoFCInteractiveElement.h>
#include <Open3DViewer/SoFCUnifiedSelection.h>
#include <Open3DViewer/SoFCBackgroundGradient.h>
#include <Open3DViewer/ViewProviderPartExt.h>
#include <Open3DViewer/Selection.h>
#include <Open3DViewer/Matrix.h>
#include <Open3DViewer/Placement.h>
#include <Open3DViewer/GLPainter.h>
#include <Open3DViewer/NavigationStyle.h>
#include <Open3DViewer/SoAxisCrossKit.h>
#include <Open3DViewer/Measure.h>

class SbBox2s;
class QOpenGLFramebufferObject;
class GLGraphicsItem;
class ViewerEventFilter;

class DLLExport View3DInventor : public SIM::Coin3D::Quarter::SoQTQuarterAdaptor, public SelectionObserver
{
	typedef SIM::Coin3D::Quarter::SoQTQuarterAdaptor inherited;

public:
	/// Pick modes for picking points in the scene
	enum SelectionMode {
		Lasso = 0,  /**< Select objects using a lasso. */
		Rectangle = 1,  /**< Select objects using a rectangle. */
		Rubberband = 2,  /**< Select objects using a rubberband. */
		BoxZoom = 3,  /**< Perform a box zoom. */
		Clip = 4,  /**< Clip objects using a lasso. */
	};

	enum SelectionRole {
		None = 0,
		Inner = 1,
		Outer = 2,
		Split = 3,
		Custom0 = 4,
		Custom1 = 5,
		Custom2 = 6,
	};

	enum ViewerMod {
		ShowCoord = 1,       /**< Enables the Coordinate system in the corner. */
		ShowFPS = 2,       /**< Enables the Frams per Second counter. */
		SimpleBackground = 4,/**< switch to a simple background. */
		DisallowRotation = 8,/**< switch off the rotation. */
		DisallowPanning = 16,/**< switch off the panning. */
		DisallowZooming = 32,/**< switch off the zooming. */
	};

	enum RenderType {
		Native,
		Framebuffer,
		Image
	};

	View3DInventor(QWidget *parent, const QtGLWidget* sharewidget = 0);
	View3DInventor(const QtGLFormat& format, QWidget *parent, const QtGLWidget* sharewidget = 0);
	virtual ~View3DInventor();
	/// adds an ViewProvider to the view, e.g. from a feature
	void addViewProvider(ViewProvider*);
	void init();
	void setEditing(SbBool edit);
	SbBool isEditing() const;

	/// Observer message from the Selection
	virtual void onSelectionChanged(const SelectionChanges &Reason);
	void checkGroupOnTop(const SelectionChanges &Reason);
	void clearGroupOnTop();
	SoDirectionalLight* getBacklight(void) const;
	void setBacklight(SbBool on);
	void setSceneGraph(SoNode *root);
	void setAnimationEnabled(const SbBool enable);
	SbBool isAnimationEnabled(void) const;
	void setPopupMenuEnabled(const SbBool on);
	SbBool isPopupMenuEnabled(void) const;
	void startAnimating(const SbVec3f& axis, float velocity);
	void stopAnimating(void);
	SbBool isAnimating(void) const;
	void setFeedbackVisibility(const SbBool enable);
	SbBool isFeedbackVisible(void) const;
	void setFeedbackSize(const int size);
	int getFeedbackSize(void) const;
	/// Get the preferred samples from the user settings
	void setRenderType(const RenderType type);
	void renderToFramebuffer(QOpenGLFramebufferObject*);
	virtual void setViewing(SbBool enable);
	virtual void setCursorEnabled(SbBool enable);
	void addGraphicsItem(GLGraphicsItem*);
	void removeGraphicsItem(GLGraphicsItem*);
	/** @name Selection methods */
	//@{
	void startSelection(SelectionMode = Lasso);
	void stopSelection();
	bool isSelecting() const;

	/** @name Pick actions */
	//@{
	// calls a PickAction on the scene graph
	bool pickPoint(const SbVec2s& pos, SbVec3f &point, SbVec3f &norm) const;
	SoPickedPoint* pickPoint(const SbVec2s& pos) const;
	const SoPickedPoint* getPickedPoint(SoEventCallback * n) const;
	SbBool pubSeekToPoint(const SbVec2s& pos);
	void pubSeekToPoint(const SbVec3f& pos);
	//@}

	/**
	 * Set up a callback function \a cb which will be invoked for the given eventtype.
	 * \a userdata will be given as the first argument to the callback function.
	 */
	void addEventCallback(SoType eventtype, SoEventCallbackCB * cb, void* userdata = 0);
	/**
	 * Unregister the given callback function \a cb.
	 */
	void removeEventCallback(SoType eventtype, SoEventCallbackCB * cb, void* userdata = 0);

	/** @name Dimension controls
	 * the "turn*" functions are wired up to parameter groups through view3dinventor.
	 * don't call them directly. instead set the parameter groups.
	 * @see TaskDimension
	 */
	 //@{
	virtual void turnAllDimensionsOn();
	virtual void turnAllDimensionsOff();
	virtual void turn3dDimensionsOn();
	virtual void turn3dDimensionsOff();
	virtual void turnDeltaDimensionsOn();
	virtual void turnDeltaDimensionsOff();
	virtual void eraseAllDimensions();
	virtual void addDimension3d(SoNode *node);
	virtual void addDimensionDelta(SoNode *node);
	//@}

		/**
	 * Set the camera's orientation. If isAnimationEnabled() returns
	 * \a true the reorientation is animated, otherwise its directly
	 * set.
	 */
	void setCameraOrientation(const SbRotation& rot, SbBool moveTocenter = false);
	void setCameraType(SoType t);
	//	void moveCameraTo(const SbRotation& rot, const SbVec3f& pos, int steps, int ms);
		/**
		 * Zooms the viewport to the size of the bounding box.
		 */
	void boxZoom(const SbBox2s&);
	/**
	 * Reposition the current camera so we can see the complete scene.
	 */
	void viewAll();
	void viewAll(float factor);

	/**
	 * Reposition the current camera so we can see all selected objects
	 * of the scene. Therefore we search for all SOFCSelection nodes, if
	 * none of them is selected nothing happens.
	 */
	void viewSelection();

	void setGradientBackground(bool b);
	bool hasGradientBackground() const;
	void setGradientBackgroundColor(const SbColor& fromColor,
		const SbColor& toColor);
	void setGradientBackgroundColor(const SbColor& fromColor,
		const SbColor& toColor,
		const SbColor& midColor);
	void setNavigationType(Base::Type);

	void setAxisCross(bool b);
	bool hasAxisCross(void);

	void setRenderCache(int);
	SbBool isRedirectedToSceneGraph() const;

	NavigationStyle* navigationStyle() const;

	ViewProvider* getViewProviderByPathFromHead(SoPath * path) const;

	void doPopMenuAction(Base::Type type);
	void setBackGround();

	void setOverrideMode(const std::string &mode);
	void updateOverrideMode(const std::string &mode);
	std::string getOverrideMode() const;
	const std::set<ViewProvider*> getVps() const;
	void animatedViewAll(int steps, int ms);
	void removeAllViewprovider();
	void removeViewprovider(ViewProvider* vp);

	void setEnabledVBO(bool on);
	bool isEnabledVBO() const;

	void setEditingCursor(const QCursor& cursor);

protected:
	GLenum getInternalTextureFormat() const;
	void renderScene();
	void renderFramebuffer();
	virtual void actualRedraw(void);
	virtual void setSeekMode(SbBool enable);
	virtual void afterRealizeHook(void);
	SbBool processSoEventBase(const SoEvent * const ev);
	virtual bool processSoEvent(const SoEvent * ev);
	void printDimension();
	void selectAll();
	void customEvent(QEvent* e);

	enum eWinGestureTuneState {
		ewgtsDisabled, //suppress tuning/re-tuning after errors
		ewgtsNeedTuning, //gestures are to be re-tuned upon next event
		ewgtsTuned
	};
	eWinGestureTuneState winGestureTuneState;//See ViewerEventFilter::eventFilter function for explanation

private:
	static void handleEventCB(void * userdata, SoEventCallback * n);
	static void interactionStartCB(void * data, SoQTQuarterAdaptor * viewer);
	static void interactionFinishCB(void * data, SoQTQuarterAdaptor * viewer);

private:
	void drawAxisCross(void);
	static void drawArrow(void);
	void setCursorRepresentation(int mode);
	void createStandardCursors(double);
	void initialize();

public:
	static View3DInventor* inventorInstance;
	std::map<SoSeparator *, ViewProvider*> _CoinMap;

protected:
	std::set<ViewProvider*> _ViewProviderSet;
	std::map<SoSeparator*, ViewProvider*> _ViewProviderMap;
	std::list<GLGraphicsItem*> graphicsItems;
	ViewProvider* editViewProvider;
	SoFCBackgroundGradient *pcBackGround;
	SoSeparator * backgroundroot;
	SoSeparator * foregroundroot;
	SoDirectionalLight* backlight;

	SoSeparator * pcViewProviderRoot;

	SoGroup * pcGroupOnTop;
	SoGroup * pcGroupOnTopSel;
	SoGroup * pcGroupOnTopPreSel;
	std::map<ViewProvider*, SoNode*> objectsOnTop;
	std::map<ViewProvider*, SoNode*> objectsOnTopPreSel;

	SoSeparator * pcEditingRoot;
	SoTransform * pcEditingTransform;
	bool restoreEditingRoot;
	SoEventCallback* pEventCallback;
	NavigationStyle* navigation;
	SoFCUnifiedSelection* selectionRoot;

	SoClipPlane *pcClipPlane;

	RenderType renderType;
	QOpenGLFramebufferObject* framebuffer;
	QImage glImage;
	SbBool shading;
	SoSeparator *dimensionRoot;
	SoSeparator* originAxisRoot;
	SoGroup* vpGroup;

	// small axis cross in the corner
	SbBool axiscrossEnabled;
	int axiscrossSize;
	// big one in the middle
	SoShapeScale* axisCross;
	SoGroup* axisGroup;

	//stuff needed to draw the fps counter
	bool fpsEnabled;
	bool vboEnabled;
	SbBool naviCubeEnabled;

	SbBool editing;
	QCursor editCursor, zoomCursor, panCursor, spinCursor;
	SbBool redirected;
	SbBool allowredir;

	std::string overrideMode;

	ViewerEventFilter* viewerEventFilter;

	// friends
	friend class NavigationStyle;
	friend class GLPainter;
	friend class ViewerEventFilter;
	friend class TreeWidget;

public:
	SoSeparator* getRoot() const;

};


#endif // !VIEWINVENTOR_H
