#include <Inventor\SoInteraction.h>
#include <Open3DViewer\SoFCBoundingBox.h>
#include <Open3DViewer\SoFCUnifiedSelection.h>
#include <Open3DViewer\SoFCSelectionAction.h>
#include <Open3DViewer\SoMouseWheelEvent.h>
#include <Open3DViewer\GestureNavigationStyle.h>
#include <Quarter\Quarter.h>
#include "MainViewer.h"

MainViewer* MainViewer::mainViewer = nullptr;

MainViewer::MainViewer(QWidget *parent, Qt::WindowFlags flags)
	:QMainWindow(parent, flags), viewInventor(nullptr), menus(nullptr), actionsList(nullptr)
{
	mainViewer = this;
}

MainViewer::~MainViewer()
{
	delete viewInventor;
	viewInventor = nullptr;
}

void MainViewer::initClass()
{
	SoInteraction::init();
	ViewProvider::init();
	ViewProviderPartExt::init();
	SoDB::init();
	SIM::Coin3D::Quarter::Quarter::init();
	SoGLRenderActionElement::initClass();
	SoFCInteractiveElement::initClass();
	SoGLWidgetElement::initClass();
	SoFCBackgroundGradient::initClass();
	SoFCBoundingBox::initClass();
	SoFCUnifiedSelection::initClass();
	SoFCPathAnnotation::initClass();
	SoFCHighlightAction::initClass();
	SoFCSelectionAction::initClass();
	SoFCDocumentAction::initClass();
	SoGLWidgetNode::initClass();
	SoGLVBOActivatedElement::initClass();
	SoFCEnableSelectionAction::initClass();
	SoFCEnableHighlightAction::initClass();
	SoFCSelectionColorAction::initClass();
	SoFCHighlightColorAction::initClass();
	SoFCDocumentObjectAction::initClass();
	SoGLSelectAction::initClass();
	SoVisibleFaceAction::initClass();
	SoUpdateVBOAction::initClass();
	SoBoxSelectionRenderAction::initClass();
	SoHighlightElementAction::initClass();
	SoSelectionElementAction::initClass();
	SoVRMLAction::initClass();
	SoSkipBoundingGroup::initClass();
	SoFCSeparator::initClass();
	SoBrepFaceSet::initClass();
	SoBrepEdgeSet::initClass();
	SoBrepPointSet::initClass();
	SoSelectionRoot::initClass();
	SoMouseWheelEvent::initClass();
	DimensionLinear::initClass();
	ArcEngine::initClass();
	DimensionAngular::initClass();
	DimensionArea::initClass();
	NavigationStyle::init();
	UserNavigationStyle::init();
	CADNavigationStyle::init();
	GestureNavigationStyle::init();
	InventorNavigationStyle::init();
}

void MainViewer::setUi()
{
	QWidget *centralwidget = new QWidget(this);
	setCentralWidget(centralwidget);
	layout = new QVBoxLayout(centralwidget);
	//this->setLayout(layout);
}

void MainViewer::setInventor()
{
	if (viewInventor) delete viewInventor;
	viewInventor = new View3DInventor(this);
	layout->addWidget(viewInventor);
}

View3DInventor* MainViewer::getInventor() const
{
	return viewInventor;
}