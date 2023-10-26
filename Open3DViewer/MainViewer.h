#pragma once
#include <QMainWindow>
#include <QLayout>
#include <Open3DViewer\DLLConfig.h>
#include <Open3DViewer\View3DInventor.h>

class DLLExport MainViewer : public QMainWindow
{
public:
	MainViewer(QWidget *parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());
	virtual ~MainViewer();

	virtual View3DInventor* getInventor() const;
	virtual void setInventor();
	virtual void initClass();
	virtual void setUi();

public:
	static MainViewer* mainViewer;
	//add new menus in the mouse right button menu
	QList<QMenu*> *menus;
	//add new actions in the mouse right button menu
	QList<QAction*> *actionsList;
protected:
	View3DInventor* viewInventor;
	QVBoxLayout* layout;
};

