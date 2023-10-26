#include <QApplication>
#include <Open3DViewer\View3DInventor.h>
#include <Open3DViewer\MainViewer.h>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainViewer w;
	w.setUi();
	w.initClass();
	w.setInventor();
	w.show();
	a.exec();
	return 0;
}