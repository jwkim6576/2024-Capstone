#include <QApplication>
#include <QIcon>
#include "palletizing_ui.h"


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  PalletizingRobotUi w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString(
                       ros::this_node::getName()));

  // load the icon from our qrc file and set it as the application icon
  QIcon icon(":/icon/icon");
  w.setWindowIcon(icon);

  w.show();
  return a.exec();
}
