#include <QApplication>
#include "mainwindow.h"
#include <ros/init.h>

int main(int argc, char* argv[]) {
    ros::init(argc,argv,"iau_ros_collect");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
