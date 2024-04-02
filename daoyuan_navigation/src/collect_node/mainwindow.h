
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFile>
#include <QMainWindow>
#include "pointshowdialog.h"
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/spinner.h>
#include <mutex>
#include <iau_ros_msgs/Location.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

   public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

   private:
    void location_callback(const iau_ros_msgs::Location &loc);
   public slots:
    void mainwindow_ready();
    void on_btnSelectFile_clicked();

    //定时显示gps消息
    void handleTimeout();

    void on_btnSavePath_clicked();

    void on_btnAddPoint_clicked();

    void on_btnDeletePoint_clicked();

    void on_btnSavePoint_clicked();

    void on_btnShow_clicked();

    void pointshowdialog_closed();
   private:
    Ui::MainWindow *ui;

    iau_ros_msgs::Location location;
    QTimer *timer;

    bool onSavePath;
    QFile filePath;
    ros::NodeHandle n;
    ros::Subscriber locReceiver;
    ros::AsyncSpinner spinner;
    std::mutex  mutex;
    PointShowDialog* dialog;
};

#endif  // MAINWINDOW_H
