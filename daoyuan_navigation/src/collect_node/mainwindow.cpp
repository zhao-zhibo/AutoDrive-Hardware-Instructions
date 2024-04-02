#include "mainwindow.h"
#if defined(__linux__) || defined(__APPLE__)
#include <unistd.h>
#endif
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QTableWidget>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QWidget>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include "ui_mainwindow.h"
#ifdef _WIN32
#include "direct.h"
#define getcwd(s, le) _getcwd(s, le)
#define chdir(x) _chdir(x)
#endif

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      timer(new QTimer(parent)),
      onSavePath(false),
      spinner(1),
      dialog(nullptr) {
    ui->setupUi(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout()));
    // 特殊用法
    QTimer::singleShot(10, this, SLOT(mainwindow_ready()));
    locReceiver = n.subscribe("IAU/Location", 1, &MainWindow::location_callback, this);
}

MainWindow::~MainWindow() {
    spinner.stop();
    timer->stop();
    delete ui;
    locReceiver.shutdown();
}

void MainWindow::location_callback(const iau_ros_msgs::Location &loc)
{
    {
        std::lock_guard<std::mutex> _guard(mutex);
        location = loc;
    }

    if (onSavePath) {
        QTextStream in(&filePath);
        in << qSetRealNumberPrecision(13) << loc.timestamp.toSec() << "\t"
           << loc.wgs84_pos[0] << "\t" << loc.wgs84_pos[1] << "\t"
           << loc.gau_pos[0] << "\t" << loc.gau_pos[1] << "\t"
           << loc.wgs84_pos[2] << "\t" << loc.orientation[0] << "\t"
           << loc.orientation[1] << "\t" << loc.orientation[2] << "\t"
           << loc.speed[0] << "\t"
           << loc.speed[1] << "\t" << loc.speed[2]
           << "\n";
    }
    if (dialog) {
        QPointF pt;
        pt.setX(loc.gau_pos[0]);
        pt.setY(loc.gau_pos[1]);
        dialog->addPoint(pt, loc.orientation[2]);
    }
   
}

void MainWindow::mainwindow_ready() {
    timer->start(40);
    spinner.start();
}

void MainWindow::on_btnSelectFile_clicked() {
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::DontUseNativeDialog);
    if (dialog.exec()) {
        QString directory = dialog.selectedFiles()[0];
        QDateTime dateTime = QDateTime::currentDateTime();
        directory.append("/NovAtel");
        directory.append(dateTime.date().toString("yyyy-MM-dd"));
        directory.append("-");
        directory.append(dateTime.time().toString("HH-mm-ss"));
        directory.append(".txt");
        ui->lineSavePath->setText(directory);
    }
}

void MainWindow::handleTimeout() {
    {
        iau_ros_msgs::Location* msg = NULL ;
        iau_ros_msgs::Location loc ;

        {
            std::lock_guard<std::mutex> _guard(mutex);
            loc = this->location;
            msg = &loc;
        }
      
        ui->tableWPosition->setItem(0, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->timestamp.toSec()))));

         

        ui->tableWPosition->setItem(
            2, 0,
            new QTableWidgetItem(QString::fromStdString(
                to_string_with_precision(msg->wgs84_pos[0], 13))));

        
        ui->tableWPosition->setItem(
            1, 0,
            new QTableWidgetItem(QString::fromStdString(
                to_string_with_precision(msg->wgs84_pos[1], 13))));

        ui->tableWPosition->setItem(3, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->wgs84_pos[2]))));
        ui->tableWPosition->setItem(4, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->speed[1]))));
        ui->tableWPosition->setItem(5, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->speed[0]))));
        ui->tableWPosition->setItem(6, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->speed[2]))));
        ui->tableWPosition->setItem(7, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->orientation[1]))));
        ui->tableWPosition->setItem(8, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->orientation[0]))));
        ui->tableWPosition->setItem(9, 0,
                                    new QTableWidgetItem(QString::fromStdString(
                                        std::to_string(msg->orientation[2]))));
    }
}

void MainWindow::on_btnSavePath_clicked() {
    if (!onSavePath) {
        QFileInfo path(ui->lineSavePath->text());
        QDir dir;
        if (dir.mkpath(path.path())) {
            filePath.setFileName(path.filePath());
            if (filePath.open(QFile::OpenModeFlag::WriteOnly |
                              QFile::OpenModeFlag::Append)) {
                ui->btnSavePath->setText("Stop");
                onSavePath = true;
                return;
            }
        }
        onSavePath = false;
        std::cout << "Counld not create path " << path.filePath().toStdString()
                  << std::endl;
        QMessageBox::information(nullptr, "Error", "Counld not create path");

    } else {
        onSavePath = false;
        ui->btnSavePath->setText("Save");
        filePath.close();
    }
}

#pragma region MissionPoint
void MainWindow::on_btnAddPoint_clicked() {
    iau_ros_msgs::Location* msg = NULL;
    iau_ros_msgs::Location loc ;
    {
        std::lock_guard<std::mutex> _guard(mutex);
        loc = this->location;
        msg = &loc;
    }
      
    ui->tableWPoint->insertRow(ui->tableWPoint->rowCount());
    ui->tableWPoint->setItem(
        ui->tableWPoint->rowCount() - 1, 0,
        new QTableWidgetItem(QString::fromStdString(
            to_string_with_precision(msg->wgs84_pos[1], 13))));
    ui->tableWPoint->setItem(
        ui->tableWPoint->rowCount() - 1, 1,
        new QTableWidgetItem(QString::fromStdString(
            to_string_with_precision(msg->wgs84_pos[0], 13))));
    ui->tableWPoint->setItem(
        ui->tableWPoint->rowCount() - 1, 2,
        new QTableWidgetItem(QString::fromStdString(
            std::to_string(ui->cbProperty1->currentIndex()))));
    ui->tableWPoint->setItem(
        ui->tableWPoint->rowCount() - 1, 3,
        new QTableWidgetItem(QString::fromStdString(
            std::to_string(ui->cbProperty2->currentIndex()))));
    ui->tableWPoint->scrollToBottom();
}

void MainWindow::on_btnDeletePoint_clicked() {
    ui->tableWPoint->removeRow(ui->tableWPoint->rowCount() - 1);
}

void MainWindow::on_btnSavePoint_clicked() {
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::FileMode::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
    dialog.setOption(QFileDialog::DontUseNativeDialog);
    QDateTime dateTime = QDateTime::currentDateTime();
    QString fileName;
    fileName.append("MissionPoint");
    fileName.append(dateTime.date().toString("yyyy-MM-dd"));
    fileName.append("-");
    fileName.append(dateTime.time().toString("HH-mm-ss"));
    fileName.append(".txt");
    dialog.selectFile(fileName);
    if (dialog.exec()) {
        fileName = dialog.selectedFiles()[0];
        QFile f;
        f.setFileName(fileName);
        f.open(QFile::OpenModeFlag::WriteOnly);
        QTextStream ts(&f);
        ts << qSetRealNumberPrecision(13);
        for (int i = 0; i < ui->tableWPoint->rowCount(); ++i) {
            ts << ui->tableWPoint->item(i, 0)->text() << "\t"
               << ui->tableWPoint->item(i, 1)->text() << "\t"
               << ui->tableWPoint->item(i, 2)->text() << "\t"
               << ui->tableWPoint->item(i, 3)->text() << "\n";
        }
        f.close();
    }
}
#pragma endregion
void MainWindow::on_btnShow_clicked() {
    if (dialog) {
        dialog->close();
    } else {
        dialog = new PointShowDialog(this);
        connect(dialog, SIGNAL(closing()), this,
                SLOT(pointshowdialog_closed()));
        dialog->show();
    }
}

void MainWindow::pointshowdialog_closed() {
    dialog->deleteLater();
    dialog = nullptr;
}
