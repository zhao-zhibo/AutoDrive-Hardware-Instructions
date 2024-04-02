#ifndef POINTSHOWDIALOG_H
#define POINTSHOWDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QGraphicsScene>
namespace Ui {
class PointShowDialog;
}

class PointShowDialog : public QDialog
{
    Q_OBJECT
signals:
    void closing();
public:
    explicit PointShowDialog(QWidget *parent = nullptr);
    ~PointShowDialog();
    void closeEvent(QCloseEvent *event) override;
public Q_SLOTS:
    void onSelected(QPointF,double azimuth);
    void addPoint(QPointF pt,double azimuth);
private:
    Ui::PointShowDialog *ui;
    QGraphicsScene* scene;
};

#endif // POINTSHOWDIALOG_H
