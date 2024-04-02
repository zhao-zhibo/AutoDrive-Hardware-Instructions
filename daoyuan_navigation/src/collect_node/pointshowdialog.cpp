#include "pointshowdialog.h"
#include "ui_pointshowdialog.h"
#include <QCloseEvent>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QScrollBar>
#include <string>
#include <iomanip>
#include <sstream>
static QPointF newpt(0,0);
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}
PointShowDialog::PointShowDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PointShowDialog)
{
    ui->setupUi(this);
    QPolygonF myPolygon1;
      myPolygon1 << QPointF(0,10) << QPointF(20,10);
      QPolygonF myPolygon2;
      myPolygon2 << QPointF(10,0) << QPointF(10,20);
      QPixmap pixmap(20, 20);
      pixmap.fill(Qt::black);
      QPainter painter(&pixmap);

      QVector<qreal> dashes;//line style--虚线
      qreal space = 2;
      dashes << 2 << space << 2 <<space;
      QPen pen(Qt::lightGray,1);
      pen.setDashPattern(dashes);
      pen.setWidth(1);

      painter.setPen(pen);
      painter.translate(0, 0);
      painter.drawPolyline(myPolygon1);
      painter.drawPolyline(myPolygon2);
      scene=new QGraphicsScene();
      scene->setBackgroundBrush(pixmap);
      ui->graphicsView->setScene(scene);
      ui->graphicsView->centerOn(0,0);
//      scene->addRect(QRect(0,0,100,100),QPen(),QBrush(Qt::white));
      connect(ui->btReset,SIGNAL(clicked(bool)),ui->graphicsView,SLOT(reset()));
      connect(ui->graphicsView,SIGNAL(onSelected(QPointF,double)),this,SLOT(onSelected(QPointF,double)));
}

PointShowDialog::~PointShowDialog()
{
    delete ui;
}

void PointShowDialog::onSelected(QPointF pt,double azimuth){
    std::string text="x:"+to_string_with_precision(pt.x()/40.0,13)+"\n"
            +"y:"+to_string_with_precision(pt.y()/40.0,13)+"\n"
            +"azimuth:"+to_string_with_precision(azimuth,13);
    ui->txPoint->setText(QString::fromStdString(text));
}

void PointShowDialog::closeEvent(QCloseEvent *event){
    event->accept();
    emit closing();
}

void PointShowDialog::addPoint(QPointF pt,double azimuth){
    pt.rx()*=40;
    pt.ry()*=40;
    ui->graphicsView->addPoint(pt,azimuth);
}
