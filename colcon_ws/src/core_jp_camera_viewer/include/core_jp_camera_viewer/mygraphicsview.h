#ifndef MY_GRAPHICS_VIEW_H
#define MY_GRAPHICS_VIEW_H

#include <QGraphicsView>
#include <QOpenGLWidget>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QKeyEvent>

class MyGraphicsView : public QGraphicsView
{
   Q_OBJECT
public:
   MyGraphicsView(QWidget *pWnd = nullptr) : QGraphicsView(pWnd)
   {
      setViewport(new QOpenGLWidget);
   };
   ~MyGraphicsView(void){};

   void setImg(const QImage &img, const std::string &msg = "");

private:
   void paintEvent(QPaintEvent *event);

   QImage m_img;
   QTransform m_matrix;
   std::string m_msg;
};

#endif // MY_GRAPHICS_VIEW_H
