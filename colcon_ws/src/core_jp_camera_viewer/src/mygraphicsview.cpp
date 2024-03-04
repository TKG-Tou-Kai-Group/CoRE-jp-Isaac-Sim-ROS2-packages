#include <iostream>
#include "core_jp_camera_viewer/mygraphicsview.h"

void MyGraphicsView::paintEvent(QPaintEvent *event)
{
    if (m_img.isNull())
    {
        return;
    }
    QPainter widgetpainter(viewport());
    widgetpainter.setWorldTransform(m_matrix);

    QImage qimg = m_img.scaled(viewport()->width(), viewport()->height(), Qt::KeepAspectRatio, Qt::FastTransformation);
    widgetpainter.drawImage(0, 0, qimg);
}

void MyGraphicsView::setImg(const QImage &img, const std::string &msg)
{
    m_img = img.copy();
    m_msg = msg;
    viewport()->update();
}
