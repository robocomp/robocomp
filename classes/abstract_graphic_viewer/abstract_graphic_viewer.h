//
// Created by pbustos on 19/9/21.
//

#ifndef GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H
#define GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QGLWidget>
#include <QScrollBar>
#include <QApplication>
#include <QVBoxLayout>
#include <QGraphicsPolygonItem>
#include <iostream>


class AbstractGraphicViewer : public QGraphicsView
{
    Q_OBJECT
    private:
        qreal m_scaleX, m_scaleY;
        QGraphicsPolygonItem *robot_polygon;
        QGraphicsEllipseItem *laser_in_robot_sr;

    public:
        AbstractGraphicViewer(QWidget *parent, QRectF dim_);
        std::tuple<QGraphicsPolygonItem*, QGraphicsEllipseItem*> add_robot(float robot_width,
                                                                           float robot_length,
                                                                           float laser_x_offset = 0,
                                                                           float laser_y_offset= 100,
                                                                           QColor color= QColor("Blue"));
        void draw_contour();
        QGraphicsScene scene;
        QGraphicsPolygonItem* robot_poly();
        QGraphicsEllipseItem* laser_in_robot();

signals:
      void new_mouse_coordinates(QPointF);

    protected:
        bool _pan = false;
        int _panStartX, _panStartY;
        virtual void wheelEvent(QWheelEvent *event);
        virtual void resizeEvent(QResizeEvent *e);
        virtual void mouseMoveEvent(QMouseEvent *event);
        virtual void mousePressEvent(QMouseEvent *event);
        virtual void mouseReleaseEvent(QMouseEvent *event);
};
#endif //GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H
