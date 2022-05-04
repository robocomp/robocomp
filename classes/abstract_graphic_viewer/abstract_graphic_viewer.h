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
        AbstractGraphicViewer(QWidget *parent, QRectF dim_)
        {
            QVBoxLayout *vlayout = new QVBoxLayout(parent);
            vlayout->addWidget(this);
            scene.setItemIndexMethod(QGraphicsScene::NoIndex);
            scene.setSceneRect(dim_);
            this->setScene(&scene);
            this->setCacheMode(QGraphicsView::CacheBackground);
            this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
            this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
            this->setRenderHint(QPainter::Antialiasing);
            this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            //this->setMinimumSize(400, 400);
            this->scale(1, -1);
            this->setMouseTracking(true);
            this->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
            this->viewport()->setMouseTracking(true);
            auto r = sceneRect();
            // bounding box
            auto sr = scene.addRect(r, QPen(QColor("Gray"), 100));
            sr->setZValue(15);
            // axis
            QLineF x_axis(r.center(), r.center()+QPointF(300,0));
            QLineF y_axis(r.center(), r.center()+QPointF(0,300));
            scene.addLine(x_axis, QPen(QColor("Red"), 30));
            scene.addLine(y_axis, QPen(QColor("Green"), 30));
            this->adjustSize();
        }
        std::tuple<QGraphicsPolygonItem*, QGraphicsEllipseItem*> add_robot(float robot_width, float robot_length,
                                                                           float laser_x_offset = 0,
                                                                           float laser_y_offset= 100,
                                                                           QColor color= QColor("Blue"))
        {
            float sl = robot_length / 2.f;
            float sw = robot_width / 2.f;
            QPolygonF poly2;
            poly2 << QPoint(-sw, -sl) << QPoint(-sw, sl) << QPoint(sw, sl) << QPoint(sw, -sl);
            QBrush brush(color, Qt::SolidPattern);
            robot_polygon = scene.addPolygon(poly2, QPen(color), brush);
            laser_in_robot_sr = new QGraphicsEllipseItem(-30, -30, 60, 60, robot_polygon);
            laser_in_robot_sr->setBrush(QBrush(QColor("White")));
            scene.addItem(laser_in_robot_sr);
            laser_in_robot_sr->setPos(laser_x_offset, laser_y_offset);
            robot_polygon->setZValue(55);
            robot_polygon->setPos(0, 0);
            return std::make_tuple(robot_polygon, laser_in_robot_sr);
        }
        QGraphicsScene scene;

    signals:
      void new_mouse_coordinates(QPointF);

    protected:
        bool _pan = false;
        int _panStartX, _panStartY;
        virtual void wheelEvent(QWheelEvent *event)
        {
            qreal factor;
            if (event->angleDelta().y() > 0)
                factor = 1.1;
            else
                factor = 0.9;
            auto view_pos = event->pos();
            auto scene_pos = this->mapToScene(view_pos);
            this->centerOn(scene_pos);
            this->scale(factor, factor);
            auto delta = this->mapToScene(view_pos) - this->mapToScene(this->viewport()->rect().center());
            this->centerOn(scene_pos - delta);
        }
        virtual void resizeEvent(QResizeEvent *e)
        {
            QGraphicsView::resizeEvent(e);
        }
        virtual void mouseMoveEvent(QMouseEvent *event)
        {
            if (_pan)
            {
                horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - _panStartX));
                verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - _panStartY));
                _panStartX = event->x();
                _panStartY = event->y();
                event->accept();

            }
            QGraphicsView::mouseMoveEvent(event);
        }
        virtual void mousePressEvent(QMouseEvent *event)
        {
            if (event->button() == Qt::LeftButton)
            {
                _pan = true;
                _panStartX = event->x();
                _panStartY = event->y();
                setCursor(Qt::ClosedHandCursor);
                event->accept();
                auto cursor_in_scene = this->mapToScene(QPoint(event->x(), event->y()));
                // std::cout << p.x() << "  " << p.y() << std::endl;
                emit new_mouse_coordinates(cursor_in_scene);
                return;
            }
            QGraphicsView::mousePressEvent(event);
        }
        virtual void mouseReleaseEvent(QMouseEvent *event)
        {
            if (event->button() == Qt::LeftButton)
            {
                _pan = false;
                setCursor(Qt::ArrowCursor);
                event->accept();
            }
            QGraphicsView::mouseReleaseEvent(event);
        }
};
#endif //GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H
