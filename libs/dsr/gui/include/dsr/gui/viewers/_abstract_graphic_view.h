//
// Created by robolab on 12/06/20.
//

#ifndef GRAPH_RELATED_CLASSES_ABSTRACT_GRAPHIC_VIEW_H
#define GRAPH_RELATED_CLASSES_ABSTRACT_GRAPHIC_VIEW_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QGLWidget>
#include <QScrollBar>
#include <QApplication>


namespace DSR
{
	class AbstractGraphicViewer : public QGraphicsView
	{
		Q_OBJECT
		private:
			qreal m_scaleX, m_scaleY;

		public:
			AbstractGraphicViewer(QWidget* parent = 0);
			QGraphicsScene scene;

		protected:
			bool _pan = false;
			int _panStartX, _panStartY;
			virtual void wheelEvent(QWheelEvent* event);
			virtual void resizeEvent(QResizeEvent* e);
			virtual void mouseMoveEvent(QMouseEvent *event);
			virtual void mousePressEvent(QMouseEvent *event);
			virtual void mouseReleaseEvent(QMouseEvent *event);
			virtual void showEvent(QShowEvent *event);
	};
};

#endif //GRAPH_RELATED_CLASSES_ABSTRACT_GRAPHIC_VIEW_H
