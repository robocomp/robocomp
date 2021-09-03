/********************************************************************************
** Form generated from reading UI file 'graphUI.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPHUI_H
#define UI_GRAPHUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_graphDlg
{
public:
    QVBoxLayout *verticalLayout;
    QSplitter *splitter;
    QTreeWidget *treeWidget;
    QTabWidget *tabWidget;
    QWidget *tab_1;
    QHBoxLayout *horizontalLayout;
    QGraphicsView *graphicsView;
    QWidget *tab_2;
    QHBoxLayout *horizontalLayout_2;
    QWidget *tab_3;
    QHBoxLayout *horizontalLayout_4;
    QGraphicsView *graphicsView_2D;

    void setupUi(QWidget *graphDlg)
    {
        if (graphDlg->objectName().isEmpty())
            graphDlg->setObjectName(QStringLiteral("graphDlg"));
        graphDlg->resize(1055, 712);
        verticalLayout = new QVBoxLayout(graphDlg);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        splitter = new QSplitter(graphDlg);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        treeWidget = new QTreeWidget(splitter);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QStringLiteral("1"));
        treeWidget->setHeaderItem(__qtreewidgetitem);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(treeWidget->sizePolicy().hasHeightForWidth());
        treeWidget->setSizePolicy(sizePolicy);
        splitter->addWidget(treeWidget);
        tabWidget = new QTabWidget(splitter);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab_1 = new QWidget();
        tab_1->setObjectName(QStringLiteral("tab_1"));
        horizontalLayout = new QHBoxLayout(tab_1);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        graphicsView = new QGraphicsView(tab_1);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));

        horizontalLayout->addWidget(graphicsView);

        tabWidget->addTab(tab_1, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        horizontalLayout_2 = new QHBoxLayout(tab_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        horizontalLayout_4 = new QHBoxLayout(tab_3);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        graphicsView_2D = new QGraphicsView(tab_3);
        graphicsView_2D->setObjectName(QStringLiteral("graphicsView_2D"));

        horizontalLayout_4->addWidget(graphicsView_2D);

        tabWidget->addTab(tab_3, QString());
        splitter->addWidget(tabWidget);

        verticalLayout->addWidget(splitter);


        retranslateUi(graphDlg);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(graphDlg);
    } // setupUi

    void retranslateUi(QWidget *graphDlg)
    {
        graphDlg->setWindowTitle(QApplication::translate("graphDlg", "DSRGraph", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_1), QApplication::translate("graphDlg", "graph", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("graphDlg", "3d", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("graphDlg", "2d", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class graphDlg: public Ui_graphDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPHUI_H
