/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Tue Oct 15 10:53:40 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

#include "myqlabel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton;
    QPushButton *connect;
    MyQLabel *label;
    QPushButton *load_map;
    QPushButton *save_place;
    QPushButton *save_file;
    QPushButton *delete_place;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(719, 480);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(570, 360, 98, 27));

        label = new MyQLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(70, 10, 400, 400));
        label->setFrameShape(QFrame::NoFrame);

	connect = new QPushButton(centralWidget);
        connect->setObjectName(QString::fromUtf8("connect"));
        //connect->setGeometry(QRect(570, 180, 98, 27));
	connect->setGeometry(QRect(570, 20, 98, 27));
	//connect->setStyleSheet("background-color:blue;");

        load_map = new QPushButton(centralWidget);
        load_map->setObjectName(QString::fromUtf8("load_map"));
        //load_map->setGeometry(QRect(570, 20, 98, 27));
	load_map->setGeometry(QRect(570, 70, 98, 27));

        save_place = new QPushButton(centralWidget);
        save_place->setObjectName(QString::fromUtf8("save_place"));
        //save_place->setGeometry(QRect(570, 70, 98, 27));
	save_place->setGeometry(QRect(570, 120, 98, 27));
	//save_place->setStyleSheet("background-color:beige;");

	delete_place = new QPushButton(centralWidget);
        delete_place->setObjectName(QString::fromUtf8("delete_place"));
        //delete_place->setGeometry(QRect(570, 120, 98, 27));
	delete_place->setGeometry(QRect(570, 170, 98, 27));
	
        save_file = new QPushButton(centralWidget);
        save_file->setObjectName(QString::fromUtf8("save_file"));
        save_file->setGeometry(QRect(570, 310, 98, 27));

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 719, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);
        QObject::connect(pushButton, SIGNAL(clicked()), MainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "PlacesLearningGui", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        load_map->setText(QApplication::translate("MainWindow", "Load map", 0, QApplication::UnicodeUTF8));
        save_place->setText(QApplication::translate("MainWindow", "Save place", 0, QApplication::UnicodeUTF8));
        save_file->setText(QApplication::translate("MainWindow", "Save file", 0, QApplication::UnicodeUTF8));
        delete_place->setText(QApplication::translate("MainWindow", "Delete place", 0, QApplication::UnicodeUTF8));
        connect->setText(QApplication::translate("MainWindow", "Load rooms", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
