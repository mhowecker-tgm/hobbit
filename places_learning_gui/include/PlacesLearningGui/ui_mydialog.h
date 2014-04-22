/********************************************************************************
** Form generated from reading UI file 'mydialog.ui'
**
** Created: Thu Apr 17 11:38:16 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MYDIALOG_H
#define UI_MYDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QRadioButton>
#include <QtGui/QVBoxLayout>

#include <iostream>

QT_BEGIN_NAMESPACE

class Ui_MyDialog
{


public:
    QDialogButtonBox *buttonBox;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QRadioButton *default_place;
    QRadioButton *searching;
    QRadioButton *goTo;
    QRadioButton *call_button;
    QRadioButton *floor_clearing;
    QRadioButton *fitness;

    void setupUi(QDialog *MyDialog)
    {
        if (MyDialog->objectName().isEmpty())
            MyDialog->setObjectName(QString::fromUtf8("MyDialog"));
        MyDialog->resize(400, 276);
        buttonBox = new QDialogButtonBox(MyDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(-60, 230, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        groupBox = new QGroupBox(MyDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(50, 20, 185, 198));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        default_place = new QRadioButton(groupBox);
        default_place->setObjectName(QString::fromUtf8("default_place"));

        verticalLayout->addWidget(default_place);

        searching = new QRadioButton(groupBox);
        searching->setObjectName(QString::fromUtf8("searching"));

        verticalLayout->addWidget(searching);

        goTo = new QRadioButton(groupBox);
        goTo->setObjectName(QString::fromUtf8("goTo"));

        verticalLayout->addWidget(goTo);

        call_button = new QRadioButton(groupBox);
        call_button->setObjectName(QString::fromUtf8("call_button"));

        verticalLayout->addWidget(call_button);

        floor_clearing = new QRadioButton(groupBox);
        floor_clearing->setObjectName(QString::fromUtf8("floor_clearing"));

        verticalLayout->addWidget(floor_clearing);

        fitness = new QRadioButton(groupBox);
        fitness->setObjectName(QString::fromUtf8("fitness"));

        verticalLayout->addWidget(fitness);


        retranslateUi(MyDialog);
        //QObject::connect(buttonBox, SIGNAL(accepted()), MyDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(accepted()), MyDialog, SLOT(on_accept_clicked()));

        QObject::connect(buttonBox, SIGNAL(rejected()), MyDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(MyDialog);
    } // setupUi

    void retranslateUi(QDialog *MyDialog)
    {
        MyDialog->setWindowTitle(QApplication::translate("MyDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MyDialog", "Select the type of place", 0, QApplication::UnicodeUTF8));
        default_place->setText(QApplication::translate("MyDialog", "default", 0, QApplication::UnicodeUTF8));
        searching->setText(QApplication::translate("MyDialog", "searching position", 0, QApplication::UnicodeUTF8));
        goTo->setText(QApplication::translate("MyDialog", "goTo place", 0, QApplication::UnicodeUTF8));
        call_button->setText(QApplication::translate("MyDialog", "callButton place", 0, QApplication::UnicodeUTF8));
        floor_clearing->setText(QApplication::translate("MyDialog", "floor clearing place", 0, QApplication::UnicodeUTF8));
        fitness->setText(QApplication::translate("MyDialog", "fitness place", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MyDialog: public Ui_MyDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MYDIALOG_H

