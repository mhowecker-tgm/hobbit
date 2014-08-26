#ifndef MYDIALOG_H
#define MYDIALOG_H

#include "ui_mydialog.h"

#include "qnode.hpp"

namespace Ui {
class MyDialog;
}

class MyDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit MyDialog(QWidget *parent = 0);
    ~MyDialog();

    void on_OK();
    std::string current_place_name;
    std::string current_place_type;

    QNode* qnode;

    int n;
    
//private:
    Ui::MyDialog *ui;

private Q_SLOTS:
    void on_accept_clicked();


};

#endif // MYDIALOG_H
