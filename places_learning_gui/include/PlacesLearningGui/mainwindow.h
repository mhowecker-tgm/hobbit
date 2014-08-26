#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include <QMainWindow>
#include <QPaintEvent>

#include "myqlabel.h"

#include "qnode.hpp"

#include "ui_mainwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    //explicit MainWindow(QWidget *parent = 0);
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    
    QNode qnode;
    
    int num; //id for call buttons

private Q_SLOTS:
    
    void on_load_map_clicked();

    void on_save_place_clicked();

    int on_save_file_clicked();

    void on_delete_place_clicked();

    void on_connect_clicked();

private:

    Ui::MainWindow *ui;
    void LoadMap();
    bool LoadFile(std::string placesFileName);

    QString map_file_name;

    float input_map_height;
    float map_size_relation;
    float map_resolution;

    float map_origin_x;
    float map_origin_y;

    float transform_x(float x);
    float transform_y(float y);

    bool bConnected;


};

#endif // MAINWINDOW_H
