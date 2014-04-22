#include "../include/PlacesLearningGui/mydialog.h"
#include "../include/PlacesLearningGui/ui_mydialog.h"

MyDialog::MyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MyDialog)
{
    ui->setupUi(this);
}

MyDialog::~MyDialog()
{
    delete ui;
}

 void MyDialog::on_accept_clicked()
{

    if (ui->searching->isChecked()) current_place_type = "searching";   
    if (ui->goTo->isChecked()) current_place_type = "goTo"; 
    if (ui->default_place->isChecked()) current_place_type = "default";
    if (ui->call_button->isChecked()) current_place_type = "call_button";
    if (ui->floor_clearing->isChecked()) current_place_type = "floor_clearing";
    if (ui->fitness->isChecked()) current_place_type = "fitness";

    std::cout << "Place name " << current_place_name << std::endl;
    std::cout << "Place type " << current_place_type << std::endl;

    qnode->learnPlace(current_place_name, current_place_type);

    accept();
}
