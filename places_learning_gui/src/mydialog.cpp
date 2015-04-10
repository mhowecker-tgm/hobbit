#include "../include/PlacesLearningGui/mydialog.h"
#include "../include/PlacesLearningGui/ui_mydialog.h"

#include <string>

#include <QInputDialog>

MyDialog::MyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MyDialog)
{
    ui->setupUi(this);
    
    n=1; //default, has to be updated for each dialog from the main window
}

MyDialog::~MyDialog()
{
    delete ui;
}

 void MyDialog::on_accept_clicked()
{

    if (ui->searching->isChecked()) current_place_type = "searchable";   
    if (ui->goTo->isChecked()) current_place_type = "goTo"; 
    if (ui->default_place->isChecked()) current_place_type = "default";
    if (ui->call_button->isChecked()) 
    {

	bool ok;
	QString text_input = QInputDialog::getText(this,("Enter button id number"),(""),QLineEdit::Normal,QString::null,&ok);
    	std::string utf8_button_number = text_input.toUtf8().constData();

	current_place_type = "call_button_";
	current_place_type+= utf8_button_number;
        //std::ostringstream oss;
        //oss<<n;
	//current_place_type += oss.str();
    }
    if (ui->floor_clearing->isChecked()) current_place_type = "floor_clearing";
    if (ui->fitness->isChecked()) current_place_type = "fitness";

    std::cout << "Place name " << current_place_name << std::endl;
    std::cout << "Place type " << current_place_type << std::endl;

    qnode->learnPlace(current_place_name, current_place_type);

    accept();
}
