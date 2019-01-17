#include "setting.h"
#include "ui_setting.h"

setting::setting(QWidget *parent) : QDialog(parent),
                                    ui(new Ui::setting){
    ui->setupUi(this);
    accepted = false;

    //ui->txtHeight ->setText("3");
    //ui->txtWidth  ->setText("4");
}

setting::~setting(){
    delete ui;
}

void setting::on_pushButtonVideoBrowse_clicked(){
    QString pathTo;

    pathTo = QFileDialog::getOpenFileName(this, tr("Open video"), "/home/victor/Vídeos/data",
                                                tr("Video (*.avi *.mp4);;All Files (*)"));

    dirVideoFile = pathTo;
    ui->txtVideoFile->setText(pathTo);
}

void setting::on_buttonBox_accepted(){
    accepted = true;

    squareSize = ui->txtSquareSize->text().toInt();
    typeGrid   = ui->comboBoxTypeGrid->currentText();
    height     = ui->txtHeight    ->text().toInt();
    width      = ui->txtWidth     ->text().toInt();
}
