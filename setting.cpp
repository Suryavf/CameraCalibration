#include "setting.h"
#include "ui_setting.h"

setting::setting(QWidget *parent) : QDialog(parent),
                                    ui(new Ui::setting){
    ui->setupUi(this);
    accepted = false;
}

setting::~setting(){
    delete ui;
}

void setting::on_pushButtonVideoBrowse_clicked(){
    QString pathTo;
    pathTo = QFileDialog::getOpenFileName(this, tr("Open video"), "",
                                                tr("Video (*.avi *.mp4);;All Files (*)"));
    dirVideoFile = pathTo;
    ui->txtVideoFile->setText(pathTo);
}

void setting::on_buttonBox_accepted(){
    accepted = true;

    squareSize = ui->txtSquareSize->selectedText().toInt();
    height     = ui->txtHeight    ->selectedText().toInt();
    width      = ui->txtWidth     ->selectedText().toInt();
}
