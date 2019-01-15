#include "setting.h"
#include "ui_setting.h"

setting::setting(QWidget *parent) : QDialog(parent),
                                    ui(new Ui::setting){
    ui->setupUi(this);
}

setting::~setting(){
    delete ui;
}

void setting::on_pushButtonVideoBrowse_clicked(){
    QString pathTo;
    pathTo = QFileDialog::getOpenFileName(this, tr("Open video"), "",
                                                tr("Video (*.avi *.mp4);;All Files (*)"));


}
