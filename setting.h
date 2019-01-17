#ifndef SETTING_H
#define SETTING_H

#include <iostream>

#include <QDialog>
#include <QFileDialog>

namespace Ui {
class setting;
}

class setting : public QDialog
{
    Q_OBJECT

public:
    explicit setting(QWidget *parent = nullptr);
    ~setting();

    bool isAccepted(){return accepted;}
    void getValues(QString &_dirVideoFile,
                   QString &_typeGrid    ,
                   int     &_squareSize  ,
                   int     &_width       ,
                   int     &_height      ){

       _dirVideoFile = dirVideoFile;
       _squareSize   =   squareSize;
       _typeGrid     =     typeGrid;
       _height       =       height;
       _width        =        width;}

private slots:
    void on_pushButtonVideoBrowse_clicked();
    void on_buttonBox_accepted();

private:
    Ui::setting *ui;

    QString   dirVideoFile;
    QString dirSettingFile;
    QString       typeGrid;

    int   squareSize;
    int width,height;

    bool accepted;
};

#endif // SETTING_H
