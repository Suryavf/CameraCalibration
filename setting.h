#ifndef SETTING_H
#define SETTING_H

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

private slots:
    void on_pushButtonVideoBrowse_clicked();

private:
    Ui::setting *ui;

    QString   dirVideoFile;
    QString dirSettingFile;
    QString       typeGrid;

    int   squareSize;
    int width,height;
};

#endif // SETTING_H
