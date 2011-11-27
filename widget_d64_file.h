//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: widget_d64_file.h                     //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 13.10.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#ifndef WIDGET_D64_FILE_H
#define WIDGET_D64_FILE_H

#include <QtGui>
#include <QtCore>

namespace Ui {
    class WidgetD64File;
}

class WidgetD64File : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetD64File(QWidget *parent = 0);
    ~WidgetD64File();
    void SetLabels(QString filename, QString spur, QString sektor, QString adresse, QString size, QString typ);
    void SetRMode(int mode);

private:
    Ui::WidgetD64File *ui;
    QIcon *green_led;
    QIcon *yellow_led;
    QIcon *red_led;
};

#endif // WIDGET_D64_FILE_H
