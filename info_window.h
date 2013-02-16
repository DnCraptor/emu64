//////////////////////////////////////////////////
//                                              //
// Emu64                                        //
// von Thorsten Kattanek                        //
//                                              //
// #file: info_window.h                         //
//                                              //
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek           //
//                                              //
// Letzte �nderung am 16.02.2013                //
// www.emu64.de                                 //
//                                              //
//////////////////////////////////////////////////

#ifndef INFO_WINDOW_H
#define INFO_WINDOW_H

#include <QDialog>

namespace Ui {
    class InfoWindow;
}

class InfoWindow : public QDialog
{
    Q_OBJECT

public:
    explicit InfoWindow(QWidget *parent = 0);
    ~InfoWindow();
    void RetranslateUi();
    void SetMoreInfoText(QString str);
    void SetEmu64VersionText(QString str);

private:
    Ui::InfoWindow *ui;
};

#endif // INFO_WINDOW_H
