//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: info_window.h                         //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 18.04.2011		//
// www.emu64.de					//
//						//
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

private:
    Ui::InfoWindow *ui;
};

#endif // INFO_WINDOW_H
