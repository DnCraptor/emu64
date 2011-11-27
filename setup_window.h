//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: setup_window.h                        //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 11.10.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#ifndef SETUP_WINDOW_H
#define SETUP_WINDOW_H

#include <QDialog>
#include <QSettings>
#include "videopal_class.h"
#include "c64_class.h"

namespace Ui {
    class SetupWindow;
}

class SetupWindow : public QDialog
{
    Q_OBJECT

public:
    explicit SetupWindow(QWidget *parent = 0,const char *member = 0,VideoPalClass *_videopal = 0, QSettings *_ini = 0);
    ~SetupWindow();
    void RetranslateUi();
    void LoadINI(C64Class *_c64);

signals:
    void ChangeGrafikModi(bool fullscreen,bool palmode, bool doublemode, bool bit32mode);

private slots:
    void on_pushButton_clicked();
    void on_C64Farbmodus_currentIndexChanged(int index);
    void on_WPal_toggled(bool checked);
    void on_WDouble_toggled(bool checked);
    void on_W16Bit_toggled(bool checked);
    void on_W32Bit_toggled(bool checked);

    void on_JoyScan_clicked();

    void on_GamePort1_currentIndexChanged(int index);

    void on_GamePort2_currentIndexChanged(int index);

private:
    Ui::SetupWindow *ui;
    VideoPalClass* videopal;
    QSettings *ini;
    C64Class *c64;
};

#endif // SETUP_WINDOW_H
