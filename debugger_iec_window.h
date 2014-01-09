//////////////////////////////////////////////////
//                                              //
// Emu64                                        //
// von Thorsten Kattanek                        //
//                                              //
// #file: debugger_iec_window.h                 //
//                                              //
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek           //
//                                              //
// Letzte �nderung am 09.01.2014                //
// www.emu64.de                                 //
//                                              //
//////////////////////////////////////////////////

#ifndef DEBUGGER_IEC_WINDOW_H
#define DEBUGGER_IEC_WINDOW_H

#include <QDialog>
#include <QIcon>

#include "c64_class.h"

namespace Ui {
class DebuggerIECWindow;
}

class DebuggerIECWindow : public QDialog
{
    Q_OBJECT
    
public:
    explicit DebuggerIECWindow(QWidget *parent = 0);
    ~DebuggerIECWindow();
    void RetranslateUi();
    void SetC64Pointer(C64Class *c64);
    void UpdateSignals();
    
private:
    Ui::DebuggerIECWindow *ui;

    C64Class *c64;

    QIcon *iOff;
    QIcon *iOn;
};

#endif // DEBUGGER_IEC_WINDOW_H
