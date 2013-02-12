//////////////////////////////////////////////////
//                                              //
// Emu64                                        //
// von Thorsten Kattanek                        //
//                                              //
// #file: button_mod.h                          //
//                                              //
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek           //
//                                              //
// Letzte �nderung am 12.02.2013                //
// www.emu64.de                                 //
//                                              //
//////////////////////////////////////////////////

#ifndef BUTTON_MOD_H
#define BUTTON_MOD_H

#include <QPushButton>

class ButtonMod : public QPushButton
{
    Q_OBJECT
public:
    explicit ButtonMod(int idx,int idy, QWidget *parent = 0);
    
signals:
    void Clicked(int idx, int idy);
    
public slots:
    void onClicked();

private:
    int idX;
    int idY;
    
};

#endif // BUTTON_MOD_H
