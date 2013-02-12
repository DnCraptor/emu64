//////////////////////////////////////////////////
//                                              //
// Emu64                                        //
// von Thorsten Kattanek                        //
//                                              //
// #file: button_mod.cpp                        //
//                                              //
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek           //
//                                              //
// Letzte �nderung am 12.02.2013                //
// www.emu64.de                                 //
//                                              //
//////////////////////////////////////////////////

#include "button_mod.h"

ButtonMod::ButtonMod(int idx,int idy, QWidget *parent) :
    QPushButton(parent)
{
    idX = idx;
    idY = idy;

    connect(this,SIGNAL(clicked()),this,SLOT(onClicked()));
}

void ButtonMod::onClicked()
{
    emit Clicked(idX,idY);
}
