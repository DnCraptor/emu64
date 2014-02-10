//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: label_widget_mod.cpp                  //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 28.08.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#include "label_widget_mod.h"

LabelWidgetMod::LabelWidgetMod(QWidget *parent) :
    QLabel(parent),
    pressed(false)
{

}

void LabelWidgetMod::mousePressEvent(QMouseEvent*)
{
    pressed = true;
}

void LabelWidgetMod::mouseReleaseEvent(QMouseEvent *event)
{
    if(pressed && rect().contains(event->pos()))
    {
        emit clicked(this);
    }
}
