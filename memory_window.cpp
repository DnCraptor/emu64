//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: memory_window.cpp                     //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 28.08.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#include "memory_window.h"
#include "ui_memory_window.h"

MemoryWindow::MemoryWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MemoryWindow),
    AktViewAdresse(0),
    NoFocusRun(true)
{
    ui->setupUi(this);
    ui->MemoryTable->setColumnCount(1);
    ui->MemoryTable->setRowCount(MemZeilenAnz);

    MemScrDest << "RAM" << "KERNAL" << "BASIC" << "VIC" << "FARBRAM" << "SID" << "CIA1" << "CIA2" << "IO1" << "IO2" << "CHARROM" << "ROM-LO" << "ROM-HI" << "ROM-HI" << "ADR.OPEN";

    for(int i=0;i<MemZeilenAnz;i++)
    {
        WidgetMemoryZeile *w = new WidgetMemoryZeile(this);
        ui->MemoryTable->setCellWidget(i,0,w);
        ui->MemoryTable->setRowHeight(i,w->height());
        ui->MemoryTable->setColumnWidth(0,w->width());
        connect(w,SIGNAL(ChangeValue(unsigned short,unsigned char)),this,SLOT(onChangeValue(unsigned short,unsigned char)));
        connect(this,SIGNAL(NoFocus()),w,SLOT(onNoFocus()));
    }

    WidgetMemoryZeile *w = (WidgetMemoryZeile*)ui->MemoryTable->cellWidget(0,0);
    w->setEnabled(false);
}

MemoryWindow::~MemoryWindow()
{
    delete ui;
}

void MemoryWindow::SetC64Pointer(C64Class *_c64)
{
    c64 = _c64;
}

void MemoryWindow::ChangeSource(int source)
{
    AktSource = source;
    if(AktSource > 0) AktFloppyNr = AktSource - 1;

    switch(AktSource)
    {
    case 0:
        setWindowTitle(QString(tr("C64 Speicher")));
        break;
    case 1:
        setWindowTitle(QString(tr("Floppy #08 Speicher")));
        break;
    case 2:
        setWindowTitle(QString(tr("Floppy #09Speicher")));
        break;
    case 3:
        setWindowTitle(QString(tr("Floppy #10 Speicher")));
        break;
    case 4:
        setWindowTitle(QString(tr("Floppy #11 Speicher")));
        break;
    }
}

void MemoryWindow::onChangeValue(unsigned short adresse, unsigned char wert)
{
    if(AktSource > 0)
    {
        c64->floppy[AktFloppyNr]->WriteByte(adresse,wert);
    }
    else
    {
        c64->WriteC64Byte(adresse,wert);
    }
}

void MemoryWindow::UpdateMemoryList(void)
{
    if(c64 == 0) return;
    if(isHidden()) return;

    WidgetMemoryZeile *w;
    unsigned char puffer[16];

    for(int i=0;i<MemZeilenAnz-1;i++)
    {
        if(AktSource > 0)
        {
            for(int x=0;x<16;x++) puffer[x] = c64->floppy[AktFloppyNr]->ReadByte(AktViewAdresse + (i*16) + x);
        }
        else
        {
            for(int x=0;x<16;x++) puffer[x] = c64->ReadC64Byte(AktViewAdresse + (i*16) + x);
        }
        w = (WidgetMemoryZeile*)ui->MemoryTable->cellWidget(i+1,0);

        unsigned char page = (AktViewAdresse + (i*16)) >> 8;
        unsigned char mem_read_source = c64->GetMapReadSource(page);
        unsigned char mem_write_destinatio = c64->GetMapWriteDestination(page);

        w->Fill(AktViewAdresse + (i*16),puffer,MemScrDest[mem_read_source],MemScrDest[mem_write_destinatio]);
    }
}

void MemoryWindow::on_MemoryScroll_valueChanged(int value)
{
    unsigned short v = value / 16;
    AktViewAdresse = v * 16;
    if(NoFocusRun) emit NoFocus();
    UpdateMemoryList();
}

void MemoryWindow::on_MemoryScroll_sliderPressed()
{
    emit NoFocus();
    NoFocusRun = false;
}

void MemoryWindow::on_MemoryScroll_sliderReleased()
{
    NoFocusRun = true;
}

void MemoryWindow::on_BitAnzeige_clicked(bool checked)
{
    WidgetMemoryZeile *w;
    for(int i=0;i<MemZeilenAnz-1;i++)
    {
        w = (WidgetMemoryZeile*)ui->MemoryTable->cellWidget(i+1,0);
        w->EndableBitLeiste(checked);
    }
}
