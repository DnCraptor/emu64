//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: tree_widget_mod.h                     //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 13.05.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#ifndef TREE_WIDGET_MOD_H
#define TREE_WIDGET_MOD_H

#include <QTreeWidget>
#include <QItemDelegate>
class TreeWidgetMod : public QTreeWidget
{
    Q_OBJECT
public:
    explicit TreeWidgetMod(QWidget *parent = 0);
protected:
    void drawRow(QPainter *painter, const QStyleOptionViewItem &options, const QModelIndex &index) const;
signals:

public slots:

};

#endif // TREE_WIDGET_MOD_H
