#include "clicksignalqlabel.h"
#include <QMouseEvent>

clickSignalQLabel::clickSignalQLabel(QWidget *parent)  : QLabel(parent)
{
}

void clickSignalQLabel::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton) {
        emit leftclicked(QPoint(ev->localPos().x(),ev->localPos().y()));
        ev->accept();
    }
    else{
        if(ev->button() == Qt::RightButton){
            emit rightclicked(QPoint(ev->localPos().x(),ev->localPos().y()));
            ev->accept();
        }
        else{
            if(ev->button() == Qt::MiddleButton){
                emit middleclicked(QPoint(ev->localPos().x(),ev->localPos().y()));
                ev->accept();
            }
        }
    }
}
