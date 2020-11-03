#ifndef CLICKSIGNALQLABEL_H
#define CLICKSIGNALQLABEL_H

#include <QLabel>

class clickSignalQLabel : public QLabel
{
    Q_OBJECT

public:
    clickSignalQLabel(QWidget *parent = 0);

protected slots:
    void mousePressEvent(QMouseEvent *ev);

signals:
    void leftclicked(QPoint);
    void middleclicked(QPoint);
    void rightclicked(QPoint);
};

#endif // CLICKSIGNALQLABEL_H
