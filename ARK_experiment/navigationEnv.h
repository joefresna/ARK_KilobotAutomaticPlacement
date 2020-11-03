#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <kilobotenvironment.h>
#include <QTime>

static const double MM_TO_PIXEL = 2000.0/2000.0;
static const double PIXEL_TO_MM = 2000.0/2000.0;

enum command {
    STRAIGHT = 0,
    LEFT = 1,
    RIGHT = 2,
    STOP = 3,
    LIGHT = 4
};

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);

    void resetEnvironment();

    QVector <QPointF> goal_locations;

    QVector < command >  commandLog;
    QVector < bool >  arrived;

    bool sendLED;
    bool paused;

    float minTimeBetweenTwoMsg = 0;
    float time;
    QVector < float >  lastSent;
//    QVector < kilobot_id >  allKiloIDs;
    QVector < int > assignedTargets;
    QVector < double > LEDdelays;
    int replacementSensitivity = 25;
    int distThreshold = 10;

signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

};
#endif // MYKILOBOTENVIRONMENT_H
