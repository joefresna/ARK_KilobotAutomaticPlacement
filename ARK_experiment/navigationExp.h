#ifndef TESTLIB_H
#define TESTLIB_H

// Project includes
#include "global.h"
#include "kilobot.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"
#include "navigationEnv.h"

#include <QObject>
#include <QFile>
#include <QTableWidget>
#include <QSpinBox>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

enum ExperimentState{
    COMPOSE,
    TRIGGER_LED,
    PAUSE,
    COMPLETED
};

class KiloLog {
public:
    // constructors
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF pos, double rot, kilobot_colour col) : id(id), position(pos), orientation(rot), colour(col) {}

    // methods
    void updateAllValues(kilobot_id id, QPointF pos, double rot, kilobot_colour col){
        this->id = id;
        this->position = pos;
        this->orientation = rot;
        this->colour = col;
    }
    void setPos(QPointF pos){
        this->position = pos;
    }
    void setOrientation(double rot){
        this->orientation = rot;
    }
    void setCol(kilobot_colour col){
        this->colour = col;
    }

    // variables
    kilobot_id id;
    QPointF position;
    double orientation;
    kilobot_colour colour;
};

class DIRECTNAVIGATIONEXPSHARED_EXPORT mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT
public:
    mykilobotexperiment();
    virtual ~mykilobotexperiment() {}

    QWidget * createGUI();

    //QVector < Kilobot * > mykilobot;
    void GetExperimentSettings();

signals:
    void errorMessage(QString);

public slots:
    void initialise(bool);
    void run();
    void stopExperiment();

//    void setupExperiment();
    void setupEnvironment(int currentshape);
    inline void setXShift(int val) { xShift = val; }
    inline void setYShift(int val) { yShift = val; }
    inline void toggleSaveImages(bool toggle) { saveImages = toggle; }
    inline void toggleLogExp(bool toggle) { logExp = toggle; }

    void addshape();
    void deleteshape();

private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobotCopy);

    //
    void assignTargetsToKilobots();
    void setupAllArrivedEnvironment();
    void setupEnvironmentHardCodedCoordinates();
    void plotEnvironment(mykilobotenvironment * myenvironcopy);

    //
    mykilobotenvironment myenviron;

    int xShift = 250;
    int yShift = 250;


    // shapes variables
    std::vector <QString> coordinatesFiles;
    std::vector <unsigned int> pauses;
    std::vector <std::pair<int,int>> shifts;
    int currentshape=-1;
    bool compoflag=false;
    float reassignEverySec = 5;


    bool allArrived;
    int numAssignedTargets = 0;

    ExperimentState currentState = COMPOSE;
    int counterPause;

    // Targets variables
    QVector < kilobot_id >  allKiloIDs;
    QVector <KiloLog> allKilos;

    // log variables
    bool saveImages;
    int savedImagesCounter = 0;
    bool logExp;
    QFile log_file;
    QString log_filename_prefix="log_navigation";
    QTextStream log_stream;

    // GUI objects
    QSpinBox * distanceThresh_spin;
    QSpinBox * replacementThresh_spin;
    QTableWidget * tableofshapes;


};



#endif // TESTLIB_H
