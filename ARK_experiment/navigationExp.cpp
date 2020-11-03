#include "navigationExp.h"
#include "navigationEnv.h"
#include <QDebug>
#include <QThread>
// widgets
#include <QPushButton>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QCheckBox>
#include <QtMath>
#include <QSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QSettings>#include <stdio.h>
#include <stdlib.h>
#include "hungarian.h"

int** array_to_matrix(int* m, int rows, int cols) {
  int i,j;
  int** r;
  r = (int**)calloc(rows,sizeof(int*));
  for(i=0;i<rows;i++)
  {
    r[i] = (int*)calloc(cols,sizeof(int));
    for(j=0;j<cols;j++)
      r[i][j] = m[i*cols+j];
  }
  return r;
}



// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" DIRECTNAVIGATIONEXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

mykilobotexperiment::mykilobotexperiment() {

    // setup the environments here
    connect(&myenviron,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));

    this->serviceInterval = 100; // timestep in ms?

}

QWidget * mykilobotexperiment::createGUI() {

    QFrame * frame = new QFrame;
    QVBoxLayout * lay = new QVBoxLayout;
    frame->setLayout(lay);

    // add a check box to record the experiment
    QCheckBox * saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(false);// make the box not checked by default
    lay->addWidget(saveImages_ckb);
    toggleSaveImages(saveImages_ckb->isChecked());

    // add a check box to log the experiment
    QCheckBox * logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true); // make the box checked by default
    lay->addWidget(logExp_ckb);
    toggleLogExp(logExp_ckb->isChecked());

    QGroupBox * formGroupThresh = new QGroupBox(tr("Thresholds:"));
    QFormLayout * layout2 = new QFormLayout;
    formGroupThresh->setLayout(layout2);
    distanceThresh_spin = new QSpinBox();
    distanceThresh_spin->setMinimum(5);
    distanceThresh_spin->setMaximum(1000);
    distanceThresh_spin->setValue(myenviron.distThreshold);
    layout2->addRow(new QLabel(tr("DistThresh:")), distanceThresh_spin);
    replacementThresh_spin = new QSpinBox();
    replacementThresh_spin->setMinimum(5);
    replacementThresh_spin->setMaximum(1000);
    replacementThresh_spin->setValue(myenviron.replacementSensitivity);
    layout2->addRow(new QLabel(tr("Replacement:")), replacementThresh_spin);
    lay->addWidget(formGroupThresh);

    QGroupBox * formGroupCoords = new QGroupBox(tr("Shapes files and parameters:"));
    QFormLayout * layoutCoord = new QFormLayout;
    formGroupCoords->setLayout(layoutCoord);

    tableofshapes= new QTableWidget();
    tableofshapes->setColumnCount(4);
    tableofshapes->setHorizontalHeaderLabels(QStringList() << tr("path") << "x-shift"<<"y-shift"<<tr("pause (sec)") );
    tableofshapes->setGeometry(QRect(5,5,500,500));
    tableofshapes->setColumnWidth(0,0.6*tableofshapes->width());
    tableofshapes->setColumnWidth(1,0.1*tableofshapes->width());
    tableofshapes->setColumnWidth(2,0.1*tableofshapes->width());
    tableofshapes->setColumnWidth(3,0.2*tableofshapes->width());

    tableofshapes->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    tableofshapes->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    QPushButton * add_shape = new QPushButton("Add");
    QPushButton * delete_shape = new QPushButton("Delete");
    layoutCoord->addRow(tableofshapes);
    layoutCoord->addRow(add_shape);
    layoutCoord->addRow(delete_shape);
    lay->addWidget(formGroupCoords);


    // signal-slot connections
    connect(add_shape,SIGNAL(clicked(bool)),this,SLOT(addshape()));
    connect(delete_shape,SIGNAL(clicked(bool)),this,SLOT(deleteshape()));
    connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
    connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));

    lay->addStretch();

    connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));

    return frame;
}

void mykilobotexperiment::initialise(bool isResume) {
    // Generate Environment:
    // setupEnvironment(coordinatesFileFirst);

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else {
        // probably nothing
    }

    emit setTrackingType(POS | ROT | LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    // Init Log File operations
    if (logExp){
        if (log_file.isOpen()){
            log_file.close(); // if it was open I close and re-open it (erasing old content!! )
        }
        QString log_filename = log_filename_prefix + ".txt";
        //        QString log_filename = log_filename_prefix + "_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file.setFileName( log_filename );
        if ( !log_file.open(QIODevice::WriteOnly) ) { // open file
            qDebug() << "ERROR(!) in opening file" << log_filename;
        } else {
            qDebug () << "Log file" << log_file.fileName() << "opened.";
            log_stream.setDevice(&log_file);
        }
    }

    savedImagesCounter = 0;
    this->time = 0;

    // Get experiment settings (shapes)
    currentshape=-1;
    compoflag=false;
    GetExperimentSettings();
    if(coordinatesFiles.size()>0){
        currentshape=0;
        qDebug() << "There is "<<coordinatesFiles.size()<<" shapes to form";
    }

    // clear old drawings (e.g., from ID-identification)
    clearDrawings();
}

void mykilobotexperiment::run() {

    // Update Environments:
    myenviron.time = (float)time;
    myenviron.update();
    myenviron.distThreshold = distanceThresh_spin->value();
    myenviron.replacementSensitivity = replacementThresh_spin->value();

    // Update Kilobot States:
    emit updateKilobotStates();

    switch (currentState) {
    case COMPOSE:{
        if(currentshape<0) return;
        if(!compoflag){
            setupEnvironment(currentshape);
            compoflag=true;
        }

        allArrived = true;

        /* This if statement has been added to account for when the number of kilobots and targets do not correspond
         * We are forcing the experiment to terminated by tricking the system into believing all kilobots have reached targets
        */
        if (allKiloIDs.size()== myenviron.goal_locations.size()){
            for (int i = 0; i < allKiloIDs.size(); ++i){
                kilobot_id kID = allKiloIDs[i];
                if (!myenviron.arrived[kID]){
                    allArrived = false;
                    break;
                }
            }
        }else{
            qDebug() << "System is terminating because number of Kilobots and targets do not correspond!";
        }

        if (allArrived){
            qDebug() << " * * * * ALL" << allKiloIDs.size() << "ROBOTS ARRIVED TO THEIR DESTINATION  * * * * ";
            allArrived = false;

            myenviron.sendLED = true;
            currentState = TRIGGER_LED;
            qDebug() << "Moving to TRIGGER_LED";
        }
        else {
            if (qRound(this->time*100.0) % qRound(this->reassignEverySec * 100.0) == 0){
                qDebug() << "Reassigment triggered!";
                assignTargetsToKilobots();
            }
        }

        break;
    }
    case TRIGGER_LED:{
        if(currentshape==coordinatesFiles.size()-1){
            currentState=COMPLETED;
            qDebug() << "Moving to COMPLETED";

        }
        else{
            currentState = PAUSE;
            counterPause = pauses[currentshape];
            myenviron.sendLED = false;
            myenviron.paused = true;
            qDebug() << "Moving to PAUSE";
        }
        break;
    }
    case PAUSE:{
        if (--counterPause == 0){
            myenviron.paused = false;
            //            myenviron.resetEnvironment();//
            //            emit getInitialKilobotStates();//
            currentState = COMPOSE;
            currentshape++;
            compoflag=false;
            for (int i = 0; i < allKiloIDs.size(); ++i){
                kilobot_id kID = allKiloIDs[i];
                myenviron.arrived[kID]=false;
                qDebug() << kID << "====>   [" << myenviron.arrived[kID]<<"]";
            }
            qDebug() << "Moving to COMPOSE the next shape";
        }
        break;
    }

    case COMPLETED:{
        if (allKiloIDs.size()== myenviron.goal_locations.size()){
        qDebug() << "Experiment completed!";
        }else{
            qDebug() << "Experiment stopped because number of Kilobots does not correspond with number of targets";
            if(myenviron.goal_locations.size() > allKiloIDs.size()){
                qDebug() << "Please add" << myenviron.goal_locations.size() - allKiloIDs.size() << "kilobots!";
            }
            else{
                qDebug() << "Please remove" << allKiloIDs.size() -myenviron.goal_locations.size() << "kilobots!";
            }
        }
        //        emit experimentComplete();
        break;
    }
    }

    if (qRound(time*10.0) % 5 == 0) { // every 0.5s
        if (saveImages) {
            emit saveImage(QString("navexp_%1.jpg").arg(savedImagesCounter++, 5,10, QChar('0')));
        }
        if (allKiloIDs.size()== myenviron.goal_locations.size()){
            if (logExp){
                log_stream << this->time;
                for (int i = 0; i < allKiloIDs.size(); ++i){
                    kilobot_id kID = allKiloIDs[i];
                    int targetIdx = myenviron.assignedTargets[kID];
                    log_stream << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y() << "\t" << allKilos[kID].orientation << "\t"
                               << allKilos[kID].colour << "\t" << (int)myenviron.arrived[kID] << "\t"
                               << myenviron.goal_locations[targetIdx].x() << "\t" << myenviron.goal_locations[targetIdx].y();
                }
                log_stream << endl;
            }
        }
    }

    clearDrawings();

    plotEnvironment(&myenviron);

    this->time += 0.1; // 100 ms in sec
}

void mykilobotexperiment::stopExperiment() {
    if (log_file.isOpen()){
        qDebug() << "Closing file" << log_file.fileName();
        log_file.close();
    }
    this->time = 0;
    numAssignedTargets = 0;
    currentState = COMPOSE;
    myenviron.resetEnvironment();
}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobotCopy) {

    // Assigns all kilobots to the environment:
    this->setCurrentKilobotEnvironment(&myenviron);

    kilobot_id kID = kilobotCopy.getID();

    // create a necessary lists and variables
    if (kID > myenviron.arrived.size()-1){
        myenviron.arrived.resize(kID+1);
        myenviron.commandLog.resize(kID+1);
        myenviron.lastSent.resize(kID+1);
        allKilos.resize(kID+1);
    }
    myenviron.arrived[kID] = false; // initialise all robots as not arrived
    myenviron.commandLog[kID] = STOP; // initialise all robots command log
    myenviron.lastSent[kID] = -1000;

    KiloLog kLog(kID, kilobotCopy.getPosition()*PIXEL_TO_MM, 0, kilobotCopy.getLedColour());
    allKilos[kID] = kLog;
    if (!allKiloIDs.contains(kID)) allKiloIDs.append(kID);
    //    myenviron.allKiloIDs = allKiloIDs;

    //myenviron.allKilos.append(kilobot.getID());
    allArrived = false;

    double timeForAMessage = 0.05;
    myenviron.minTimeBetweenTwoMsg = allKiloIDs.size()*timeForAMessage/2.0;
    qDebug() << "Min time between two messages is" << myenviron.minTimeBetweenTwoMsg;

}

void mykilobotexperiment::setupEnvironment(int currentshape) {

    QString coordinatesFile=coordinatesFiles[currentshape];
    xShift=shifts[currentshape].first;
    yShift=shifts[currentshape].second;

    myenviron.goal_locations.clear(); // clear goal location list

    QFile coordsFile(coordinatesFile);
    if ( !coordsFile.open(QIODevice::ReadOnly) ) { // open file
        qDebug() << "ERROR(!) in opening file" << coordinatesFile;
    } else {
        qDebug () << "Coords file" << coordsFile.fileName() << "opened.";
    }
    QTextStream stream(&coordsFile);

    QString line;
    line = stream.readLine();

    while (!stream.atEnd()){
        line = stream.readLine();
        QStringList coords = line.split("\t");
        if (coords.size() == 2){
            myenviron.goal_locations.push_back(QPointF(coords[0].toDouble(), coords[1].toDouble()) + QPointF(xShift,yShift) );
        } else {
            qDebug() << "This line has not 2 items:" << line;
        }
    }

    qDebug() << "Goal location list size is" << myenviron.goal_locations.size();

    assignTargetsToKilobots();

    plotEnvironment(&myenviron);

}

void mykilobotexperiment::setupAllArrivedEnvironment() {
    myenviron.goal_locations.clear(); // clear goal location list

    for (int k=0; k < allKiloIDs.size(); ++k){
        kilobot_id kID = allKiloIDs[k];
        myenviron.goal_locations.push_back(allKilos[kID].position);
    }

    qDebug() << "Goal location list size is" << myenviron.goal_locations.size();

    assignTargetsToKilobots();

    plotEnvironment(&myenviron);

}

void mykilobotexperiment::assignTargetsToKilobots() {
    qDebug() << allKiloIDs.size() << "This is allKiloIDs.size";
    if (allKiloIDs.size()!= myenviron.goal_locations.size()){
        // If negative remove Kilobots
        qDebug() << "Number of targets and number of Kilobots do not correspond. Please add " << myenviron.goal_locations.size() -allKiloIDs.size()<< "Kilobots";
        currentState=COMPLETED;
        return;
    }else{
        if (allKiloIDs.empty()){
            qDebug() << "Kilobot list is empty. I can't assign targets to kilobots.";
            return;
        } else {
            qDebug() << "Assigning" << myenviron.goal_locations.size() << "targets to" << allKiloIDs.size() << "Kilobots.";
        }

        int number_of_targets =  myenviron.goal_locations.size();
        int size_of_array = number_of_targets*number_of_targets;
        int r[size_of_array]={};
        int counter=0;
        // Initialising cost array which will be later converted into a matrix
        for(int ii=0;ii < size_of_array; ++ii){
            r[ii] = QLineF(allKilos[counter].position, myenviron.goal_locations[ii % number_of_targets]).length();
            if(ii % number_of_targets == 0 && ii!=0)
                counter++;
        }
        /* ***************************************************************************
         *      From here main of Hungarian
         ******************************************************************************/
        hungarian_problem_t p;

         int** m = array_to_matrix(r,number_of_targets,number_of_targets);

         // initialize the gungarian_problem using the cost matrix
         int matrix_size = hungarian_init(&p, m , number_of_targets,number_of_targets, HUNGARIAN_MODE_MINIMIZE_COST) ;

         fprintf(stderr, "assignement matrix has a now a size %d rows and %d columns.\n\n",  matrix_size,matrix_size);

         // some output
         //fprintf(stderr, "cost-matrix:");
         //hungarian_print_costmatrix(&p);

         // solve the assignement problem
         hungarian_solve(&p);

         // some output
         //fprintf(stderr, "assignment:");
         //hungarian_print_assignment(&p);


         // free used memory
         /*hungarian_free(&p);

         int idx;
         for (idx=0; idx < 4; idx+=1) {
           free(m[idx]);
         }
         free(m);*/
         /* *******************************************************************************************************
          *     End of hungarian_free
          * *********************************************************************************************************/

        QVector < bool > assignedKilo;
        assignedKilo.resize( allKilos.size() );
        assignedKilo.fill(false);
        myenviron.assignedTargets.clear();
        myenviron.assignedTargets.resize( allKilos.size() );
        myenviron.assignedTargets.fill(-1);
        /*for (int t=0; t < myenviron.goal_locations.size(); ++t){
            int kilo_found = -1;
            double minDist = std::numeric_limits<double>::max();
            for (int k=0; k < allKiloIDs.size(); ++k){
                kilobot_id kID = allKiloIDs[k];
                if (assignedKilo[kID]) continue;
                double dist = QLineF(allKilos[kID].position, myenviron.goal_locations[t]).length();
                if (dist < minDist) {
                    kilo_found = kID;
                    minDist = dist;
                    //qDebug() << kID << dist;
                }

            }
            if (kilo_found >= 0) {
                myenviron.assignedTargets[kilo_found] = t;
                assignedKilo[kilo_found] = true;
                qDebug() << t << allKilos[t].id << myenviron.assignedTargets[t];
            } else {
                //qDebug() << "We have a problem!! Target with index" << t << "couldn't be assigned to any kilobot!";
            }
        }*/
        for(int ii =0;ii< number_of_targets;ii++){
            for(int jj=0;jj<number_of_targets;jj++){
                if(p.assignment[ii][jj]==1){
                    myenviron.assignedTargets[ii]=jj;
                }
            }
        }


        myenviron.LEDdelays.clear();
        myenviron.LEDdelays.resize(allKilos.size());
        double minDist = std::numeric_limits<double>::max();
        double maxDist = 0;
        // find all distances from a right corner
        for (int k=0; k < allKiloIDs.size(); ++k){
            kilobot_id kID = allKiloIDs[k];
            double dist = QLineF(myenviron.goal_locations[myenviron.assignedTargets[kID]], QPointF(0,0)).length();
            if (dist < minDist) minDist=dist;
            if (dist > maxDist) maxDist=dist;
            myenviron.LEDdelays[kID] = dist;
        }
        // normalise the distances
        for (int k=0; k < allKiloIDs.size(); ++k){
            kilobot_id kID = allKiloIDs[k];
            myenviron.LEDdelays[kID] /= (maxDist-minDist);
        }
    }
}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {
    // update values for logging
    if (logExp){
        kilobot_id kID = kilobotCopy.getID();
        kilobot_colour kCol = kilobotCopy.getLedColour();
        QPointF kPos = kilobotCopy.getPosition()*PIXEL_TO_MM;
        double kRot = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));
        allKilos[kID].updateAllValues(kID, kPos, kRot, kCol);
    }

    //    if (myenviron.counterLED > 0){
    //        if (--myenviron.counterLED == 1){
    //            int targetIdx = allKiloIDs.indexOf(kilobotCopy.getID());
    //            QPointF prevLoc = myenviron.goal_locations[targetIdx];
    //            myenviron.goal_locations[targetIdx].setX( prevLoc.y() );
    //            myenviron.goal_locations[targetIdx].setY( prevLoc.x() );
    //            myenviron.arrived[targetIdx] = false;
    //        }
    //    }
}
// Setup Environment:
void mykilobotexperiment::setupEnvironmentHardCodedCoordinates( ) {

    //coordinatesFile = QFileDialog::getOpenFileName((QWidget *) sender(), tr("Load Coordinates"), QDir::home().absolutePath(), tr("Coord files (*.txt *.csv);; All files (*)"));

    myenviron.goal_locations.clear(); // clear goal location list

    // For now, do this:
    //    QPointF goal_loc1,goal_loc2;
    //    goal_loc1.setX(500); // define goal 1
    //    goal_loc1.setY(1500); // define goal 1
    //    goal_loc2.setX(1500); // define goal 2
    //    goal_loc2.setY(1500); // define goal 2
    //    myenviron.goal_locations.push_back(goal_loc1);
    //    for (int i=1; i < 10; ++i){
    //        myenviron.goal_locations.push_back(goal_loc2);
    //    }
    /*    QPointF goal_loc;
    goal_loc.setX(525);
    goal_loc.setY(1300);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(550);
    goal_loc.setY(1240);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(600);
    goal_loc.setY(1200);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(650);
    goal_loc.setY(1250);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(700);
    goal_loc.setY(1200);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(750);
    goal_loc.setY(1240);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(775);
    goal_loc.setY(1300);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(750);
    goal_loc.setY(1350);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(700);
    goal_loc.setY(1400);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(650);
    goal_loc.setY(1450);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(600);
    goal_loc.setY(1400);
    myenviron.goal_locations.push_back(goal_loc);
    goal_loc.setX(550);
    goal_loc.setY(1350);
    myenviron.goal_locations.push_back(goal_loc);
    */


    /*************************************************/
    /****************    HEART SHAPE    **************/
    /*************************************************/
    //    QPointF goal_loc;
    //    goal_loc.setX(525);
    //    goal_loc.setY(1400);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(550);
    //    goal_loc.setY(1330);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(590);
    //    goal_loc.setY(1270);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(630);
    //    goal_loc.setY(1220);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(690);
    //    goal_loc.setY(1210);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(750);
    //    goal_loc.setY(1250);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(800);
    //    goal_loc.setY(1300);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(850);
    //    goal_loc.setY(1250);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(910);
    //    goal_loc.setY(1210);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(970);
    //    goal_loc.setY(1220);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(1010);
    //    goal_loc.setY(1270);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(1050);
    //    goal_loc.setY(1330);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(1075);
    //    goal_loc.setY(1400);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(1050);
    //    goal_loc.setY(1450);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(1000);
    //    goal_loc.setY(1500);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(950);
    //    goal_loc.setY(1550);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(900);
    //    goal_loc.setY(1600);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(850);
    //    goal_loc.setY(1650);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(800);
    //    goal_loc.setY(1700);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(750);
    //    goal_loc.setY(1650);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(700);
    //    goal_loc.setY(1600);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(650);
    //    goal_loc.setY(1550);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(600);
    //    goal_loc.setY(1500);
    //    myenviron.goal_locations.push_back(goal_loc);
    //    goal_loc.setX(550);
    //    goal_loc.setY(1450);
    //    myenviron.goal_locations.push_back(goal_loc);

    QPointF goal_loc;
    for (int i = 0; i < 5; ++i){
        goal_loc.setX(800 + (i*100));
        for (int j = 0; j < 10; ++j){
            goal_loc.setY(700 + (j*100));
            myenviron.goal_locations.push_back(goal_loc);
        }
    }

    qDebug() << "Goal location list size is" << myenviron.goal_locations.size();

    plotEnvironment(&myenviron);
}
// Plot Environment on frame:
void mykilobotexperiment::plotEnvironment(mykilobotenvironment *myenvironcopy) {

    // Draw a circle for each goal
    for (int i = 0; i < myenvironcopy->goal_locations.size(); i++){
        int assignedKilo = myenviron.assignedTargets.indexOf(i);
        std::string assignedKiloStr = (assignedKilo >= 0)? std::to_string(assignedKilo) : "";
        drawCircle(myenvironcopy->goal_locations[i]*MM_TO_PIXEL, myenvironcopy->distThreshold*MM_TO_PIXEL, QColor(0, 255, 0, 255), 2, assignedKiloStr );
    }

}
void mykilobotexperiment::addshape(){

    QSettings settings;
    QString lastDir = settings.value("CoordLastDir", QDir::homePath()).toString();
    QString filepath = QFileDialog::getOpenFileName(NULL, tr("Select coordinates file"), lastDir,tr("Text file (*.txt)"));
    if(filepath.isEmpty()) return;
    QTableWidgetItem * item;

    tableofshapes->setRowCount(tableofshapes->rowCount()+1);


    tableofshapes->setItem(tableofshapes->rowCount()-1,0,new QTableWidgetItem());
    item=tableofshapes->item(tableofshapes->rowCount()-1,0);
    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
    item->setText(filepath);

    tableofshapes->setItem(tableofshapes->rowCount()-1,1,new QTableWidgetItem());
    item=tableofshapes->item(tableofshapes->rowCount()-1,1);
    item->setText(QString::number(0));

    tableofshapes->setItem(tableofshapes->rowCount()-1,2,new QTableWidgetItem());
    item=tableofshapes->item(tableofshapes->rowCount()-1,2);
    item->setText(QString::number(0));


    tableofshapes->setItem(tableofshapes->rowCount()-1,3,new QTableWidgetItem());
    item=tableofshapes->item(tableofshapes->rowCount()-1,3);
    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
    item->setBackgroundColor(QColor(0,0,0,50));

    if(tableofshapes->rowCount()>1){
        item=tableofshapes->item(tableofshapes->rowCount()-2,3);
        item->setText(QString::number(5));
        item->setBackgroundColor(QColor(255,255,255,50));
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
    }
}
void mykilobotexperiment::deleteshape() {

    QTableWidgetItem * item;


    tableofshapes->setRowCount(tableofshapes->rowCount()-1);

    if(tableofshapes->rowCount()>0){
        item=tableofshapes->item(tableofshapes->rowCount()-1,3);
        item->setText(QString());
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        item->setBackgroundColor(QColor(0,0,0,50));

    }
}
void mykilobotexperiment::GetExperimentSettings(){

    if(coordinatesFiles.size()>0){
        coordinatesFiles.clear();
        shifts.clear();
        pauses.clear();
    }

    QTableWidgetItem * item;
    for(int i=0;i<tableofshapes->rowCount();i++){
        item=tableofshapes->item(i,0);
        coordinatesFiles.push_back(item->text());
        item=tableofshapes->item(i,1);
        int xshift=item->text().toInt();
        item=tableofshapes->item(i,2);
        int yshift=item->text().toInt();

        shifts.push_back(std::pair<int,int>(xshift,yshift));

        if(i<tableofshapes->rowCount()-1){
            item=tableofshapes->item(i,3);
            pauses.push_back(item->text().toUInt());
        }
    }
}
