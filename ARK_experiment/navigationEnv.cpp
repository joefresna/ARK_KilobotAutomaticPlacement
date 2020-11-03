#include "navigationEnv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>
#include <QtMath>

#include "kilobot.h"

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {

    // Define Environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).

    // Once setup in Gui - do this here instead of in experiment initialise:
    //    setupEnvironment1( );

    // Setup goal environmental features:
    //    setupGoal(numGoal); // should connect with UI to setup goal locations and parameters

    resetEnvironment();
    qDebug() << "Environment is up and running.";
}

// Reset all variables
void mykilobotenvironment::resetEnvironment() {
    goal_locations.clear();
    assignedTargets.clear();
    commandLog.clear();
    arrived.clear();
    minTimeBetweenTwoMsg = 0;
    lastSent.clear();
    time = 0;
    sendLED = false;
    paused = false;
}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {

}

// Update the Kilobot virtual sensor readings here.
// generate virtual sensor readings & send to KB
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot) {

    if (paused || assignedTargets.empty()){ return; }

    kilobot_id kID = kilobot.getID();

    if (sendLED){
        int colour = 1; // 0=WHITE, 1=RED, 2=GREEN, 3=BLUE
        int ledDelay = qRound(LEDdelays[kID]*90.0);
        kilobot_message msg;
        msg.id = kID;
        msg.type = LIGHT;
        msg.data = (colour*256) + ledDelay;
        emit transmitKiloState(msg);

        return;
    }

//    uint16_t VSval = 0; // VS value to send to kb

//    if (kID >= goal_locations.size()) {
//        qDebug() << "WARNING! More goals than robots!!!";
//        return;
//    }
    if (!this->arrived[kID]) {

        bool sendMessage = false;
        command cmd = STRAIGHT;

        // Get kilobot position and led colour.
        QPointF kPos = kilobot.getPosition()*PIXEL_TO_MM;

        // Compute the path to the goal (easy path: straight line from kilobot to goal)
//        int targetIdx = allKiloIDs.indexOf(kID);
        int targetIdx = assignedTargets[kID];
        if (targetIdx < 0) { // if the current robot is not assigned to any target, no directions are needed
            return;
        }
        QLineF path(kPos, goal_locations[targetIdx]);

        if (path.length() < distThreshold) {  // closer than the min-dist threshold
            this->arrived[kID] = true;
            qDebug() << "Kilobot #" << kID << "arrived to its destination" << goal_locations[targetIdx];
            sendMessage = true;
            cmd = STOP;
        }
        else { // decide the direction
            double pathAngle = path.angle();
            double kOrientation = qRadiansToDegrees(qAtan2(-kilobot.getVelocity().y(), kilobot.getVelocity().x()));
            double diffAngle = pathAngle - kOrientation;
            if (qAbs(diffAngle) < 55){ // if the goal is less than 45 degrees in front of the kilobot: GO STRAIGHT
                cmd = STRAIGHT;
            } else {
                // normalise the diffAngle between -180 and 180
                while (diffAngle < -180){
                    diffAngle += 360;
                }
                while (diffAngle > 180){
                    diffAngle -= 360;
                }
                if (diffAngle < 0){
                    cmd = RIGHT;
                } else {
                    cmd = LEFT;
                }
            }

//            if (this->commandLog[kID] != cmd){
//                QString sCmd = (cmd==STRAIGHT)? "STRAIGHT" : (cmd==LEFT)? "LEFT" : (cmd==RIGHT)? "RIGHT" : "STOP";
//                qDebug() << "Sending to kilobot #" << kID << "command" << cmd << sCmd;
//                qDebug() << "[" << kID << "] pathAngle" << pathAngle << "kOrientation" << kOrientation << "diffAngle(" << (pathAngle-kOrientation) << ")" << diffAngle;
//            }
        }

        // Deciding if sending or not the message
//        if (cmd == STOP && this->commandLog[kID] != cmd){ // if cmd is first STOP I send anyway
//            sendMessage = true;
//        }
//        if ( (cmd == LEFT || cmd == RIGHT) && this->time - this->lastSent[kID] > minTimeBetweenTwoMsg){ // after minTimeBetweenTwoMsg I can send turn again
//            sendMessage = true;
//        }
//        if ( cmd != STOP && this->lastSent[kID] < 0) { // if it's the first motion message I send in anyway
//            sendMessage = true;
//        }
//        if (sendMessage){
//            this->commandLog[kID] = cmd;
//            qDebug() << "Sending to kilobot #" << kID << "command" << cmd;
//        }
        if (this->commandLog[kID] != cmd && // sending only if the new command is different from the previous one
                (cmd == STOP || this->time - this->lastSent[kID] > minTimeBetweenTwoMsg)){ // and enough time has elapsed (except for STOP)
            this->commandLog[kID] = cmd;
            sendMessage = true;
            lastSent[kID] = this->time;
            //qDebug() << "Sending to kilobot #" << kID << "command" << cmd;
        }

        if (sendMessage){
            kilobot_message msg;
            msg.id = kID;
            msg.type = cmd;
            msg.data = 0;
            emit transmitKiloState(msg);
        }

    } else { // if the robot has been marked as 'arrived' but it has been pushed by other robots


        int targetIdx = assignedTargets[kID];
        if (targetIdx < 0) { // if the current robot is not assigned to any target, no directions are needed
            return;
        }
        double pathLength = QLineF(kilobot.getPosition()*PIXEL_TO_MM, goal_locations[targetIdx]).length();
        if (pathLength > replacementSensitivity) { // it has been moved more than 2.5cm
            this->arrived[kID] = false;
        }
    }

}



