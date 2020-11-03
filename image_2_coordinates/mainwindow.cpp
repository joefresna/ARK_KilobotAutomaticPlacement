#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include <QDebug>
#include <fstream>
#include <QMessageBox>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mindistslider->setValue(min_distance);
    ui->mindistlabel->setText(QString::number(min_distance));

    connect(ui->sel_image,SIGNAL(clicked(bool)),this,SLOT(setImagepath()));
    connect(this, SIGNAL(showImage(QPixmap)),ui->image,SLOT(setPixmap(QPixmap)));
    connect(ui->zoomin,SIGNAL(clicked(bool)),this,SLOT(zoomin()));
    connect(ui->zoomout,SIGNAL(clicked(bool)),this,SLOT(zoomout()));
    connect(ui->convert,SIGNAL(clicked(bool)),this,SLOT(convertImage()));
    connect(ui->mindistslider,SIGNAL(valueChanged(int)),this,SLOT(setMindist(int)));
    connect(ui->save,SIGNAL(clicked(bool)),this,SLOT(saveCoordinates()));
    connect(ui->image,SIGNAL(leftclicked(QPoint)),this,SLOT(manuallysetcoordinate(QPoint)));
    connect(ui->image,SIGNAL(rightclicked(QPoint)),this,SLOT(undocoordinates()));
//    connect(ui->image,SIGNAL(rightclicked(QPoint)),this,SLOT(deletecoordinate(QPoint)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setImagepath(){

    QSettings settings;
    QString lastDir = settings.value("ImageLastDir", QDir::homePath()).toString();
    QString dirName = QFileDialog::getOpenFileName(this, tr("Select the shape's image"), lastDir);

    if (!dirName.isEmpty()) {
        settings.setValue ("ImageLastDir", dirName);
        ui->image_path_label->setText(dirName);
        image=imread(dirName.toStdString());

        if(!image.empty()){
            image.copyTo(copyofimage);
            image.copyTo(resultimage);
            imagesize=copyofimage.rows;
            showMat(copyofimage);
            if(!coordinates.empty()){
                coordinates.clear();
            }
        }
    }

}

void MainWindow::showMat(Mat display)
{
    // display
    cv::resize(display,display,Size(600,600));

    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = display;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    emit showImage(pix);
}



void MainWindow::zoomin(){
    if(imagesize <= 4000){
        imagesize=imagesize+200;
        Mat dst;
        cv::resize(image, dst, cv::Size(imagesize,imagesize) , 0, 0,INTER_LINEAR);
        copyofimage=Scalar(0,0,0);
        if(imagesize>2000){
            dst(cv::Rect((imagesize-2000)/2,(imagesize-2000)/2,2000,2000)).copyTo(copyofimage);
        }
        else{
            dst.copyTo(copyofimage(cv::Rect((2000-imagesize)/2,(2000-imagesize)/2,imagesize,imagesize)));
        }

        showMat(copyofimage);
        this->repaint();
        coordinates.clear();
    }
}

void MainWindow::zoomout(){
    if(imagesize >= 600){
        imagesize=imagesize-200;
        Mat dst;
        cv::resize(image, dst, cv::Size(imagesize,imagesize) , 0, 0,INTER_LINEAR);
        copyofimage=Scalar(0,0,0);

        if(imagesize<2000){
            dst.copyTo(copyofimage(cv::Rect((2000-imagesize)/2,(2000-imagesize)/2,imagesize,imagesize)));
        }
        else {
            dst(cv::Rect((imagesize-2000)/2,(imagesize-2000)/2,2000,2000)).copyTo(copyofimage);
        }

        showMat(copyofimage);
        this->repaint();
        coordinates.clear();
    }
}

void MainWindow::convertImage(){


    // clear previous result
    if(!coordinates.empty()) coordinates.clear();
    copyofimage.copyTo(resultimage);


    Mat dst;
    cvtColor(copyofimage,dst,CV_BGR2GRAY);


    //    int row=copyofimage.rows;
    //    int col=copyofimage.cols;


    // shape frame

    int colbeg=-1;
    int colend=-1;
    int rowbeg=-1;
    int rowend=-1;
    int start=-1;
    int stop=-1;

    int z=1;

    //Get shape frame

    long int sum;

    for(int i=1999;i>1;i--){
        sum=0;
        for(int j=0;j<2000;j++){
            sum+= (int) dst.at<uchar>(j,i);
        }
        if(sum!=0) {
            colbeg=i;
        }
    }

    for(int i=0;i<2000;i++){
        sum=0;
        for(int j=0;j<2000;j++){
            sum+= (int) dst.at<uchar>(j,i);
        }
        if(sum!=0) {
            colend=i;
        }
    }

    for(int i=1999;i>1;i--){
        sum=0;
        for(int j=0;j<2000;j++){
            sum+= (int) dst.at<uchar>(i,j);
        }
        if(sum!=0) {
            rowbeg=i;
        }
    }

    for(int i=0;i<2000;i++){
        sum=0;
        for(int j=0;j<2000;j++){
            sum+= (int) dst.at<uchar>(i,j);
        }
        if(sum!=0) {
            rowend=i;
        }
    }

    // get colums indeces

    int columnnum=(colend-colbeg)/(min_distance*10); // min distance is converted from cm to px
    int columndist=round((colend-colbeg)/columnnum);

    columnnum++;

    int colind[columnnum];

    for(int i=0;i<columnnum;i++){
        if(i==0) colind[i]=round(colbeg+columndist/2);
        else colind[i]=round(colind[i-1]+columndist);
    }


    for(int k=0;k<columnnum;k++){

        int j = colind[k];

        for(int i=rowbeg;i<=rowend;i++) {

            if((dst.at<uchar>(i,j)>=200) && (start==-1))
                start=i;

            if((dst.at<uchar>(i,j)<200) && (start!=-1))
                stop=i-1;


            if((start!=-1)&&(stop!=-1)){

                int dist=stop-start+1;
                int number=round(dist/(min_distance*10))+1;
                dist=dist/number;

                int m=start;
                while(m<=stop){
                    coordinates.push_back( Point(j,m) ); // or "m+dist/2"
                    m=m+dist;
                }

                i=stop+1;
                start=-1;
                stop=-1;
            }
        }

    }

    ui->numberofpoints->setEnabled(true);
    ui->numberofpoints->display( QString::number((int) coordinates.size()));
    for(int i=0; i< coordinates.size(); i++)
        circle(resultimage,coordinates[i],10,Scalar(255,0,0),-1);
    showMat(resultimage);
    this->repaint();

}


void MainWindow::setMindist(int min_dist){
    min_distance=min_dist;
    ui->mindistlabel->setText(QString::number(min_distance));
}

void MainWindow::saveCoordinates(){
    if(coordinates.empty()){
        QMessageBox::information(this, tr("Error"),QString("There is no coordinates to save yet!"));
    }
    else
    {
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        tr("Save Coordinates"), QDir::homePath(),
                                                        tr("Text file (*.txt);;All Files (*)"));

        if (fileName.isEmpty())
            return;
        else {
            QFile file(fileName);
            if (!file.open(QIODevice::WriteOnly)) {
                QMessageBox::information(this, tr("Unable to open file"),
                                         file.errorString());
                return;
            }

            QTextStream out(&file);
//            out.setVersion(QDataStream::Qt_4_5);

            out << " X\t Y"<<"\n";
            for(int i=0;i<( (int) coordinates.size());i++)
                out << QString::number(coordinates[i].x)<<"\t"<<QString::number(coordinates[i].y)<<"\n";

            file.close();

        }
    }

}


void MainWindow::manuallysetcoordinate(QPoint pt){
    if(ui->manualset->isChecked()){
        copyofimage.copyTo(resultimage);
        coordinates.push_back(Point(pt.x()*2000/600,pt.y()*2000/600));
        for(int i=0; i< coordinates.size(); i++)
            circle(resultimage,coordinates[i],10,Scalar(255,0,0),-1);
        showMat(resultimage);
        ui->numberofpoints->setEnabled(true);
        ui->numberofpoints->display( QString::number((int) coordinates.size()));
        this->repaint();
    }
}


void MainWindow::undocoordinates(){
    copyofimage.copyTo(resultimage);
    coordinates.pop_back();
    for(int i=0; i< coordinates.size(); i++)
        circle(resultimage,coordinates[i],10,Scalar(255,0,0),-1);
    showMat(resultimage);
    ui->numberofpoints->setEnabled(true);
    ui->numberofpoints->display( QString::number((int) coordinates.size()));
    this->repaint();
}
void MainWindow::deletecoordinate(QPoint pt){
    unsigned int mindist=UINT_MAX;
    int min =-1;
    for(int i=0; i< coordinates.size(); i++){
        float dist=sqrt((coordinates[i].x-pt.x()*2000/600)^2+(coordinates[i].y-pt.y()*2000/600)^2);
        if(dist<mindist){
            mindist=dist;
            min=i;
        }
    }

    coordinates.erase(coordinates.begin()+min);

    copyofimage.copyTo(resultimage);
    for(int i=0; i< coordinates.size(); i++)
        circle(resultimage,coordinates[i],10,Scalar(255,0,0),-1);
    showMat(resultimage);
    ui->numberofpoints->setEnabled(true);
    ui->numberofpoints->display( QString::number((int) coordinates.size()));
    this->repaint();
}

