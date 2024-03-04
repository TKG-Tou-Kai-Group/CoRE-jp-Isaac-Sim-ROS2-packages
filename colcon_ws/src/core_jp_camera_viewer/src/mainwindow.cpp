#include "core_jp_camera_viewer/mainwindow.h"
#include "ui_mainwindow.h"
#include <opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateImage);
    timer->start(1000 / UPDATE_RATE_Hz); /*ms*/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setCameraImage(cv::Mat &camera_image)
{
    camera_image_ = camera_image.clone();
}

void MainWindow::updateImage()
{
    if (!camera_image_.empty())
    {
        auto img = QImage((uchar *)camera_image_.data, camera_image_.cols, camera_image_.rows, camera_image_.step, QImage::Format_RGB888);
        ui->graphicsView->setImg(img, "camera_image");
    }
}
