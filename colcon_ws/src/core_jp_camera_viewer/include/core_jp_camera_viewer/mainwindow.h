#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <QMainWindow>
#include <QTimer>
#include <chrono>

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    static const int UPDATE_RATE_Hz = 10;
    static constexpr float INITIALIZE_LIMIT_s = 30.0f;

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setCameraImage(cv::Mat &camera_image);

private slots:
    void updateImage();

private:
    // 画像を画像に貼り付ける関数
    void paste(cv::Mat &dst, cv::Mat &src, int x, int y, int width, int height)
    {
        cv::Mat resized_img;
        cv::resize(src, resized_img, cv::Size(width, height));

        if (x >= dst.cols || y >= dst.rows)
            return;
        int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
        int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
        int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
        int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
        int px = std::max(x, 0);
        int py = std::max(y, 0);

        cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
        cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));
        roi_resized.copyTo(roi_dst);
    }

    // 画像を画像に貼り付ける関数（サイズ指定を省略したバージョン）
    void paste(cv::Mat &dst, cv::Mat &src, int x, int y)
    {
        paste(dst, src, x, y, src.cols, src.rows);
    }

    Ui::MainWindow *ui;

    QTimer *timer;

    cv::Mat camera_image_;
};

#endif // MAINWINDOW_H
