/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUImageView -- OpenGL picture, scaled to fit and keeping its shape.  Shows either a still QImage or a raw
// Mono10 camera frame.  A raw frame is levelled (the same maths as LAUBreadboardCalibration::picture()) and,
// when enabled, has the lens distortion removed, all in fragment shaders:
//   1. log of the frame, averaged down to 1/8 scale
//   2. wide Gaussian blur of that, in two passes: the brightness of each neighbourhood
//   3. the levelled log picture at every 4th pixel, read back only for its two display percentiles
//   4. to the screen: for each screen pixel find its place in the distorted frame, level it, and scale it
// The overall level (the median of step 2) and the percentiles are the only numbers read back to the CPU.
//
#ifndef LAUGLIMAGEVIEW_H
#define LAUGLIMAGEVIEW_H

#include <QImage>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLFramebufferObject>
#include <QOpenGLVertexArrayObject>
#include <opencv2/core.hpp>

class LAUImageView : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit LAUImageView(QWidget *parent = nullptr);
    ~LAUImageView() override;

    void setImage(const QImage &img);
    void setFrame(const cv::Mat &raw16);         // CV_16UC1, continuous, 10-bit values

    // newK is the corrected picture's camera matrix, as given to cv::undistort
    void setUndistortion(const cv::Matx33d &K, const double dist[5], const cv::Matx33d &newK);
    void enableUndistortion(bool state);

protected:
    void initializeGL() override;
    void paintGL() override;

private:
    QImage image;
    cv::Mat frame;
    bool showingFrame = false;
    bool frameDirty = false;
    bool imageDirty = false;

    bool correcting = false;
    cv::Matx33d cameraK, cameraNewK;
    double cameraDist[5] = { 0, 0, 0, 0, 0 };

    bool glReady = false;
    QOpenGLVertexArrayObject vao;
    QOpenGLShaderProgram *logProgram = nullptr;
    QOpenGLShaderProgram *blurProgram = nullptr;
    QOpenGLShaderProgram *statsProgram = nullptr;
    QOpenGLShaderProgram *frameProgram = nullptr;
    QOpenGLShaderProgram *imageProgram = nullptr;
    QOpenGLFramebufferObject *smallFbo = nullptr;
    QOpenGLFramebufferObject *blurFbo = nullptr;
    QOpenGLFramebufferObject *localFbo = nullptr;
    QOpenGLFramebufferObject *statsFbo = nullptr;
    GLuint rawTexture = 0;
    GLuint imageTexture = 0;
    cv::Size rawSize;

    float level = 0.0f;                          // overall log brightness
    float blackPoint = 0.0f, whitePoint = 1.0f;  // display percentiles of the levelled log picture

    QOpenGLShaderProgram *buildProgram(const QString &fragmentSource);
    void releaseFbos();
    void prepareFbos(const cv::Size &size);
    void uploadFrame();
    void processFrame();
    void drawFrame();
    void drawImage();
    void fitViewport(const QSize &picture);
    void bindTexture(int unit, GLuint texture);
    void drawQuad();
};

#endif // LAUGLIMAGEVIEW_H
