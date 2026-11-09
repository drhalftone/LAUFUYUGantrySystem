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
// The mouse wheel zooms about the cursor and dragging with the left button moves the picture, which coasts on
// when released.  Zoom and pan are applied in the last shader, so nothing else depends on them.
//
#ifndef LAUGLIMAGEVIEW_H
#define LAUGLIMAGEVIEW_H

#include <QImage>
#include <QTimer>
#include <QPointF>
#include <QElapsedTimer>
#include <QColor>
#include <QVector>
#include <QPolygonF>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLFramebufferObject>
#include <QOpenGLVertexArrayObject>
#include <opencv2/core.hpp>

// Lines drawn over a camera frame, in raw camera pixels (the view applies the lens correction itself)
struct LAUViewOverlayLayer
{
    QVector<QPolygonF> paths;
    QColor color;
    qreal width = 1.0;
    QString label;
};

struct LAUViewOverlay
{
    QVector<LAUViewOverlayLayer> layers;
};

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
    void setOverlay(const LAUViewOverlay &overlay);

    double viewZoom() const
    {
        return zoom;
    }
    QPointF viewCentre() const
    {
        return centre;
    }
    bool isCoasting() const
    {
        return coastTimer->isActive();
    }

protected:
    void initializeGL() override;
    void paintGL() override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private slots:
    void onCoast();

private:
    double zoom = 1.0;                           // 1 = the whole picture fits the window
    QPointF centre = QPointF(0.5, 0.5);          // picture point at the middle of the window, 0..1
    bool dragging = false;
    QPointF lastMouse;
    QPointF velocity;                            // px per second
    QElapsedTimer moveClock, coastClock;
    QTimer *coastTimer = nullptr;

    LAUViewOverlay overlay, shownOverlay;        // as given; and in the picture's own (perhaps corrected) pixels
    bool overlayDirty = false;

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
    QSize pictureSize() const;
    double screenScale() const;                  // window pixels per picture pixel
    bool clampCentre();                          // true if it had to move
    void stopCoasting();
    void prepareOverlay();
    void drawOverlay();
    void setViewUniform(QOpenGLShaderProgram *program);
    void bindTexture(int unit, GLuint texture);
    void drawQuad();
};

#endif // LAUGLIMAGEVIEW_H
