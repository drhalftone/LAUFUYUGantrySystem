/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauglimageview.h"
#include "laubreadboardcalibration.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <QDebug>
#include <QPainter>
#include <QSurfaceFormat>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <opencv2/imgproc.hpp>

namespace
{
    const int SMALL_SCALE = 8;               // the neighbourhood blur runs at 1/8 scale; it is smooth anyway
    const int STATS_SCALE = 4;               // display percentiles come from every 4th pixel

    const char *VERTEX_SOURCE = R"(#version 330 core
out vec2 uv;
void main()
{
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

    // each output pixel is the mean log brightness of one 8 x 8 block
    const char *LOG_SOURCE = R"(#version 330 core
uniform sampler2D raw;
uniform int scale;
out vec4 color;
void main()
{
    ivec2 origin = ivec2(gl_FragCoord.xy) * scale;
    float total = 0.0;
    for (int y = 0; y < scale; y++) {
        for (int x = 0; x < scale; x++) {
            total += log(texelFetch(raw, origin + ivec2(x, y), 0).r * 65535.0 + 1.0);
        }
    }
    color = vec4(total / float(scale * scale));
}
)";

    // RADIUS is filled in; edges reflect like OpenCV's default BORDER_REFLECT_101
    const char *BLUR_SOURCE = R"(#version 330 core
uniform sampler2D source;
uniform float kernel[RADIUS + 1];
uniform int dx;
uniform int dy;
out vec4 color;
int mirror(int c, int n)
{
    int period = max(2 * (n - 1), 1);
    c = abs(c) % period;
    return (c < n) ? c : period - c;
}
void main()
{
    ivec2 size = textureSize(source, 0);
    ivec2 here = ivec2(gl_FragCoord.xy);
    float total = kernel[0] * texelFetch(source, here, 0).r;
    for (int i = 1; i <= RADIUS; i++) {
        ivec2 a = here + ivec2(dx, dy) * i;
        ivec2 b = here - ivec2(dx, dy) * i;
        total += kernel[i] * (texelFetch(source, ivec2(mirror(a.x, size.x), mirror(a.y, size.y)), 0).r +
                              texelFetch(source, ivec2(mirror(b.x, size.x), mirror(b.y, size.y)), 0).r);
    }
    color = vec4(total);
}
)";

    const char *STATS_SOURCE = R"(#version 330 core
uniform sampler2D raw;
uniform sampler2D local;
uniform int scale;
uniform float strength;
uniform float level;
out vec4 color;
void main()
{
    ivec2 p = ivec2(gl_FragCoord.xy) * scale;
    float lg = log(texelFetch(raw, p, 0).r * 65535.0 + 1.0);
    float neighbourhood = texture(local, (vec2(p) + 0.5) / vec2(textureSize(raw, 0))).r;
    color = vec4(lg - strength * (neighbourhood - level));
}
)";

    const char *FRAME_SOURCE = R"(#version 330 core
in vec2 uv;
uniform sampler2D raw;
uniform sampler2D local;
uniform float strength;
uniform float level;
uniform float blackPoint;
uniform float whitePoint;
uniform int correcting;
uniform vec4 cameraK;       // fx, fy, cx, cy of the lens
uniform vec4 cameraNewK;    // the same of the corrected picture
uniform vec3 radial;        // k1, k2, k3
out vec4 color;
void main()
{
    vec2 size = vec2(textureSize(raw, 0));
    vec2 source = vec2(uv.x, 1.0 - uv.y) * size - 0.5;    // the frame's first row is the top of the screen
    if (correcting != 0) {
        vec2 ideal = (source - cameraNewK.zw) / cameraNewK.xy;
        float r2 = dot(ideal, ideal);
        source = ideal * (1.0 + r2 * (radial.x + r2 * (radial.y + r2 * radial.z))) * cameraK.xy + cameraK.zw;
    }
    if (source.x < -0.5 || source.y < -0.5 || source.x > size.x - 0.5 || source.y > size.y - 0.5) {
        color = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    vec2 place = (source + 0.5) / size;
    float lg = log(texture(raw, place).r * 65535.0 + 1.0);
    float levelled = lg - strength * (texture(local, place).r - level);
    float scale = 245.0 / max(whitePoint - blackPoint, 1e-6);
    color = vec4(vec3(clamp(((levelled - blackPoint) * scale + 5.0) / 255.0, 0.0, 1.0)), 1.0);
}
)";

    const char *IMAGE_SOURCE = R"(#version 330 core
in vec2 uv;
uniform sampler2D picture;
out vec4 color;
void main()
{
    color = vec4(texture(picture, vec2(uv.x, 1.0 - uv.y)).rgb, 1.0);
}
)";

    QOpenGLFramebufferObject *floatFbo(int width, int height, QOpenGLFunctions *gl)
    {
        QOpenGLFramebufferObjectFormat format;
        format.setInternalTextureFormat(GL_R32F);
        QOpenGLFramebufferObject *fbo = new QOpenGLFramebufferObject(width, height, format);
        gl->glBindTexture(GL_TEXTURE_2D, fbo->texture());
        gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        return fbo;
    }

    std::vector<float> readFbo(QOpenGLFramebufferObject *fbo, QOpenGLFunctions *gl)
    {
        std::vector<float> values((size_t)fbo->width() * fbo->height());
        fbo->bind();
        gl->glReadPixels(0, 0, fbo->width(), fbo->height(), GL_RED, GL_FLOAT, values.data());
        return values;
    }

    float percentile(std::vector<float> &values, double q)
    {
        const size_t k = std::min(values.size() - 1, (size_t)std::llround(q / 100.0 * (values.size() - 1)));
        std::nth_element(values.begin(), values.begin() + k, values.end());
        return values[k];
    }
}

LAUImageView::LAUImageView(QWidget *parent) : QOpenGLWidget(parent)
{
    setMinimumSize(640, 400);
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);
}

LAUImageView::~LAUImageView()
{
    if (context() == nullptr) {
        return;
    }
    makeCurrent();
    releaseFbos();
    delete logProgram;
    delete blurProgram;
    delete statsProgram;
    delete frameProgram;
    delete imageProgram;
    if (rawTexture) {
        glDeleteTextures(1, &rawTexture);
    }
    if (imageTexture) {
        glDeleteTextures(1, &imageTexture);
    }
    vao.destroy();
    doneCurrent();
}

void LAUImageView::setImage(const QImage &img)
{
    image = img;
    showingFrame = false;
    imageDirty = true;
    update();
}

void LAUImageView::setFrame(const cv::Mat &raw16)
{
    frame = raw16;
    showingFrame = true;
    frameDirty = true;
    update();
}

void LAUImageView::setUndistortion(const cv::Matx33d &K, const double dist[5], const cv::Matx33d &newK)
{
    cameraK = K;
    cameraNewK = newK;
    std::copy(dist, dist + 5, cameraDist);
    update();
}

void LAUImageView::enableUndistortion(bool state)
{
    correcting = state;
    update();
}

QOpenGLShaderProgram *LAUImageView::buildProgram(const QString &fragmentSource)
{
    QOpenGLShaderProgram *program = new QOpenGLShaderProgram(this);
    if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, QString::fromLatin1(VERTEX_SOURCE)) ||
        !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentSource) || !program->link()) {
        qWarning().noquote() << "LAUImageView: shader failed:" << program->log();
        glReady = false;
    }
    return program;
}

void LAUImageView::initializeGL()
{
    initializeOpenGLFunctions();
    vao.create();

    const double sigma = LAUBreadboardCalibration::NEIGHBOURHOOD_PX / SMALL_SCALE;
    const int ksize = cvRound(sigma * 8.0 + 1.0) | 1;     // what cv::GaussianBlur picks for float images
    const QString blur = QString::fromLatin1(BLUR_SOURCE).replace(QString("RADIUS"), QString::number(ksize / 2));

    glReady = true;
    logProgram = buildProgram(QString::fromLatin1(LOG_SOURCE));
    blurProgram = buildProgram(blur);
    statsProgram = buildProgram(QString::fromLatin1(STATS_SOURCE));
    frameProgram = buildProgram(QString::fromLatin1(FRAME_SOURCE));
    imageProgram = buildProgram(QString::fromLatin1(IMAGE_SOURCE));
    if (!glReady) {
        return;
    }

    cv::Mat kernel = cv::getGaussianKernel(ksize, sigma, CV_32F);
    blurProgram->bind();
    blurProgram->setUniformValueArray("kernel", kernel.ptr<GLfloat>(ksize / 2), ksize / 2 + 1, 1);
    blurProgram->release();

    GLuint textures[2];
    glGenTextures(2, textures);
    rawTexture = textures[0];
    imageTexture = textures[1];
    for (GLuint texture : textures) {
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
}

void LAUImageView::releaseFbos()
{
    delete smallFbo;
    delete blurFbo;
    delete localFbo;
    delete statsFbo;
    smallFbo = blurFbo = localFbo = statsFbo = nullptr;
}

void LAUImageView::prepareFbos(const cv::Size &size)
{
    releaseFbos();
    smallFbo = floatFbo(size.width / SMALL_SCALE, size.height / SMALL_SCALE, this);
    blurFbo = floatFbo(size.width / SMALL_SCALE, size.height / SMALL_SCALE, this);
    localFbo = floatFbo(size.width / SMALL_SCALE, size.height / SMALL_SCALE, this);
    statsFbo = floatFbo(size.width / STATS_SCALE, size.height / STATS_SCALE, this);
}

void LAUImageView::bindTexture(int unit, GLuint texture)
{
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, texture);
}

void LAUImageView::drawQuad()
{
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

void LAUImageView::uploadFrame()
{
    glBindTexture(GL_TEXTURE_2D, rawTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
    if (rawSize != frame.size()) {
        rawSize = frame.size();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, rawSize.width, rawSize.height, 0, GL_RED, GL_UNSIGNED_SHORT, frame.data);
        prepareFbos(rawSize);
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, rawSize.width, rawSize.height, GL_RED, GL_UNSIGNED_SHORT, frame.data);
    }
}

void LAUImageView::processFrame()
{
    uploadFrame();

    smallFbo->bind();
    glViewport(0, 0, smallFbo->width(), smallFbo->height());
    logProgram->bind();
    bindTexture(0, rawTexture);
    logProgram->setUniformValue("raw", 0);
    logProgram->setUniformValue("scale", SMALL_SCALE);
    drawQuad();

    blurProgram->bind();
    blurProgram->setUniformValue("source", 0);
    blurFbo->bind();
    bindTexture(0, smallFbo->texture());
    blurProgram->setUniformValue("dx", 1);
    blurProgram->setUniformValue("dy", 0);
    drawQuad();

    localFbo->bind();
    bindTexture(0, blurFbo->texture());
    blurProgram->setUniformValue("dx", 0);
    blurProgram->setUniformValue("dy", 1);
    drawQuad();

    std::vector<float> values = readFbo(localFbo, this);
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    level = values[values.size() / 2];

    statsFbo->bind();
    glViewport(0, 0, statsFbo->width(), statsFbo->height());
    statsProgram->bind();
    bindTexture(0, rawTexture);
    bindTexture(1, localFbo->texture());
    statsProgram->setUniformValue("raw", 0);
    statsProgram->setUniformValue("local", 1);
    statsProgram->setUniformValue("scale", STATS_SCALE);
    statsProgram->setUniformValue("strength", (GLfloat)LAUBreadboardCalibration::STRENGTH);
    statsProgram->setUniformValue("level", level);
    drawQuad();

    values = readFbo(statsFbo, this);
    blackPoint = percentile(values, 0.5);
    whitePoint = percentile(values, 99.8);
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
}

void LAUImageView::fitViewport(const QSize &picture)
{
    const qreal ratio = devicePixelRatioF();
    const QSize device(qRound(width() * ratio), qRound(height() * ratio));
    const QSize fitted = picture.scaled(device, Qt::KeepAspectRatio);
    glViewport((device.width() - fitted.width()) / 2, (device.height() - fitted.height()) / 2, fitted.width(), fitted.height());
}

void LAUImageView::drawFrame()
{
    fitViewport(QSize(rawSize.width, rawSize.height));
    frameProgram->bind();
    bindTexture(0, rawTexture);
    bindTexture(1, localFbo->texture());
    frameProgram->setUniformValue("raw", 0);
    frameProgram->setUniformValue("local", 1);
    frameProgram->setUniformValue("strength", (GLfloat)LAUBreadboardCalibration::STRENGTH);
    frameProgram->setUniformValue("level", level);
    frameProgram->setUniformValue("blackPoint", blackPoint);
    frameProgram->setUniformValue("whitePoint", whitePoint);
    frameProgram->setUniformValue("correcting", correcting ? 1 : 0);
    frameProgram->setUniformValue("cameraK", QVector4D(cameraK(0, 0), cameraK(1, 1), cameraK(0, 2), cameraK(1, 2)));
    frameProgram->setUniformValue("cameraNewK", QVector4D(cameraNewK(0, 0), cameraNewK(1, 1), cameraNewK(0, 2), cameraNewK(1, 2)));
    frameProgram->setUniformValue("radial", QVector3D(cameraDist[0], cameraDist[1], cameraDist[4]));
    drawQuad();
}

void LAUImageView::drawImage()
{
    if (imageDirty) {
        const QImage rgba = image.convertToFormat(QImage::Format_RGBA8888);
        glBindTexture(GL_TEXTURE_2D, imageTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, rgba.bytesPerLine() / 4);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, rgba.width(), rgba.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.constBits());
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        imageDirty = false;
    }
    fitViewport(image.size());
    imageProgram->bind();
    bindTexture(0, imageTexture);
    imageProgram->setUniformValue("picture", 0);
    drawQuad();
}

void LAUImageView::paintGL()
{
    // QPainter (the waiting message) leaves blending and the like switched on, which would wreck the float passes
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const bool haveFrame = showingFrame && !frame.empty();
    if (glReady && (haveFrame || (!showingFrame && !image.isNull()))) {
        vao.bind();
        if (haveFrame) {
            if (frameDirty) {
                processFrame();
                frameDirty = false;
            }
            drawFrame();
        } else {
            drawImage();
        }
        vao.release();
        glUseProgram(0);
        return;
    }

    QPainter p(this);
    p.setPen(Qt::gray);
    p.drawText(rect(), Qt::AlignCenter, glReady ? QString("Waiting for the camera...") : QString("This computer's OpenGL 3.3 isn't working."));
}
