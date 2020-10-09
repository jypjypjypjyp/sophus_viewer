#include "sophus_viewer/viewer.h"

namespace sophus_viewer
{

const float RED[3] = {1, 0, 0};
const float GREEN[3] = {0, 1, 0};
const float BLUE[3] = {0, 0, 1};

Viewer::Viewer()
{
    thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    view_maxtrix_ = Matrix4d::Identity();
}

void Viewer::Close()
{
    viewer_running_ = false;
    thread_.join();
}

void Viewer::Wait()
{
    thread_.join();
}

void Viewer::AddFrame(double time, SE3d Twc)
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    frames[time] = Twc;
}

void Viewer::AddPoint(unsigned long id, Vector3d point)
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    points[id] = point;
}

void Viewer::SetViewerPose(SE3d Twc)
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    view_maxtrix_ = Twc.matrix();
}

void Viewer::ThreadLoop()
{
    pangolin::CreateWindowAndBind("Sophus", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    while (!pangolin::ShouldQuit() && viewer_running_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);

        UpdateViewerPose(vis_camera);
        DrawAxis();
        DrawFrames();
        DrawPoints();

        pangolin::FinishFrame();
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

void Viewer::UpdateViewerPose(pangolin::OpenGlRenderState &vis_camera)
{
    pangolin::OpenGlMatrix m(view_maxtrix_);
    vis_camera.Follow(m, true);
}

void Viewer::DrawAxis()
{
    const float sz = 1.0;
    const int line_width = 2.0;

    glPushMatrix();
    glMultMatrixf((GLfloat *)view_maxtrix_.data());
    glLineWidth(line_width);
    glBegin(GL_LINES);
    glColor3f(RED[0], RED[1], RED[2]);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * 1, 0, 0);

    glColor3f(GREEN[0], GREEN[1], GREEN[2]);
    glVertex3f(0, 0, 0);
    glVertex3f(0, sz * 1, 0);

    glColor3f(BLUE[0], BLUE[1], BLUE[2]);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, sz * 1);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawFrame(SE3d Twc, const float *color)
{
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat *)m.data());

    if (color == nullptr)
    {
        glColor3f(1, 0, 0);
    }
    else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawFrames()
{
    for (auto &pair : frames)
    {
        DrawFrame(pair.second, GREEN);
    }
}

void Viewer::DrawPoints()
{
    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto pair : points)
    {
        auto point = pair.second;
        glColor3f(RED[0], RED[1], RED[2]);
        glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
}

} // namespace sophus_viewer
