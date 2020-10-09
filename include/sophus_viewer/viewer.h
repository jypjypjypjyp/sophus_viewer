#ifndef SOPHUS_VIEWER_VIEWER_H
#define SOPHUS_VIEWER_VIEWER_H

#include <pangolin/pangolin.h>
#include <sophus_viewer/common.h>

namespace sophus_viewer
{
class Viewer
{
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();
    void Close();
    void Wait();
    // Twc: pose.inverse()
    void AddFrame(double time, SE3d Twc);
    void AddPoint(unsigned long id, Vector3d point);
    // Twc: pose.inverse()
    void SetViewerPose(SE3d Twc);

private:
    void ThreadLoop();
    void DrawAxis();
    void DrawFrame(SE3d Twc, const float *color);
    void DrawFrames();
    void DrawPoints();
    void UpdateViewerPose(pangolin::OpenGlRenderState &vis_camera);

    std::thread thread_;
    bool viewer_running_ = true;
    std::mutex viewer_data_mutex_;
    Matrix4d view_maxtrix_;
    std::map<double, SE3d> frames;
    std::map<unsigned long, Vector3d> points;
};
} // namespace sophus_viewer

#endif // SOPHUS_VIEWER_VIEWER_H
