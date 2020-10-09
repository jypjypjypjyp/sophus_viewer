#include "sophus_viewer/viewer.h"

using namespace sophus_viewer;

int main()
{
    auto viewer = Viewer::Ptr(new Viewer);
    viewer->AddFrame(1.0, SE3d(SO3d(), Vector3d::Zero()));
    viewer->AddFrame(2.0, SE3d(SO3d(), Vector3d(1, 2, 3)));
    viewer->AddFrame(3.0, SE3d(SO3d(), Vector3d(3, 4, 5)));
    viewer->SetViewerPose(SE3d(SO3d(), Vector3d::Zero()));
    viewer->Wait();
}