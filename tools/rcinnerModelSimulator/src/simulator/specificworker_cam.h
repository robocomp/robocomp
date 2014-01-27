// ------------------------------------------------------------------------------------------------
// Camera.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::cam_camera_getYUVImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{
}


void SpecificWorker::cam_getYImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYLogPolarImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYImageCR ( const QString& server, int cam, int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getRGBPackedImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYRGBImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


TCamParams SpecificWorker::cam_getCamParams ( const QString& server )
{
	TCamParams tcp;
	return tcp;
}


void SpecificWorker::cam_setInnerImage ( const QString& server, const RoboCompCamera::imgType& roi )
{}
