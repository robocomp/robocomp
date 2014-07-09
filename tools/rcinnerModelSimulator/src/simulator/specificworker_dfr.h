// ------------------------------------------------------------------------------------------------
// DifferentialRobot.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::dfr_getBaseState ( const QString& server, RoboCompDifferentialRobot::TBaseState& state )
{}


void SpecificWorker::dfr_getBasePose ( const QString& server, int& x, int& z, float& alpha )
{}


void SpecificWorker::dfr_setSpeedBase ( const QString& server, float adv, float rot )
{}


void SpecificWorker::dfr_stopBase ( const QString& server )
{}


void SpecificWorker::dfr_resetOdometer ( const QString& server )
{}


void SpecificWorker::dfr_setOdometer ( const QString& server, const RoboCompDifferentialRobot::TBaseState& state )
{}


void SpecificWorker::dfr_setOdometerPose ( const QString& server, int x, int z, float alpha )
{}


void SpecificWorker::dfr_correctOdometer ( const QString& server, int x, int z, float alpha )
{}
