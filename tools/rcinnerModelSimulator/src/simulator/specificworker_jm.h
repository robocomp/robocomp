// ------------------------------------------------------------------------------------------------
// JointMotor.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::jm_setPosition(const QString &name, const MotorGoalPosition &goal)
{
	if (goal.maxSpeed == 0.0f)
	{
		printf("Instantaneous movement!\n");
		JointMovement m;
		m.endPos = goal.position;
		m.endSpeed = goal.maxSpeed;
		m.maxAcc = INFINITY;
		m.mode = JointMovement::FixedPosition;
		d->jointMovements[name] = m;
	}
	else
	{
		printf("Target position\n");
		JointMovement m;
		m.endPos = goal.position;
		m.endSpeed = goal.maxSpeed;
		m.maxAcc = INFINITY;
		m.mode = JointMovement::TargetPosition;
		d->jointMovements[name] = m;
	}
}
// 		joint->setAngle(goal.position);

void SpecificWorker::jm_setVelocity(const QString &name, const MotorGoalVelocity &goal)
{
	printf("Target speed\n");
	JointMovement m;
	m.endPos = INFINITY;
	m.endSpeed = goal.velocity;
	m.maxAcc = goal.maxAcc;
	m.mode = JointMovement::TargetSpeed;
	d->jointMovements[name] = m;
}


void SpecificWorker::jm_setSyncPosition(const QString &server, const MotorGoalPositionList &listGoals)
{}


void SpecificWorker::jm_setSyncVelocity(const QString &server, const MotorGoalVelocityList &listGoals)
{}


MotorParams SpecificWorker::jm_getMotorParams(const QString &server, const std::string &motor)
{
	MotorParams mp;
	return mp;
}


MotorState SpecificWorker::jm_getMotorState(const QString &server, const std::string &motor)
{
	MotorState ms;
	return ms;
}


MotorStateMap SpecificWorker::jm_getMotorStateMap(const QString &server, const MotorList &mList)
{
	MotorStateMap msm;
	return msm;
}


void SpecificWorker::jm_getAllMotorState(const QString &server, MotorStateMap &mstateMap)
{}


MotorParamsList SpecificWorker::jm_getAllMotorParams(const QString &server)
{
	MotorParamsList mpl;
	return mpl;
}


RoboCompJointMotor::BusParams SpecificWorker::jm_getBusParams(const QString &server)
{
	BusParams bp;
	return bp;
}


void SpecificWorker::jm_setZeroPos(const QString &server, const std::string &motor)
{}


void SpecificWorker::jm_setSyncZeroPos(const QString &server)
{}


void SpecificWorker::jm_stopAllMotors(const QString &server)
{}


void SpecificWorker::jm_stopMotor(const QString &server, const std::string &motor)
{}


void SpecificWorker::jm_releaseBrakeAllMotors(const QString &server)
{}


void SpecificWorker::jm_releaseBrakeMotor(const QString &server, const std::string &motor)
{}


void SpecificWorker::jm_enableBrakeAllMotors(const QString &server)
{}


void SpecificWorker::jm_enableBrakeMotor(const QString &server, const std::string &motor)
{}
