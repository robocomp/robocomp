#ifndef ACTIVITYRECOGNITION_ICE
#define ACTIVITYRECOGNITION_ICE

module RoboCompActivityRecognition
{
        sequence<float> FrameJoints;

        interface ActivityRecognition
        {
                bool addSkeleton(FrameJoints skeleton);
                idempotent string getCurrentActivity();  
        };
};

#endif

