#ifndef ACTIVITYRECOGNITION_ICE
#define ACTIVITYRECOGNITION_ICE

module RoboCompActivityRecognition
{
        sequence<float> Joint;
        sequence<Joint> Skeleton3D;

        interface ActivityRecognition
        {
                bool addSkeleton(Skeleton3D skeleton);
                idempotent string getCurrentActivity();  
        };
};

#endif

