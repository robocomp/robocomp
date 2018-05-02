
#ifndef EMOTIONRECOGNITION_ICE
#define EMOTIONRECOGNITION_ICE

module RoboCompEmotionRecognition
{
    struct SEmotion
    {
        int x;
        int y;
        int w;
        int h;
        string emotion;
    };

    sequence<SEmotion> EmotionList;

    interface EmotionRecognition
    {
        idempotent void getEmotionList(out EmotionList emotionL);
    };
};

#endif
