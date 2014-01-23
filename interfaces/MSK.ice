module RobocompMSKRGBD{

	sequence<byte> TRGB;
	sequence<short> TDepth;

	struct TRGBImage{
		TRGB image;
		int height;
		int width;
	};

	struct TDepthImage{
		TDepth image;
		int height;
		int width;
	};

	interface MSKRGBDEvent{
		
		void newRGBImageAvailable(TRGBImage RGBImage);
		void newDepthImageAvailable(TDepthImage depthImage);
		
	};
	interface MSKRGBD{
		
		void getRGBImage(out TRGBImage RGBImage);
		void getDepthImage(out TDepthImage depthImage);
		
	};
};

module RoboCompMSKBody
{
	

	struct Vector4
	{
	   float W;
	   float X;
	   float Y;
	   float Z; 
	};

	struct Matrix4
    {
       float M11;
       float M12;
       float M13;
       float M14;
       float M21; 
       float M22;
       float M23; 
       float M24; 
       float M31; 
       float M32; 
       float M33; 
       float M34;
       float M41; 
       float M42; 
       float M43;
       float M44;
	
	};

	struct BoneRotation
    {
       Matrix4 Matrix;
       Vector4 Quaternion;
    };

	enum JointType
    {
        HipCenter,
        Spine,
        ShoulderCenter ,
        Head ,
        ShoulderLeft ,
        ElbowLeft,
        WristLeft ,
        HandLeft ,
        ShoulderRight,
        ElbowRight,
        WristRight ,
        HandRight ,
        HipLeft ,
        KneeLeft,
        AnkleLeft ,
        FootLeft,
        HipRight ,
        KneeRight ,
        AnkleRight ,
        FootRight 
    };

	struct BoneOrientation
    {
        BoneRotation AbsoluteRotation;
        JointType EndJoint;
        BoneRotation HierarchicalRotation;
        JointType StartJoint;
    };

	sequence <BoneOrientation> BoneOrientations;

	enum FrameEdges{None, Right , Left , Top ,  Bottom };

	enum stateType{NoTracking, PositionOnly, Tracking};
	enum JointTrackingState {NotTracked,Inferred, Tracked};

	struct SkeletonPoint
	{
		float X;
		float Y;
		float Z;
	};

	struct DepthImagePoint
	{
		int X;
		int Y;
		//distance in milimetres
		int Depth;
	};

	struct ColorImagePoint
	{
		int X;
		int Y;
	};

	struct Joint
	{
		JointTrackingState state;
		SkeletonPoint Position;
	};

	dictionary<JointType, Joint> JointList;

	struct TPerson
	{
        JointList joints;
		stateType state;
		int TrackingId;
        SkeletonPoint Position;
       	BoneOrientations boneOrien;
        FrameEdges ClippedEdges; 
    };
	
	dictionary<int,TPerson> PersonList;

	sequence <byte> TImg;  
	sequence <short> TDepth;

/*	struct TRGBD
	{
	TImg r;
	TImg g;
	TImg b;
	//TShort d;
	short d;
	};

	struct TRGB
	{
	TImg r;
	TImg g;
	TImg b;	
	};
*/

	interface MSKBody
	{
			//basic		
		void getUserList( out PersonList personListIn);
		void getRTMatrixList(int id, out JointList jointListIn );
		stateType getUserState(int id);
		int getNumUsers();
		void getPerson(int TrackingId, out TPerson person);
		

			//useful
		void  getJointPixelPosition(int id, JointType  nameJoint, out ColorImagePoint point);
		
		//Mapping functions
		void  colorImagePointToSkeletonPoint(ColorImagePoint point,  out SkeletonPoint point3D);
		void  colorImagePointToDepthImagePoint (ColorImagePoint point, out DepthImagePoint depthPoint);
		void  skeletonPointToColorImagePoint(SkeletonPoint point3D, out ColorImagePoint point);
		void  skeletonPointToDepthImagePoint (SkeletonPoint point3D, out DepthImagePoint depthPoint);
		void  depthImagePointToSkeletonPoint(DepthImagePoint depthPoint, out SkeletonPoint point3D);
		void  depthImagePointToColorImagePoint(DepthImagePoint depthPoint, out ColorImagePoint point);
		


		//void  getRGB(out TRGB imgRGB); //?? un pixel?
		void  getDepth(out TDepth imgDepth);
	//	void  getRGBD(out TRGBD imgRGBD); // ???
    };

	

	interface MSKBodyEvent
	{
		void newMSKBodyEvent(PersonList people,long timestamp);
    };
};


module RoboCompMSKFace
{

	struct Vector3DF
	{
		float X ;
		float Y;
		float Z;

	}; 

/*	enum FeaturePoint
    {
        TopSkull = 0,
        TopRightForehead = 1,
        MiddleTopDipUpperLip = 7,
        AboveChin = 9,
        BottomOfChin = 10,
        RightOfRightEyebrow = 15,
        MiddleTopOfRightEyebrow = 16,
        LeftOfRightEyebrow = 17,
        MiddleBottomOfRightEyebrow = 18,
        AboveMidUpperRightEyelid = 19,
        OuterCornerOfRightEye = 20,
        MiddleTopRightEyelid = 21,
        MiddleBottomRightEyelid = 22,
        InnerCornerRightEye = 23,
        UnderMidBottomRightEyelid = 24,
        RightSideOfChin = 30,
        OutsideRightCornerMouth = 31,
        RightOfChin = 32,
        RightTopDipUpperLip = 33,
        TopLeftForehead = 34,
        MiddleTopLowerLip = 40,
        MiddleBottomLowerLip = 41,
        LeftOfLeftEyebrow = 48,
        MiddleTopOfLeftEyebrow = 49,
        RightOfLeftEyebrow = 50,
        MiddleBottomOfLeftEyebrow = 51,
        AboveMidUpperLeftEyelid = 52,
        OuterCornerOfLeftEye = 53,
        MiddleTopLeftEyelid = 54,
        MiddleBottomLeftEyelid = 55,
        InnerCornerLeftEye = 56,
        UnderMidBottomLeftEyelid = 57,
        LeftSideOfCheek = 63,
        OutsideLeftCornerMouth = 64,
        LeftOfChin = 65,
        LeftTopDipUpperLip = 66,
        OuterTopRightPupil = 67,
        OuterBottomRightPupil = 68,
        OuterTopLeftPupil = 69,
        OuterBottomLeftPupil = 70,
        InnerTopRightPupil = 71,
        InnerBottomRightPupil = 72,
        InnerTopLeftPupil = 73,
        InnerBottomLeftPupil = 74,
        RightTopUpperLip = 79,
        LeftTopUpperLip = 80,
        RightBottomUpperLip = 81,
        LeftBottomUpperLip = 82,
        RightTopLowerLip = 83,
        LeftTopLowerLip = 84,
        RightBottomLowerLip = 85,
        LeftBottomLowerLip = 86,
        MiddleBottomUpperLip = 87,
        LeftCornerMouth = 88,
        RightCornerMouth = 89,
        BottomOfRightCheek = 90,
        BottomOfLeftCheek = 91,
        AboveThreeFourthRightEyelid = 95,
        AboveThreeFourthLeftEyelid = 96,
        ThreeFourthTopRightEyelid = 97,
        ThreeFourthTopLeftEyelid = 98,
        ThreeFourthBottomRightEyelid = 99,
        ThreeFourthBottomLeftEyelid = 100,
        BelowThreeFourthRightEyelid = 101,
        BelowThreeFourthLeftEyelid = 102,
        AboveOneFourthRightEyelid = 103,
        AboveOneFourthLeftEyelid = 104,
        OneFourthTopRightEyelid = 105,
        OneFourthTopLeftEyelid = 106,
        OneFourthBottomRightEyelid = 107,
        OneFourthBottomLeftEyelid = 108,
    };*/

	enum FeaturePoint
    {
        TopSkull,
        TopRightForehead ,
		FP2,
		FP3,
		FP4,
		FP5,
		FP6,
        MiddleTopDipUpperLip ,
        F8,
		AboveChin,
        BottomOfChin ,
		F11,
		FP12,
		FP13,
		FP14,
        RightOfRightEyebrow ,
        MiddleTopOfRightEyebrow ,
        LeftOfRightEyebrow ,
        MiddleBottomOfRightEyebrow ,
        AboveMidUpperRightEyelid ,
        OuterCornerOfRightEye ,
        MiddleTopRightEyelid ,
        MiddleBottomRightEyelid ,
        InnerCornerRightEye ,
        UnderMidBottomRightEyelid ,
        FP25,
		FP26,
		FP27,
		FP28,
		FP29,
		RightSideOfChin ,
        OutsideRightCornerMouth ,
        RightOfChin ,
        RightTopDipUpperLip ,
        TopLeftForehead ,
		FP35,
		FP36,
		FP37,
		FP38,
		FP39,
        MiddleTopLowerLip ,
        MiddleBottomLowerLip ,
		FP42,
		FP43,
		FP44,
		FP45,
		FP46,
		FP47,
        LeftOfLeftEyebrow ,
        MiddleTopOfLeftEyebrow ,
        RightOfLeftEyebrow ,
        MiddleBottomOfLeftEyebrow ,
        AboveMidUpperLeftEyelid ,
        OuterCornerOfLeftEye ,
        MiddleTopLeftEyelid ,
        MiddleBottomLeftEyelid ,
        InnerCornerLeftEye ,
        UnderMidBottomLeftEyelid ,
			FP58,
			FP59,
			FP60,
			FP61,
			FP62,
        LeftSideOfCheek ,
        OutsideLeftCornerMouth ,
        LeftOfChin ,
        LeftTopDipUpperLip ,
        OuterTopRightPupil ,
        OuterBottomRightPupil ,
        OuterTopLeftPupil ,
        OuterBottomLeftPupil ,
        InnerTopRightPupil ,
        InnerBottomRightPupil ,
        InnerTopLeftPupil ,
        InnerBottomLeftPupil ,
			FP75,
			FP76,
			FP77,
			FP78,
        RightTopUpperLip ,
        LeftTopUpperLip ,
        RightBottomUpperLip ,
        LeftBottomUpperLip ,
        RightTopLowerLip ,
        LeftTopLowerLip ,
        RightBottomLowerLip ,
        LeftBottomLowerLip ,
        MiddleBottomUpperLip ,
        LeftCornerMouth ,
        RightCornerMouth ,
        BottomOfRightCheek ,
        BottomOfLeftCheek ,
			FP92,
			FP93,
			FP94,
        AboveThreeFourthRightEyelid ,
        AboveThreeFourthLeftEyelid ,
        ThreeFourthTopRightEyelid ,
        ThreeFourthTopLeftEyelid ,
        ThreeFourthBottomRightEyelid ,
        ThreeFourthBottomLeftEyelid ,
        BelowThreeFourthRightEyelid ,
        BelowThreeFourthLeftEyelid ,
        AboveOneFourthRightEyelid ,
        AboveOneFourthLeftEyelid,
        OneFourthTopRightEyelid ,
        OneFourthTopLeftEyelid ,
        OneFourthBottomRightEyelid ,
        OneFourthBottomLeftEyelid,
			FP109,
			FP110,
			FP111,
			FP112,
			FP113,
			FP114,
			FP115,
			FP116,
			FP117,
			FP118,
			FP119,
			FP120

	};

	enum AnimationUnit
    {
        LipRaiser ,
        JawLower ,
        LipStretcher,
        BrowLower ,
        LipCornerDepressor ,
        BrowRaiser,
    };

	struct PointF
	{
		float X;
		float Y;
	};

	struct FaceTriangle
	{
		int First;
		int Second;
		int Third;
	};
	
      struct Eye
	{
		float x;
		float y; // Eye position on the image
	};

	sequence <byte> RGBImage;

	//  faceState - Is there a detailedFace on the scene?
	enum DetailedFaceState {isFace, noFace}; 

	dictionary<FeaturePoint, Vector3DF> Shape3D;

	dictionary<AnimationUnit, float> animationUnitCoefficients;
	dictionary<FeaturePoint, PointF> projected3DShape;
	
	sequence <FaceTriangle> triangles;

	// DetailedFace - structure of the detailedFace (face and eyes data, identifier)
	struct DetailedFace
	{
		//Face image (portion of the input image where the face is located)
		RobocompMSKRGBD::TRGBImage faceImage;
		int left;
		int right;
		int top;
		int bottom;
		int height;
		int width;		// Size of the face image
		float yaw;
		float pitch;
		float roll; // Orientation of the face image 

       Vector3DF translation;

		//Eyes position in the face image
		Eye leftEye; Eye rightEye;
		// Identifier
		int identifier;
		bool invalid;
		Shape3D Shape3DFace;
		animationUnitCoefficients animationCoefficients;
		projected3DShape projected3DShapeMap;
		triangles faceTriangles;

	};

	// FaceMap - A map storing all faces in the scene
	dictionary<int, DetailedFace> DetailedFaceMap;
	
	interface MSKFaceEvent{
		void newFaceAvailable(DetailedFaceMap face,long timestamp);
	};
	interface MSKFace{
		void getFaces(out DetailedFaceMap faces);
	};
};

module RoboCompMSKASR
{
	sequence<string> WordsRecognized;
	// TSentence - Frase reconocida por el ASR

	struct TSentence
	{	
		WordsRecognized words;
		string grammarUsed;
		bool blockingCall;
		int acquisitionHour;
		int acquisitionSecs;
		int acquisitionDay;
	};

	interface MSKASR
	{
		void newSentenceAvailable(TSentence sentence);
	};
};

module RoboCompMSKHand
{
	enum InteractionHandEventType{NoneEvent ,Grip, GripRelease};
	enum InteractionHandType{None, Left, Right};

	//Hand information
	struct InteractionHandPointer
	{

	InteractionHandEventType HandEventType;
        InteractionHandType HandType;
        bool IsActive ;
        bool IsInteractive ;
        bool IsPressed ;
        bool IsPrimaryForUser ;
        bool IsTracked ;
        double PressExtent ;
        double RawX ;
        double RawY ;
        double RawZ ;
        double X ;
        double Y;
	};

	sequence <InteractionHandPointer> InteractionHandPointers;

	//Hands user information
	struct UserInfo
	{
		InteractionHandPointers HandPointers;
		int SkeletonTrackingId;
	};

	//Output of the InteractionStream,Hands information of all the users in the scene
	dictionary <int, UserInfo> UsersInfo;


	interface MSKHand{
		void newInteractionEvent(UsersInfo usersInf,long timestamp);
	};
};


  

