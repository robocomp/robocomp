module RoboCompMSKRGBD{

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
		byte R;
		byte G;
		byte B;
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
        SkeletonPoint Position;
       	BoneOrientations boneOrien;
        FrameEdges ClippedEdges; 
	int TrackingId;
	ColorImagePoint spineJointColor;
    };
	
	dictionary<int,TPerson> PersonList;

	sequence <byte> TImg;  
	sequence <short> TDepth;



	// Not implemented!

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
		//void  getRGBD(out TRGBD imgRGBD); // ???
    };
	
	// 
	

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
		Vector3DF translation;
		//Eyes position in the face image
		Eye leftEye; 
		Eye rightEye;
		Shape3D Shape3DFace;
		animationUnitCoefficients animationCoefficients;
		projected3DShape projected3DShapeMap;
		triangles faceTriangles;

		int left;
		int right;
		int top;
		int bottom;
		int height;
		int width;		// Size of the face image
		// Identifier
		int identifier;
		bool invalid;

		float yaw;
		float pitch;
		float roll; // Orientation of the face image 

		//Face image (portion of the input image where the face is located)
		RoboCompMSKRGBD::TRGBImage faceImage;       
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
