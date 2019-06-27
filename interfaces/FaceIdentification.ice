
#ifndef FACEIDENTIFICATION_ICE
#define FACEIDENTIFICATION_ICE

module RoboCompFaceIdentification
{
	sequence<byte> ImgType;

	struct TImage
	{
		int width;
		int height;
		int depth;
		ImgType image;
	};

	sequence<TImage> FaceImages;

	sequence<string> FaceLabels;

	interface FaceIdentification
	{
		idempotent void addNewFace(FaceImages faceImg);
		idempotent void getFaceLabels(FaceImages faces, out FaceLabels faceNames);  
		idempotent void deleteLabel(string faceLabel);
	};
};

#endif
