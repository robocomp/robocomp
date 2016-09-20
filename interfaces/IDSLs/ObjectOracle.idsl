import "RGBD.idsl";

module RoboCompObjectOracle
{
	struct Label
	{
		string name;
		float believe;
	};

	sequence<Label> ResultList;
	
	interface ObjectOracle
	{
		void getLabelsFromImage(RoboCompRGBD::ColorSeq image, out ResultList result);
		void semanticDistance(string word1, string word2, out float result);
	};
};
