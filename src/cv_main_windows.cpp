#include "PlantDetect.h"

static void help()
{
	printf("\nThis sample demonstrates rowDetect\n"
		"Call:\n"
		"    /.rowDetect [image_name -- Default is fruits.jpg]\n\n");
}

const char* keys =
{
	"{help h||}{@image |fruits.jpg|input image name}"
};

int main(int argc, const char** argv)
{	
	Mat currentFrame;

	help();
	CommandLineParser parser(argc, argv, keys);
	string filename = parser.get<string>(0);

	// Read in image as BGR
	currentFrame = imread(samples::findFile(filename), IMREAD_COLOR);
	if (currentFrame.empty())
	{
		printf("Cannot read image file: %s\n", filename.c_str());
		help();
		return -1;
	}

	// Init windows for testing
	if (!InitTestWindows())
	{
		printf("Error initializing test windows\n");
		return -1;
	}

	for (int frameNum = 0; true ;frameNum++)
	{
		// Do processing on this frame
		if (!ProcessFrame(currentFrame))
		{
			printf("Error processing frame: %d\n", frameNum);
			return -1;
		}

		char key = (char)waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}
	return 0;
}