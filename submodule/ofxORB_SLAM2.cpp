#include "ofxORB_SLAM2.h"

static const string CameraParameters =
R"foo(
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: %f
Camera.fy: %f
Camera.cx: %f
Camera.cy: %f

Camera.k1: %f
Camera.k2: %f
Camera.p1: %f
Camera.p2: %f
Camera.k3: %f

Camera.width: %d
Camera.height: %d

# Camera frames per second 
Camera.fps: 30.0

# IR projector baseline times fx (aprox.)
Camera.bf: %f    #fx(pixel) * baseline(mm)

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 0

# Close/Far threshold. Baseline times.
ThDepth: %f    #mThDepth = mbf*(float)fSettings["ThDepth"] / fx;

# Deptmap values factor 
DepthMapFactor: 1.0
)foo";

static const string ORBParameters =
R"foo(
#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 50
ORBextractor.minThFAST: 7
)foo";

static const string ViewerParameters =
R"foo(
#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize:2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
)foo";

static vector<ofPoint> all_color_points;

ofxORB_SLAM2::ofxORB_SLAM2(string binaryVocFile, shared_ptr<ofxDepthDevice> depthDevice) {
	auto calib = depthDevice->getColorCalibrationResult();
	if (!calib.isLoaded()) {
		REPORT_ERROR("ofxORB_SLAM2", "ColorCalibrationResult empty");
		return;
	}

	char buff[1500];
	sprintf(buff, CameraParameters.c_str(),
		calib.fx,
		calib.fy,
		calib.cx,
		calib.cy,
		calib.distCoeffs.at<double>(0),
		calib.distCoeffs.at<double>(1),
		calib.distCoeffs.at<double>(2),
		calib.distCoeffs.at<double>(3),
		calib.distCoeffs.at<double>(4),
		calib.resolutionX,
		calib.resolutionY,
		calib.fx * 40, //baseline: can the distance between ir and color cameras be a valid baseline?
		4500.f / 40	//max depth divided by baseline
		);
	string parameters = string(R"foo(%YAML:1.0)foo") + buff;
	parameters += ORBParameters;
	parameters += ViewerParameters;

	cout << endl << parameters << endl << endl;

	cv::FileStorage fSettings(parameters, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
	ORB_SLAM2 = make_shared<ORB_SLAM2::System>(binaryVocFile, fSettings, ORB_SLAM2::System::RGBD, depthDevice);

	if (all_color_points.size() == 0) {
		all_color_points.resize(depthDevice->getColorWidth() * depthDevice->getColorHeight());
		for (size_t y = 0; y < depthDevice->getColorHeight(); y++)
			for (size_t x = 0; x < depthDevice->getColorWidth(); x++)
				all_color_points[y*depthDevice->getColorWidth() + x] = ofPoint(x, y);
	}
}

cv::Mat ofxORB_SLAM2::TrackRGBD(const cv::Mat & im, const cv::Mat & depthmap, const double & timestamp)
{
	return ORB_SLAM2->TrackRGBD(im, depthmap, timestamp);
}

cv::Mat ofxORB_SLAM2::TrackRGBD() {
	auto depthDevice = ORB_SLAM2->GetDepthDevice();
	depthDevice->update(); //not really necessary
	auto color_mat = depthDevice->getColorMat();
	auto depth_mat = depthDevice->getDepthShortMat();
	auto colorToDepthMapping = depthDevice->colorToDepth(all_color_points);

	int COLOR_WIDTH = depthDevice->getColorWidth(), COLOR_HEIGHT = depthDevice->getColorHeight();
	int DEPTH_WIDTH = depthDevice->getDepthWidth(), DEPTH_HEIGHT = depthDevice->getDepthHeight();

	Mat depth_float = Mat::zeros(COLOR_HEIGHT, COLOR_WIDTH, CV_32F);
	for (size_t y = 0; y < COLOR_HEIGHT; y++) {
		for (size_t x = 0; x < COLOR_WIDTH; x++) {
			int color_idx = y*COLOR_WIDTH + x;
			ofPoint depth_pt = colorToDepthMapping[color_idx];
			bool not_ok =
				std::isinf(depth_pt.x) || std::isinf(depth_pt.y)
				|| isnan(depth_pt.x) || isnan(depth_pt.y);
			if (!not_ok) {
				int depth_idx = depth_pt.y * DEPTH_WIDTH + depth_pt.x;
				depth_float.at<float>(color_idx) = depth_mat.at<ushort>(depth_idx); // to meter unit?
			}
		}
	}

	return TrackRGBD(color_mat.clone(), depth_float.clone(), ofGetElapsedTimef());
}