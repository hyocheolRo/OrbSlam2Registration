#pragma once
#include <ofMain.h>
#include "ofxDepthDevice.hpp"
#include "orb/System.h"

class ofxORB_SLAM2 {
	string TAG;
protected:
	shared_ptr<ORB_SLAM2::System> ORB_SLAM2;

public:
	ofxORB_SLAM2(string binaryVocFile, shared_ptr<ofxDepthDevice> depthDevice);
	shared_ptr<ORB_SLAM2::System> getSLAM() { return ORB_SLAM2; }

	// Process the given rgbd frame. Depthmap must be registered to the RGB frame.
	// Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
	// Input depthmap: Float (CV_32F).
	// Returns the camera pose (empty if tracking fails).
	cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);
	cv::Mat TrackRGBD();
};