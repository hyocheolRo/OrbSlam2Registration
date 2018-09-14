#include"PointCloudRegistration.hpp"
#include "submodule/ofxORB_SLAM2.h"

class OrbSlam2Registration : public PointCloudRegistration {
	const string TAG = typeid(*this).name();
protected:
	shared_ptr<ofxORB_SLAM2> orb_slam2;
public:
	OrbSlam2Registration(AlgorithmOption _option) : PointCloudRegistration(_option) {
		orb_slam2 = make_shared<ofxORB_SLAM2>("data/ORBvoc.bin", this->kinect2);
		//orb_slam2 = make_shared<ofxORB_SLAM2>("data/ORBvoc.txt", this->kinect2);
	}

	string getAlgorithmName() {
		string algo_name = string(TAG).replace(TAG.find("class "), string("class ").length(), "");
		switch (option) {
		case default: algo_name += " (Default)";
			break;
		case option1: algo_name += " (Iterative Closest)";
			break;
		case option2:
		case option3:
		case option4:
		default: algo_name += " (Undefined)";
			break;
		}
		return algo_name;
	}

	void registerScenes(AlgorithmConfig* config = nullptr) {
		for (int i = SCENE_INDEX_FROM; i < min((int)multi_scene_data.size() - 1, SCENE_INDEX_TO); i += SCENE_INDEX_STEP) {
			int scene1Idx = i, scene2Idx = i + SCENE_INDEX_STEP;
			SceneData &scene1 = multi_scene_data[scene1Idx], &scene2 = multi_scene_data[scene2Idx];
			SceneInfo &info1 = multi_scene_info[scene1Idx], &info2 = multi_scene_info[scene2Idx];
			registerORB_SLAM2(scene1, scene2, info1, info2);
		}
		switch (option) {
		case default:
			break;
		case option1:
			for (int i = SCENE_INDEX_FROM; i < min((int)multi_scene_data.size() - 1, SCENE_INDEX_TO); i += SCENE_INDEX_STEP) {
				int scene1Idx = i, scene2Idx = i + SCENE_INDEX_STEP;
				SceneData &scene1 = multi_scene_data[scene1Idx], &scene2 = multi_scene_data[scene2Idx];
				SceneInfo &info1 = multi_scene_info[scene1Idx], &info2 = multi_scene_info[scene2Idx];
				refineIterativeClosest(scene1, scene2, info1, info2);
			}
			break;
		case option2:
			break;
		case option3:
			break;
		case option4:
			break;
		}
	}

protected:
	void registerORB_SLAM2(
		const SceneData &scene1, const SceneData &scene2, const SceneInfo &info1, SceneInfo &info2)
	{
		auto ColorFrameToDepthFloat = [this](const Mat& depth16UC1) -> Mat {
			int COLOR_WIDTH = kinect2->getColorWidth(), COLOR_HEIGHT = kinect2->getColorHeight();
			int DEPTH_WIDTH = kinect2->getDepthWidth(), DEPTH_HEIGHT = kinect2->getDepthHeight();

			vector<ofPoint> all_color_points;
			all_color_points.resize(COLOR_WIDTH * COLOR_HEIGHT);
			for (size_t y = 0; y < COLOR_HEIGHT; y++)
				for (size_t x = 0; x < COLOR_WIDTH; x++)
					all_color_points[y*COLOR_WIDTH + x] = ofPoint(x, y);

			auto colorToDepthMapping = colorToDepth(all_color_points, depth16UC1);
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
						depth_float.at<float>(color_idx) = depth16UC1.at<ushort>(depth_idx); // to meter unit?
					}
				}
			}
			return depth_float;
		};

		bool beginning = orb_slam2->getSLAM()->GetTracker()->mState == ORB_SLAM2::Tracking::eTrackingState::NO_IMAGES_YET;
		if (beginning) {
			orb_slam2->TrackRGBD(scene1.color, ColorFrameToDepthFloat(scene1.depth), ofGetElapsedTimef());
		}
		auto pose = orb_slam2->TrackRGBD(scene2.color, ColorFrameToDepthFloat(scene2.depth), ofGetElapsedTimef());
		info2.to_ref = ofMatrix4x4((float*)Mat(pose.t()).data);
		return;
	}

};

shared_ptr<PointCloudRegistration> PointCloudRegistration::makeOrbSlam2Registration(AlgorithmOption option) {
	return make_shared<OrbSlam2Registration>(option);
}