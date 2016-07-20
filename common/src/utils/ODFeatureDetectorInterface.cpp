#include "od/common/utils/ODFeatureDetectorInterface.h"

namespace od
{
	
	std::map<std::string, FeatureType> od_enum_map = {{"SIFT", SIFT}, {"SURF", SURF}, {"ORB", ORB}, {"SIFT_GPU", SIFT_GPU}, {"ORB_GPU", ORB_GPU}};

	FeatureType string2FeatureType(const std::string & name)
	{
		return od_enum_map[name];
	}

}