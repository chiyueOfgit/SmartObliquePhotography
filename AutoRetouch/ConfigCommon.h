#pragma once

namespace KEY_WORDS
{
	constexpr char RESOLUTION[] = "RESOLUTION";
	constexpr char CLUSTERTOLERANCE[] = "CLUSTERTOLERANCE";
	constexpr char MINCLUSTERSIZE[] = "MINCLUSTERSIZE";
	constexpr char MAXCLUSTERSIZE[] = "MAXCLUSTERSIZE";

	//growing parameters
	constexpr char ENABLE_COLOR_TEST[] = "ENABLE_COLOR_TEST";
	constexpr char COLOR_TEST_MODE[] = "COLOR_TEST_MODE";
	constexpr char COLOR_TEST_THRESHOLD[] = "COLOR_TEST_THRESHOLD";
	constexpr char ENABLE_GROUND_TEST[] = "ENABLE_GROUND_TEST";
	constexpr char GROUND_TEST_THRESHOLD[] = "GROUND_TEST_THRESHOLD";
	constexpr char ENABLE_NORMAL_TEST[] = "ENABLE_NORMAL_TEST";
	constexpr char SEARCH_RADIUS[] = "SEARCH_RADIUS";

	constexpr char POINT_SHOW_SIZE[] = "POINT_SHOW_SIZE";
	constexpr char OUTLIER_MEAN_KNN_NUMBER[] = "OUTLIER_MEAN_KNN_NUMBER";
	constexpr char OUTLIER_STD_MULTIPLE_THRESHOLD[] = "OUTLIER_STD_MULTIPLE_THRESHOLD";

	//binary parameters
	constexpr char EXCUTEAREA_EXPAND_RATIO[] = "EXCUTEAREA_EXPAND_RATIO";
	constexpr char BINARY_CLASSIFIER_NORMAL_RATIO_THRESHOLD[] = "BINARY_CLASSIFIER_NORMAL_RATIO_THRESHOLD";

	//combine binary paramerters
	constexpr char ENABLE_VFH[] = "ENABLE_VFH";
	constexpr char ENABLE_SCORE[] = "ENABLE_SCORE";
	constexpr char ENABLE_NORMAL[] = "ENABLE_NORMAL";
	constexpr char VFH_WEIGHT[] = "VFH_WEIGHT";
	constexpr char SCORE_WEIGHT[] = "SCORE_WEIGHT";
	constexpr char NORMAL_WEIGHT[] = "NORMAL_WEIGHT";
	constexpr char EXPECT_SCORE[] = "EXPECT_SCORE";

}
