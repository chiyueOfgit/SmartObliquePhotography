#include "pch.h"
#include "AutoRetouchInterface.h"

//≤‚ ‘”√¿˝¡–±Ì£∫

TEST(Test_RegionGrowing, RegionGrow) 
{
	std::vector<std::uint64_t> SeedSet = { 1, 2 };

	hiveObliquePhotography::AutoRetouch::hiveExecuteClassifer(hiveObliquePhotography::AutoRetouch::CLASSIFIER_REGION_GROW, SeedSet, hiveObliquePhotography::AutoRetouch::EPointLabel::UNWANTED);
}
