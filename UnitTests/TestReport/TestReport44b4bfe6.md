Report 
=================
###### 2021.07.07
###### Commit id: 44b4bfe6

#### Overview
所有测试用例都可以运行


#### Unit_Test_001 (load_point_cloud_tile)

| Test | LoadTilePly | LoadTilePcd | DeathTest_LoadInexistentTile |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_002 (load_point_cloud_scene)

| Test | LoadScene | DeathTest_LoadDuplicatedFile1 | DeathTest_LoadDuplicatedFile2 | DeathTest_LoadPartiallyIncorrectFileSet | DeathTest_LoadIncorrectFileSet |
| :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_010 (init_point_cloud_retouch)

| Test | InitPointCloudRetouchScene | DeathTest_InitSceneWithErrorPtr | InitPointLabelSet | DeathTest_InitSceneWithNegativeSize | InitRetouchTask | DeathTest_InitRetouchTaskWithErrorConfig | InitPointCloudRetouchManager |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_011 (create_initial_cluster)

| Test | GenerateHardness4EveryPoint | ComputeClusterCenter | DivideUserSpecifiedRegion | CreateInitialCluster |
| :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_012 (point_cluster)

| Test | DeathTest_Uninitialized | DeathTest_InvalidIndex | FalseProbability_Test |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_013 (point_label_set)

| Test | Uninitialized_Test | Illegal_Input_Test |
| :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_014 (point_cluster_expander)

| Test | NoRepeatIndex | EmptyInput | NullptrInput |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_015 (feature)

| Test | Color_Feature_BaseTest_1 | Color_Feature_BaseTest_2 | Color_Feature_BaseTest_3 | Color_Feature_BaseTest_4 | Plane_Feature_BaseTest_1 | Plane_Feature_BaseTest_2 |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_016 (neighborhood_builder)

| Test | Illegal_Input_Test | Symmetry_Test | Anti_Reflexive_Test |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |


#### Unit_Test_017 (selecting)

| Test | Selecting_NoThroughTest_CompleteTree | Selecting_NoThroughTest_CompleteGround | Selecting_NoThroughTest_CompleteBuilding | Selecting_MultipleObjectsTest_CompleteMoreTrees |
| :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> |

