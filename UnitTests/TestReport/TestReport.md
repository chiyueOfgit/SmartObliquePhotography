Report 
=================
###### 2021.07.22
###### Commit id: [d8efcaf5](http://192.168.62.249:9090/tfs/STUDENT_PROJECTS/_git/SmartObliquePhotography/pushes/5937?refName=refs%2Fheads%2Fdevelop)

#### Overview
所有测试用例都可以运行


#### Unit_Test_001 (load_point_cloud_tile)

| Test | LoadTilePly | LoadTilePcd | DeathTest_LoadInexistentTile |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_002 (load_point_cloud_scene)

| Test | LoadScene | DeathTest_LoadDuplicatedFile1 | DeathTest_LoadDuplicatedFile2 | DeathTest_LoadPartiallyIncorrectFileSet | DeathTest_LoadIncorrectFileSet |
| :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_003 (save_point_cloud_scene)

| Test | SaveScene4PLY | SaveScene4PCD | Save4NonexistentFormat |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_008 (undo)

| Test | Empty_ResultQueue_Expect_Test | LabelSet_Undo_Overview_Test | Timestamp_Undo_Overview_Test | LabelSet_Undo_Cleanup_Test | Timestamp_Undo_Cleanup_Test | Empty_Input_Expect_Test |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_009 (outlier_detector)

| Test | DeathTest_InvalidInput | FunctionTest_Test1 | FunctionTest_Test2 | FunctionTest_Test3 |
| :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_010 (init_point_cloud_retouch)

| Test | InitPointCloudRetouchScene | DeathTest_InitSceneWithErrorPtr | InitPointLabelSet | DeathTest_InitSceneWithNegativeSize | InitRetouchTask | DeathTest_InitRetouchTaskWithErrorConfig | InitPointCloudRetouchManager | ResetPointCloudRetouchManager | ReInitPointCloudRetouchManager |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_011 (create_initial_cluster)

| Test | BaseTest1_Create_Cluster | DeathTest1_User_Caused_Error | DeathTest2_User_Caused_Error |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_012 (point_cluster)

| Test | DeathTest_Uninitialized | DeathTest_InvalidIndex | FalseProbability_Test |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_013 (point_label_set)

| Test | Uninitialized_Test | Illegal_Input_Test |
| :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_014 (point_cluster_expander)

| Test | NoRepeatIndex | EmptyInput | NullptrInput |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_015 (feature)

| Test | Color_Feature_BaseTest_1 | Color_Feature_BaseTest_2 | Color_Feature_BaseTest_3 | Color_Feature_BaseTest_4 | Color_Feature_BaseTest_5 | Plane_Feature_BaseTest_1 | Plane_Feature_BaseTest_2 | Normal_Feature_BaseTest_1 | Normal_Feature_BaseTest_2 | Normal_Feature_BaseTest_3 |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#F0645A>Fail</font> | <font color=#F0645A>Fail</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |  <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_016 (neighborhood_builder)

| Test | Radius_Illegal_Input_Test | Nearest_Illegal_Input_Test | Radius_Symmetry_Test |
| :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |


#### Unit_Test_017 (selecting)

| Test | Selecting_NoThroughTest_CompleteTree | Selecting_NoThroughTest_CompleteGround | Selecting_NoThroughTest_CompleteBuilding | Selecting_MultipleObjectsTest_CompleteMoreTrees | Selecting_CullingTest_KeepATree | Selecting_CullingTest_KeepGround | Selecting_CullingTest_KeepABuilding | Selecting_CullingTest_KeepMoreTrees |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| <font color=#7E858B>Result</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> | <font color=#3CC8B4>Pass</font> |

