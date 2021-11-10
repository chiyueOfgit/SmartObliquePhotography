#pragma once

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		namespace KEYWORD
		{
			//config
			const std::string SURFACE = "SURFACE";
			const std::string POISSONRECONSTRUCTION = "PoissonReconstruction";
			const std::string OCTREE_DEPTH = "OCTREE_DEPTH";

			const std::string TEXCOORD = "TEXCOORD";
			const std::string PARAMETERIZATION = "Parameterization";

			const std::string TEXTURE = "TEXTURE";
			const std::string RAYCASTING = "RayCasting";
			const std::string RESOLUTION = "RESOLUTION";
			const std::string SURFEL_RADIUS = "SURFEL_RADIUS";
			const std::string NUM_SAMPLE = "NUM_SAMPLE";
			const std::string SEARCH_RADIUS = "SEARCH_RADIUS";
			const std::string DISTANCE_THRESHOLD = "DISTANCE_THRESHOLD";
			const std::string WEIGHT_COEFFICIENT = "WEIGHT_COEFFICIENT";

			const std::string SUTURE = "SUTURE";
			const std::string BASICSUTURE = "BasicSuture";

			//product sig
			const std::string POISSON_RECONSTRUCTOR = "POISSON_RECONSTRUCTOR";
			const std::string RAYCASTING_TEXTUREBAKER = "RAYCASTING_TEXTUREBAKER";
			const std::string ARAP_MESH_PARAMETERIZATION = "ARAP_MESH_PARAMETERIZATION";
			const std::string BASIC_MESH_SUTURE = "BASIC_MESH_SUTURE";
			const std::string COLLAPSE_BASED_SIMPLIFICATION = "COLLAPSE_BASED_SIMPLIFICATION";
		}
	}
}
