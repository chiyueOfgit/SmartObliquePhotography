#pragma once

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		namespace KEYWORD
		{
			//config
			const std::string SURFACE = "SURFACE";
			const std::string OCTREE_DEPTH = "OCTREE_DEPTH";

			const std::string TEXCOORD = "TEXCOORD";

			const std::string TEXTURE = "TEXTURE";
			const std::string RESOLUTION = "RESOLUTION";
			const std::string SURFEL_RADIUS = "SURFEL_RADIUS";
			const std::string NUM_SAMPLE = "NUM_SAMPLE";

			//product sig
			const std::string POISSON_RECONSTRUCTOR = "POISSON_RECONSTRUCTOR";
			const std::string RAYCASTING_TEXTUREBAKER = "RAYCASTING_TEXTUREBAKER";
			const std::string ARAP_MESH_PARAMETERIZATION = "ARAP_MESH_PARAMETERIZATION";
			const std::string BASIC_MESH_SUTURE = "BASIC_MESH_SUTURE";
		}
	}
}
