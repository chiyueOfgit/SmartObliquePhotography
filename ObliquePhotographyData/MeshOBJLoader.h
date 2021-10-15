#pragma once
#include "MeshLoader.h"

namespace hiveObliquePhotography
{
	class CMeshOBJLoader : public IMeshLoader
	{
	private:
		int __loadDataFromFileV(const std::string& vFileName, CMesh& voMesh) override;
	};
}