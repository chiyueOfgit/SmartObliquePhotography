#pragma once
#include "MeshSaver.h"
#include "Mesh.h"

namespace hiveObliquePhotography
{
	class CMeshOBJSaver : public IMeshSaver
	{
		void saveDataToFileV(const CMesh& vMesh, const std::string& vFilePath) override;
	};
}