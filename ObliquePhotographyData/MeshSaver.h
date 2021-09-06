#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	class IMeshSaver : public hiveDesignPattern::IProduct
	{
	public:
		IMeshSaver() = default;
		~IMeshSaver() override = default;

		virtual void saveDataToFileV(const CMesh& vMesh, const std::string& vFilePath) = 0;

	private:
	};
}