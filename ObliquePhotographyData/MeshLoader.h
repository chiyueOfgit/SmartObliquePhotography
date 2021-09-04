#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	class IMeshLoader : public hiveDesignPattern::IProduct
	{
	public:
		IMeshLoader() = default;
		~IMeshLoader() override = default;

		bool loadDataFromFile(CMesh& voMesh, const std::string& vFileName);

	private:
		virtual int __loadDataFromFileV(const std::string& vFileName, CMesh& voMesh) = 0;
	};
}