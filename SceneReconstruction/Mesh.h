#pragma once
#include <pcl/TextureMesh.h>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		using DataType = float;
		using IndexType = uint32_t;
		struct SVertex
		{
			DataType x;
			DataType y;
			DataType z;
			DataType nx;
			DataType ny;
			DataType nz;
			DataType u;
			DataType v;
		};

		struct SFace
		{
			IndexType a;
			IndexType b;
			IndexType c;
		};

		class CMesh
		{
		public:
			CMesh() = default;
			CMesh(const pcl::PolygonMesh& vPolMesh);
			CMesh(const pcl::TextureMesh& vTexMesh);

			static pcl::PolygonMesh toPolMesh(const CMesh& vMesh);
			static pcl::TextureMesh toTexMesh(const CMesh& vMesh);

			std::vector<SVertex> m_Vertices;
			std::vector<SFace> m_Faces;

		private:
			void __fillVertices(std::vector<SVertex>& vVertices, const pcl::PolygonMesh& vPolMesh);
			void __fillVertices(std::vector<SVertex>& vVertices, const pcl::TextureMesh& vTexMesh);
		};
	}
}
