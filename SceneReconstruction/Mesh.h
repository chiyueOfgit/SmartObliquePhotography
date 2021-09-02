#pragma once
#include <pcl/TextureMesh.h>
#include "SceneREconstructionExport.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		using DataType = float;
		using IndexType = std::uint32_t;
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

		class RECONSTRUCTION_DECLSPEC CMesh
		{
		public:
			CMesh() = default;
			CMesh(const pcl::PolygonMesh& vPolMesh);
			CMesh(const pcl::TextureMesh& vTexMesh);

			pcl::PolygonMesh toPolMesh();
			pcl::TextureMesh toTexMesh();

			std::vector<SVertex> m_Vertices;
			std::vector<SFace> m_Faces;

		private:
			void __fillVertices(std::vector<SVertex>& vVertices, const pcl::PolygonMesh& vPolMesh);
			void __fillVertices(std::vector<SVertex>& vVertices, const pcl::TextureMesh& vTexMesh);
			std::map<std::uint32_t, std::uint32_t> __getOffsetTable(const std::vector<pcl::PCLPointField>& vVertexAttributes);
			void __copyAttributes(std::vector<SVertex>& vVertices, const std::vector<uint8_t>& vData, const std::map<uint32_t, uint32_t>& vOffsetTable, int vPointStep);

			void __fillFaces(std::vector<SFace>& vFaces, const std::vector<pcl::Vertices>& vFaceData);

			void __fillCloud(const std::vector<SVertex>& vVertices, pcl::PCLPointCloud2& vCloud);
			void __fillPolygons(const std::vector<SFace>& vFaces, std::vector<pcl::Vertices>& vPolygons);
		}
	}
}
