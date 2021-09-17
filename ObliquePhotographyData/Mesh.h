#pragma once
#include <pcl/TextureMesh.h>
#include "ObliquePhotographyDataExport.h"

namespace hiveObliquePhotography
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

		Eigen::Vector3f xyz() const { return { x, y, z }; }
		Eigen::Vector3f normal() const { return { nx, ny, nz }; }
		Eigen::Vector2f uv() const { return { u, v }; }

		DataType operator[](int i) const
		{
			if (i >= 0 && i < sizeof(SVertex) / sizeof(DataType))
				return *(reinterpret_cast<const DataType*>(this) + i);
			else
				return ;
		}
	};

	struct SFace
	{
		IndexType a;
		IndexType b;
		IndexType c;

		IndexType operator[](int i) const
		{
			if (i >= 0 && i < 3)
				return *(reinterpret_cast<const IndexType*>(this) + i);
			else
				return -1;
		}
	};

	class OPDATA_DECLSPEC CMesh
	{
	public:
		CMesh() = default;
		CMesh(const pcl::PolygonMesh& vPolMesh);
		CMesh(const pcl::TextureMesh& vTexMesh);

		pcl::PolygonMesh toPolMesh() const;
		pcl::TextureMesh toTexMesh(const pcl::TexMaterial& vMaterial) const;

		std::vector<SFace> findFacesByVertex(IndexType vVertex) const;
		std::pair<Eigen::Vector3f, Eigen::Vector3f> calcAABB() const;

		std::vector<SVertex> m_Vertices;
		std::vector<SFace> m_Faces;

	private:
		void __fillVertices(std::vector<SVertex>& vVertices, const pcl::PolygonMesh& vPolMesh) const;
		void __fillVertices(std::vector<SVertex>& vVertices, const pcl::TextureMesh& vTexMesh) const;
		std::map<std::uint32_t, std::uint32_t> __getOffsetTable(const std::vector<pcl::PCLPointField>& vVertexAttributes) const;
		void __copyAttributes(std::vector<SVertex>& vVertices, const std::vector<uint8_t>& vData, const std::map<uint32_t, uint32_t>& vOffsetTable, int vPointStep) const;

		void __fillFaces(std::vector<SFace>& vFaces, const std::vector<pcl::Vertices>& vFaceData) const;

		void __fillCloud(const std::vector<SVertex>& vVertices, pcl::PCLPointCloud2& vCloud) const;
		void __fillPolygons(const std::vector<SFace>& vFaces, std::vector<pcl::Vertices>& vPolygons) const;
	};
}
