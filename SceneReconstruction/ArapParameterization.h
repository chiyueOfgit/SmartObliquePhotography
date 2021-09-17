#pragma once
#include "MeshParameterization.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct SVertexInfo
		{
			int VertexRef;
			int HalfEdgeRef;
		};

		struct SHalfEdge
		{
			int VertexRef;
			int Prev;
			int Next;
			int Twin = -1;
		};

		class CArapParameterization : public IMeshParameterization
		{
		public:
			CArapParameterization();
			~CArapParameterization() = default;

			void buildHalfEdge();
			std::vector<int> findBoundaryPoint();
			
		private:
			std::vector<SVertexInfo> m_VertexInfoTable;
			std::vector<SHalfEdge> m_HalfEdgeTable;
		};
	}
}

