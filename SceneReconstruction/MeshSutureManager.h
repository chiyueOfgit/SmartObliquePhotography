#pragma once
#include "BasicMeshSuture.h"

using namespace std;

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct TileInfo
		{
			string Name;
			double CentrolX;
			double CentrolY;
			double Width;
			double Height;
		};

		class CMeshSutureManager :public hiveDesignPattern::CSingleton<CMeshSutureManager>
		{
		public:
			~CMeshSutureManager() = default;
			bool calTileInfo(const PointCloud_t::Ptr vTilePtr, const std::string vName);
			bool calSutureSequence(vector<pair<string, string>> voSutureNames);

		private:
			CMeshSutureManager()
			{
				m_MinCentrolX = DBL_MAX;
				m_MaxCentrolX = 0.0;
				m_MinCentrolY = DBL_MAX;
				m_MaxCentrolY = 0.0;
				m_AvgWidth = 0.0;
				m_AvgHeight = 0.0;
			};

			bool __calAvgTileSize();

			vector<TileInfo*> m_TileInfoSet;
			vector<vector<string>> m_TileAdjacency;

			double m_MinCentrolX;
			double m_MaxCentrolX;
			double m_MinCentrolY;
			double m_MaxCentrolY;
			double m_AvgWidth;
			double m_AvgHeight;

			const double m_ErrorRate = 0.1;

			friend class hiveDesignPattern::CSingleton<CMeshSutureManager>;
		};
	}
}