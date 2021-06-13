#pragma once
#include "OpResult.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointLabelChangeRecord : public IOpResult
		{
		public:
			CPointLabelChangeRecord() = delete;
			CPointLabelChangeRecord(const std::vector<SPointLabelChange>& vChangeRecord) : m_ChangeRecord(vChangeRecord) {}
			~CPointLabelChangeRecord() = default;

			bool undoV();

		private:
			std::vector<SPointLabelChange> m_ChangeRecord;
			//bool bIsClusterChanged;
		};
	}
}

