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
			CPointLabelChangeRecord(const std::vector<SPointLabelChange>& vChangeRecord, bool vClusterFlag) : m_ChangeRecord(vChangeRecord), m_bIsClusterChanged(vClusterFlag) {}
			~CPointLabelChangeRecord() = default;

			bool undoV();

		private:
			std::vector<SPointLabelChange> m_ChangeRecord;
			bool m_bIsClusterChanged;
		};
	}
}

