#pragma once
#include "AutoRetouchCommon.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class ICommand : public hiveDesignPattern::ISubject
		{
		public:
			ICommand() = default;
			~ICommand() = default;

			void undo();

		protected:
			std::vector<SPointLabelChange> m_LabelChangeRecord;
		};
	}
}