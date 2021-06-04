#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IOpResult;

		class COpResultQueue
		{
		public:
			COpResultQueue() = default;
			~COpResultQueue() = default;

			bool undo();
			
			void recordResult(IOpResult* vResult) { _ASSERTE(vResult);  m_OpResultQueue.push_back(vResult); }

		private:
			std::deque<IOpResult*> m_OpResultQueue;
		};
	}
}

