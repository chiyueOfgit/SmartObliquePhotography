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
			void clear();

#ifdef _UNIT_TEST
			std::size_t getNumOfResultQueue()const { return m_OpResultQueue.size(); }
#endif// _UNIT_TEST 
			void recordResult(IOpResult* vResult) { _ASSERTE(vResult);  m_OpResultQueue.push_back(vResult); }

		private:
			std::deque<IOpResult*> m_OpResultQueue;
		};
	}
}

