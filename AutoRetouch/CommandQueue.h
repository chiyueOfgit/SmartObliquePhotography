#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class ICommand;

		class CCommandQueue
		{
		public:
			CCommandQueue() = default;
			~CCommandQueue();

			bool undo();
	
		private:
			std::deque<ICommand*> m_CommandQueue;
		};
	}
}

