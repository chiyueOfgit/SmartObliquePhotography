#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
//IMPORTANT: 现在这个类不是线程安全！！！
		class CPointLabel4Classfier
		{
		public:
			CPointLabel4Classfier() = default;
			~CPointLabel4Classfier() = default;

			bool dumpPointLabelChangeRecord(std::vector<SPointLabelChange>& voResult) const;
			bool reset();

			std::uint64_t getTimestamp() const { return m_Timestamp; }

			EPointLabel getPointLabel(std::uint64_t vIndex) const { _ASSERTE(vIndex < m_PointLabelSet.size()); return m_PointLabelSet[vIndex]; }

			const std::vector<EPointLabel>& getPointLabelSet() const { return m_PointLabelSet; }

		protected:
			bool _reset(const std::vector<SPointLabelChange>& vChangeRecord);

			std::vector<SPointLabelChange> m_PointLabelChangeRecord;
			std::vector<EPointLabel> m_PointLabelSet;
			std::uint64_t m_Timestamp = 0;

		private:
			virtual bool __isReady2DumpChangeRecordV() const { return true; }
			virtual bool __isReady2ResetV() const { return true; }
		};

		class CGlobalPointLabelSet;

		class CLocalPointLabelSet : public CPointLabel4Classfier
		{
		public:
			~CLocalPointLabelSet() = default;

			[[nodiscard]] bool startRecord();
			[[nodiscard]] bool stopRecord();

			bool changePointLabel(std::uint64_t vPointIndex, EPointLabel vDstLabel);
			bool isRecording() const { return m_IsRecording; }
//			bool reset(const std::vector<SPointLabelChange>& vChangeRecord);
			bool update(CGlobalPointLabelSet *vGlobalLabelSet);

		private:
			CLocalPointLabelSet() = default;  //限制只能通过CGlobalPointLabelSet::clone()才能创建CLocalPointLabelSet对象

			bool m_IsRecording = false;

			void __cleanPointLabelChangeRecord();
			void __sortPointLabelChangeRecordByIndex();

			bool __isReady2DumpChangeRecordV() const override { return !m_IsRecording; }
			bool __isReady2ResetV() const override { return !m_IsRecording; }

		friend class CGlobalPointLabelSet;
		};

		class CGlobalPointLabelSet : public CPointLabel4Classfier
		{
		public:
			CGlobalPointLabelSet() { m_Timestamp = hiveCommon::hiveGetGlobalTimestamp(); }
			~CGlobalPointLabelSet() = default;
	
			void init(std::size_t, EPointLabel);

			CLocalPointLabelSet* clone() const;

			void applyPointLabelChange(const std::vector<SPointLabelChange>& vChangeRecord);
		};
	}
}