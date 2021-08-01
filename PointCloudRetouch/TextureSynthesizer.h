#pragma once
namespace hiveObliquePhotography::PointCloudRetouch
{
	using Texture_t = std::vector<std::vector<Eigen::Vector3f>>;
	using Mask_t = std::vector<std::vector<bool>>;

	class CTextureSynthesizer
	{
	public:
		CTextureSynthesizer() = default;
		~CTextureSynthesizer() = default;

		bool execute(const Texture_t& vInput, const Mask_t& vMask, Texture_t& vioScene);

	private:


	};
}
