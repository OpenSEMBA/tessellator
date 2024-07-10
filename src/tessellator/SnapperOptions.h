#pragma once
namespace meshlib {
namespace tessellator {

struct SnapperOptions {
	double forbiddenLength{ 0.0 };
	std::size_t edgePoints{ 0 };
};


}
}
