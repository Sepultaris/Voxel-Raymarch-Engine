#pragma once

#include <cstdint>

namespace voxel {

#define VOXEL_SHARED_INLINE [[nodiscard]] constexpr
#define VOXEL_SHARED_UINT std::uint32_t
#include "../../shared/geodesic_material_shared.inc"
#undef VOXEL_SHARED_UINT
#undef VOXEL_SHARED_INLINE

} // namespace voxel
