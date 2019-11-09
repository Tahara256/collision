#pragma once

#include <memory>

namespace SpatialPartition {

class OctreeObject;

/// <summary> 空間にアタッチできるオブジェクトへのポインタ </summary>
using OctreeObjectPtr = std::shared_ptr<OctreeObject>;

} // namespace SpatialPartition
