#pragma once

#include <memory>

class CollisionFilter;

/// <summary> 衝突判定のフィルターへのポインタ </summary>
using CollisionFilterPtr = std::unique_ptr<CollisionFilter>;
