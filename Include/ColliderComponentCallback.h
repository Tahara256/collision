#pragma once

#include <functional>
#include <Util/RefPtr.h>

class Collision;
class ColliderComponent;

/// <summary> 衝突時コールバック </summary>
using OnCollision = std::function<void(const Collision &)>;

/// <summary> トリガー衝突時コールバック </summary>
using OnTrigger = std::function<void(RefPtr<ColliderComponent>)>;
