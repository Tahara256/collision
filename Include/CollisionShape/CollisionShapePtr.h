#pragma once

#include <memory>

class CollisionShape;

/// <summary> 衝突形状クラスへのポインタ </summary>
using CollisionShapePtr = std::shared_ptr<CollisionShape>;
