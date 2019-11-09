#pragma once

#include <memory>

class Collider;

/// <summary> 衝突オブジェクトクラスへのポインタ </summary>
using ColliderPtr = std::shared_ptr<Collider>;
