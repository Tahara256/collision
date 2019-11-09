#pragma once

#include "Collider.h"

class OBB;

/// <summary> 直方体のコライダー </summary>
class BoxCollider final : public Collider
{
public:
	/// <summary> コンストラクタ </summary>
	/// <param name="localBox"> ローカル座標系の直方体 </param>
	BoxCollider(OBB const & localBox);
};
