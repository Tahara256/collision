#pragma once

#include "Collider.h"

class Capsule;

/// <summary> カプセルのコライダー </summary>
class CapsuleCollider final : public Collider
{
public:
	/// <summary> コンストラクタ </summary>
	/// <param name="localCapsule"> ローカル座標系のカプセル </param>
	CapsuleCollider(Capsule const & localCapsule);
};
