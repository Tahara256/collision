#pragma once

#include "Collider.h"

class Sphere;

/// <summary> 球体のコライダー </summary>
class SphereCollider final : public Collider
{
public:
	/// <summary> コンストラクタ </summary>
	/// <param name="localSphere"> ローカル座標系の球体 </param>
	SphereCollider(Sphere const & localSphere);
};
