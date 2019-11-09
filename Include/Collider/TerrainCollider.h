#pragma once

#include <Collider/Collider/Collider.h>

class TerrainCollider : public Collider {

public:

	TerrainCollider(UniformGridMesh && terrainMesh);

};
