#include <Collider/Collider/TerrainCollider.h>
#include <Collider/CollisionShape/TerrainShape.h>

TerrainCollider::TerrainCollider(UniformGridMesh && terrainMesh) :
	Collider(std::make_shared<TerrainShape>(std::move(terrainMesh))) {
}
