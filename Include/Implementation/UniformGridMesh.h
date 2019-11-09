#pragma once

#include <ActorComponent/StaticInstanceMeshComponent.h>
#include <Collider/Face.h>
#include <Collider/Implementation/SpatialPartition/LinearOctreeManager.h>
#include <Collider/AABB.h>

class UniformGridMesh {

public:

	using Index = uint32;
	using TrianglePair = std::pair<Triangle, Triangle>;

	UniformGridMesh();
	UniformGridMesh(RefPtr<TerrainComponent> terrainComponent);

	const TrianglePair getTrianglePair(Index x, Index z) const;

	void getMinMaxIndex(const AABB & coverAABB, Index * outMinX, Index * outMinZ, Index * outMaxX, Index * outMaxZ) const;

	const AABB coverAABB() const;

private:

	RefPtr<TerrainComponent> _terrainComponent;

	Index _xHeightSize;
	Index _zHeightSize;
	Index _xCellMaxIndex;
	Index _zCellMaxIndex;
	Vector2 _cellSize;
	Vector2 _minPos;

	AABB _area;

};
