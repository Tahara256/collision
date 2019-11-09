#include <Collider/Implementation/UniformGridMesh.h>
#include <LMath.h>
#include <Renderer/GameRenderer.h>

using namespace std;

UniformGridMesh::UniformGridMesh() :
	_terrainComponent(),
	_xHeightSize(0),
	_zHeightSize(0),
	_xCellMaxIndex(0),
	_zCellMaxIndex(0) {
}

UniformGridMesh::UniformGridMesh(RefPtr<TerrainComponent> terrainComponent) :
	_terrainComponent(terrainComponent),
	_xHeightSize(2049),
	_zHeightSize(2049),
	_xCellMaxIndex(2047),
	_zCellMaxIndex(2047) {
	const auto size = static_cast<float>(_terrainComponent->getWidth());
	const auto interval = size / (_xHeightSize - 1);
	_cellSize = Vector2(interval, interval);
	_area = AABB(Vector3::zero, Vector3(size, 150.0f, size));
}

const UniformGridMesh::TrianglePair UniformGridMesh::getTrianglePair(Index x, Index z) const {
	auto HeightLB = 0.0f;
	auto HeightLF = 0.0f;
	auto HeightRB = 0.0f;
	auto HeightRF = 0.0f;

	// TerrainComponentを持ってたら高さを直接取得する
	if (!_terrainComponent) return UniformGridMesh::TrianglePair();

	constexpr auto normalize = 150.0f / MAXUINT16;
	HeightLB = normalize * _terrainComponent->getTerrainHeightIndex(x, z);
	HeightLF = normalize * _terrainComponent->getTerrainHeightIndex(x, z + 1);
	HeightRB = normalize * _terrainComponent->getTerrainHeightIndex(x + 1, z);
	HeightRF = normalize * _terrainComponent->getTerrainHeightIndex(x + 1, z + 1);

	const auto VertexLB = Vector3(x * _cellSize.x, HeightLB, z *_cellSize.y);
	const auto VertexLF = Vector3(x * _cellSize.x, HeightLF, (z + 1) *_cellSize.y);
	const auto VertexRB = Vector3((x + 1) * _cellSize.x, HeightRB, z *_cellSize.y);
	const auto VertexRF = Vector3((x + 1) * _cellSize.x, HeightRF, (z + 1) *_cellSize.y);

	return { { VertexLB, VertexLF, VertexRB }, { VertexRF, VertexRB, VertexLF } };
}

void UniformGridMesh::getMinMaxIndex(const AABB & coverAABB, Index * outMinX, Index * outMinZ, Index * outMaxX, Index * outMaxZ) const {
	const auto min = coverAABB.min;
	const auto max = coverAABB.max;

	// AABBの最小値・最大値のXZ座標を使用して
	// 均一格子からAABBが触れている格子の最小添え字と最大添え字を取得する
	*outMinX = Mathf::clamp(static_cast<Index>((min.x - _minPos.x) / _cellSize.x), Index(0), _xCellMaxIndex);
	*outMinZ = Mathf::clamp(static_cast<Index>((min.z - _minPos.y) / _cellSize.y), Index(0), _zCellMaxIndex);
	*outMaxX = Mathf::clamp(static_cast<Index>((max.x - _minPos.x) / _cellSize.x), Index(0), _xCellMaxIndex);
	*outMaxZ = Mathf::clamp(static_cast<Index>((max.z - _minPos.y) / _cellSize.y), Index(0), _zCellMaxIndex);
}

const AABB UniformGridMesh::coverAABB() const {
	return _area;
}
