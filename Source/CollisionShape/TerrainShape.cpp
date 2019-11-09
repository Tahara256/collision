#include <Collider/CollisionShape/TerrainShape.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Renderer/SceneRendererManager.h>

using namespace CollisionUtility;

//Vector3 TerrainShape::DrawPos = Vector3::zero;

TerrainShape::TerrainShape(UniformGridMesh && terrainMesh) :
	_terrainMesh(terrainMesh) {
}

const AABB TerrainShape::coverAABB(const TransformQ & transform) const {
	return _terrainMesh.coverAABB();
}

CollisionShapeType TerrainShape::getType() const {
	return CollisionShapeType::Terrain;
}

bool TerrainShape::linecast(const TransformQ & transform, const Segment & line, LinecastContact * outInfo) const {
	auto minXIndex = UniformGridMesh::Index();
	auto minZIndex = UniformGridMesh::Index();
	auto maxXIndex = UniformGridMesh::Index();
	auto maxZIndex = UniformGridMesh::Index();

	// 均一格子から線分のAABBが触れている格子の最小添え字と最大添え字を取得する
	_terrainMesh.getMinMaxIndex(line.coverAABB(), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	auto anyCollided = false;
	auto minDistance = std::numeric_limits<float>::max();

	// AABBが触れている格子に格納されているメッシュと衝突判定
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// 一つの格子に入っている三角ポリゴン二つ
			decltype(auto) trianglePair = _terrainMesh.getTrianglePair(x, z);

			auto distance = 0.0f;

			// 三角ポリゴンそれぞれと衝突判定
			auto isCollided = Collision3D::intersect(line, trianglePair.first, &distance);
			if (isCollided) {
				if (distance < minDistance) minDistance = distance;
				anyCollided = true;
			}

			isCollided = Collision3D::intersect(line, trianglePair.second, &distance);
			if (isCollided) {
				if (distance < minDistance) minDistance = distance;
				anyCollided = true;
			}
		}
	}

	if (!anyCollided) return false;
	if (!outInfo) return true;

	outInfo->setDistance(minDistance);
	outInfo->setPosition(line.pointFromDistance(minDistance));

	return true;
}

void TerrainShape::drawWireFrame(const TransformQ & transform, const Vector4 & color) const {
	//const auto offset = Vector3(1, 1, 1);
	//decltype(auto) aabb = AABB(DrawPos - offset, DrawPos + offset);

	//// AABBの最小値・最大値のXZ座標を使用して
	//// 均一格子からAABBが触れている格子の最小添え字と最大添え字を取得する
	//auto minXIndex = UniformGridMesh::Index();
	//auto minZIndex = UniformGridMesh::Index();
	//auto maxXIndex = UniformGridMesh::Index();
	//auto maxZIndex = UniformGridMesh::Index();

	//_terrainMesh.getMinMaxIndex(aabb, &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	//for (auto x = minXIndex; x <= maxXIndex; x++) {
	//	for (auto z = minZIndex; z <= maxZIndex; z++) {
	//		auto trianglePair = _terrainMesh.getTrianglePair(x, z);

	//		for (auto i = 0; i < 3; i++) {
	//			decltype(auto) edge = edgeFromFace(trianglePair.first, i);
	//			SceneRendererManager::debugDrawLine(edge.start(), edge.end(), color);
	//		}
	//	}
	//}
}

const UniformGridMesh & TerrainShape::getTerrainMesh() const {
	return _terrainMesh;
}

void TerrainShape::setTerrainMesh(const UniformGridMesh & mesh) {
	_terrainMesh = mesh;
}
