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

	// ‹ÏˆêŠiq‚©‚çü•ª‚ÌAABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÌÅ¬“Y‚¦š‚ÆÅ‘å“Y‚¦š‚ğæ“¾‚·‚é
	_terrainMesh.getMinMaxIndex(line.coverAABB(), &minXIndex, &minZIndex, &maxXIndex, &maxZIndex);

	auto anyCollided = false;
	auto minDistance = std::numeric_limits<float>::max();

	// AABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÉŠi”[‚³‚ê‚Ä‚¢‚éƒƒbƒVƒ…‚ÆÕ“Ë”»’è
	for (auto x = minXIndex; x <= maxXIndex; x++) {
		for (auto z = minZIndex; z <= maxZIndex; z++) {

			// ˆê‚Â‚ÌŠiq‚É“ü‚Á‚Ä‚¢‚éOŠpƒ|ƒŠƒSƒ““ñ‚Â
			decltype(auto) trianglePair = _terrainMesh.getTrianglePair(x, z);

			auto distance = 0.0f;

			// OŠpƒ|ƒŠƒSƒ“‚»‚ê‚¼‚ê‚ÆÕ“Ë”»’è
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

	//// AABB‚ÌÅ¬’lEÅ‘å’l‚ÌXZÀ•W‚ğg—p‚µ‚Ä
	//// ‹ÏˆêŠiq‚©‚çAABB‚ªG‚ê‚Ä‚¢‚éŠiq‚ÌÅ¬“Y‚¦š‚ÆÅ‘å“Y‚¦š‚ğæ“¾‚·‚é
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
