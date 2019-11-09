#include <Collider/CollisionShape/TerrainShape.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Renderer/SceneRendererManager.h>

using namespace CollisionUtility;

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
    // d‚¢‚Ì‚Å–³
}

const UniformGridMesh & TerrainShape::getTerrainMesh() const {
	return _terrainMesh;
}

void TerrainShape::setTerrainMesh(const UniformGridMesh & mesh) {
	_terrainMesh = mesh;
}
