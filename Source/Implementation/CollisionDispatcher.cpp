#include <Collider/Implementation/CollisionDispatcher.h>
#include <unordered_map>
#include <Collider/Implementation/IntersectAgent/IntersectAgentPtr.h>
#include <Collider/CollisionShape/CollisionShapeType.h>
#include <Collider/Collider/Collider.h>
#include <Collider/Implementation/IntersectAgent/SphereSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereBoxAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleBoxAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleTriangleAgent.h>
#include <Collider/Implementation/IntersectAgent/BoxBoxAgent.h>
#include <Collider/Implementation/IntersectAgent/BoxSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/BoxCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/BoxTriangleAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereTriangleAgent.h>
#include <Collider/Implementation/IntersectAgent/TriangleSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/TriangleCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/TriangleBoxAgent.h>
#include <Collider/Implementation/IntersectAgent/TerrainAnyColliderAgent.h>
#include <Collider/Implementation/IntersectAgent/AnyColliderTerrainAgent.h>
#include <Collider/Implementation/IntersectAgent/NullIntersectAgent.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;

using AgentTable = unordered_map<CollisionShapeType, unordered_map<CollisionShapeType, IntersectAgentPtr>>;

static AgentTable table = {
	{ CollisionShapeType::Sphere,{
		{ CollisionShapeType::Sphere, make_shared<SphereSphereAgent>() },
		{ CollisionShapeType::Capsule, make_shared<SphereCapsuleAgent>() },
		{ CollisionShapeType::Box, make_shared<SphereBoxAgent>() } ,
		{ CollisionShapeType::Triangle, make_shared<SphereTriangleAgent>() } ,
		{ CollisionShapeType::Terrain, make_shared<AnyColliderTerrainAgent>() } } },
	{ CollisionShapeType::Capsule,{
		{ CollisionShapeType::Capsule, make_shared<CapsuleCapsuleAgent>() },
		{ CollisionShapeType::Sphere, make_shared<CapsuleSphereAgent>() },
		{ CollisionShapeType::Box, make_shared<CapsuleBoxAgent>() },
		{ CollisionShapeType::Triangle, make_shared<CapsuleTriangleAgent>() },
		{ CollisionShapeType::Terrain, make_shared<AnyColliderTerrainAgent>() } } },
	{ CollisionShapeType::Box,{
		{ CollisionShapeType::Box, make_shared<BoxBoxAgent>() },
		{ CollisionShapeType::Sphere, make_shared<BoxSphereAgent>() },
		{ CollisionShapeType::Capsule, make_shared<BoxCapsuleAgent>() },
		{ CollisionShapeType::Triangle, make_shared<BoxTriangleAgent>() },
		{ CollisionShapeType::Terrain, make_shared<AnyColliderTerrainAgent>() } } },
	{ CollisionShapeType::Triangle,{
		{ CollisionShapeType::Sphere, make_shared<TriangleSphereAgent>() },
		{ CollisionShapeType::Capsule, make_shared<TriangleCapsuleAgent>() },
		{ CollisionShapeType::Box, make_shared<TriangleBoxAgent>() },
		{ CollisionShapeType::Triangle, make_shared<NullIntersectAgent>() },
		{ CollisionShapeType::Terrain, make_shared<AnyColliderTerrainAgent>() } } },
	{ CollisionShapeType::Terrain,{
		{ CollisionShapeType::Sphere, make_shared<TerrainAnyColliderAgent>() },
		{ CollisionShapeType::Capsule, make_shared<TerrainAnyColliderAgent>() },
		{ CollisionShapeType::Box, make_shared<TerrainAnyColliderAgent>() },
		{ CollisionShapeType::Triangle, make_shared<TerrainAnyColliderAgent>() },
		{ CollisionShapeType::Terrain, make_shared<NullIntersectAgent>() } } }
};

static const IntersectAgentPtr & intersectAgent(const ColliderPtr & collider1, const ColliderPtr & collider2) {
	const auto type1 = collider1->getCollisionShapeType();
	const auto type2 = collider2->getCollisionShapeType();
	return table[type1][type2];
}

bool CollisionDispatcher::intersectBlock(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) {
	return intersectAgent(collider1, collider2)->intersect(collider1, collider2, outHitPos1, outHitPos2);
}

bool CollisionDispatcher::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) {
	return intersectAgent(collider1, collider2)->intersectTrigger(collider1, collider2);
}
