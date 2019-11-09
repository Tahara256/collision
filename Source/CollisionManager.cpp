#include <Collider/CollisionManager.h>
#include <Collider/Implementation/CollisionAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/Segment.h>
#include <Collider/CollisionFilter/CollisionFilter.h>
#include <Collider/CollisionFilter/CollisionFilterNull.h>
#include <Collider/Collider/TerrainCollider.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Contact/LinecastContact.h>
#include <Collider/Implementation/CollisionLogManager.h>

using namespace std;
using namespace SpatialPartition;

template<> CollisionManager* Accessor<CollisionManager>::mSingleton = 0;

CollisionManager::CollisionManager() :
	_collisionFilter(std::make_unique<CollisionFilterNull>()),
	_octree(AABB(Vector3(-20, -200, -20), Vector3(1020, 800, 1020)), 5) {
}

CollisionManager::~CollisionManager() = default;

void CollisionManager::addCollider(const ColliderPtr & collider) {
	_colliderTable[collider->getID()] = collider;
	_octree.addObject(collider);
}

void CollisionManager::removeCollider(const ColliderPtr & collider) {
	_colliderTable.erase(collider->getID());
	_octree.removeObject(collider);
}

void CollisionManager::clear() {
	_colliderTable.clear();
}

void CollisionManager::update() {
	_octree.update();
	octreeCollision();
	CollisionLogManager::ins().notifyExitCallbackIfNeeded();
	CollisionLogManager::ins().updateLog();
}

const ColliderPtr CollisionManager::getCollider(ColliderID id) const {
	const auto contains = _colliderTable.count(id);
	if (!contains) return nullptr;
	return _colliderTable.at(id);
}

void CollisionManager::drawWireFrame(const Vector4 & color) const {
	//_octree.drawWireFrame(color);

	for (const auto & collider : _colliderTable) {
		if (!collider.second->getIsActive()) continue;
		collider.second->drawWireFrame(color);
	}
}

bool CollisionManager::linecast(const Segment & line, LinecastContact * outInfo, bool ignoreTrigger) const {
	const auto pre = [=](const ColliderPtr & collider) {
		if (!ignoreTrigger) return true;
		return !collider->getIsTrigger();
	};
	return linecast(line, pre, outInfo);
}

bool CollisionManager::linecast(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo, bool ignoreTrigger) const {
	const auto pre = [=](const ColliderPtr & collider) {
		if (!(collider->getCollisionLayer() & layerMask)) return false;
		if (!ignoreTrigger) return true;
		return !collider->getIsTrigger();
	};
	return linecast(line, pre, outInfo);
}

bool CollisionManager::linecastClosest(const Segment & line, LinecastContact * outInfo, bool ignoreTrigger) const {
	auto list = LinecastContactList();
	const auto isCollided = linecastAll(line, &list, ignoreTrigger);
	if (!isCollided) return false;
	auto result = std::min_element(begin(list), end(list), [&](const LinecastContact & a, const LinecastContact & b) {
		return a.getDistance() < b.getDistance();
	});
	*outInfo = *result;
	return true;
}

bool CollisionManager::linecastClosest(const Segment & line, CollisionLayerType layerMask, LinecastContact * outInfo, bool ignoreTrigger) const {
	auto list = LinecastContactList();
	const auto isCollided = linecastAll(line, layerMask, &list, ignoreTrigger);
	if (!isCollided) return false;
	auto result = std::min_element(begin(list), end(list), [&](const LinecastContact & a, const LinecastContact & b) {
		return a.getDistance() < b.getDistance();
	});
	*outInfo = *result;
	return true;
}

bool CollisionManager::linecastAll(const Segment & line, LinecastContactList * outInfos, bool ignoreTrigger) const {
	const auto pre = [=](const ColliderPtr & collider) {
		if (!ignoreTrigger) return true;
		return !collider->getIsTrigger();
	};
	return linecastAll(line, pre, outInfos);
}

bool CollisionManager::linecastAll(const Segment & line, CollisionLayerType layerMask, LinecastContactList * outInfos, bool ignoreTrigger) const {
	const auto pre = [=](const ColliderPtr & collider) {
		if (!(collider->getCollisionLayer() & layerMask)) return false;
		if (!ignoreTrigger) return true;
		return !collider->getIsTrigger();
	};
	return linecastAll(line, pre, outInfos);
}


void CollisionManager::setCollisionFilter(CollisionFilterPtr && collisionFilter) {
	_collisionFilter = move(collisionFilter);
}

bool CollisionManager::linecast(const Segment & line, const std::function<bool(const ColliderPtr &)> & pre, LinecastContact * outInfo) const {
	auto list = LinearOctreeManager::CollisionList();

	_octree.traverseAABB(line.coverAABB(), &list);
	for (const auto & obj : list) {
		const auto collider = static_pointer_cast<Collider>(obj);
		if (!pre(collider)) continue;
		if (collider->linecast(line, outInfo)) return true;
	}

	return false;
}

bool CollisionManager::linecastAll(const Segment & line, const std::function<bool(const ColliderPtr& )> & pre, LinecastContactList * outInfos) const {
	auto list = LinearOctreeManager::CollisionList();
	auto anyCollided = false;

	_octree.traverseAABB(line.coverAABB(), &list);
	for (const auto & obj : list) {
		const auto collider = static_pointer_cast<Collider>(obj);

		if (!pre(collider)) continue;

		auto info = LinecastContact();
		const auto isCollided = collider->linecast(line, &info);
		if (!isCollided) continue;

		anyCollided = true;
		outInfos->push_back(info);
	}

	return anyCollided;
}

void CollisionManager::octreeCollision() {
	auto list = LinearOctreeManager::CollisionPairList();
	_octree.getCollisionPairList(&list);

	for (const auto & pair : list) {
		const auto a = static_pointer_cast<Collider>(pair.first);
		const auto b = static_pointer_cast<Collider>(pair.second);
		collision(a, b);
	}
}

void CollisionManager::collision(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	const auto passFilter = _collisionFilter->filter(collider1, collider2);

	auto passNotBothFilter = !passFilter.first && !passFilter.second;

	if (passNotBothFilter) return;
	CollisionAgent::collision(collider1, collider2, passFilter.first, passFilter.second);
}
