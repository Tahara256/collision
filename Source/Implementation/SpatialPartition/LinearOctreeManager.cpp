#include <Collider/Implementation/SpatialPartition/LinearOctreeManager.h>
#include <cassert>
#include <Renderer/SceneRendererManager.h>
#include <Collider/Implementation/CollisionUtility.h>
#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

using namespace SpatialPartition;
using namespace std;
using namespace CollisionUtility;

LinearOctreeManager::LinearOctreeManager(const AABB & area, int32 level) :
	_octree(level),
	_unitSize(area.size() / static_cast<float>(1 << level)),
	_level(level),
	_maxIndex((1 << level) - 1),
	_area(area) {

	// モートン番号算出時1バイト変数を使用してるため、分割レベルは8以下(軸毎256分割)に限定
	assert(level < 9 && "制御できる空間数を超えるため、空間分割数を減らしてください。");
}

void LinearOctreeManager::addObject(const OctreeObjectPtr & obj) {
	_objects.push_back(obj);
}

void LinearOctreeManager::removeObject(const OctreeObjectPtr & object) {
	detachObject(object);
	_objects.remove(object);
}

void LinearOctreeManager::clearObject() {
	for (const auto & obj : _objects)
		detachObject(obj);

	_objects.clear();
}

void LinearOctreeManager::update() {
	for (const auto & object : _objects) {
		if (!object->needsUpdate()) continue;
		updateObject(object);
	}
}

void LinearOctreeManager::getCollisionPairList(CollisionPairList * outCollisionPairList) const {
	auto colliderStack = std::list<OctreeObjectPtr>();
	createCollisionPairList(0, outCollisionPairList, &colliderStack);
}

void LinearOctreeManager::traverseAABB(const AABB & coverAABB, CollisionList * outCollisionList) const {
	auto belongsLevel = int32();
	auto mortonCode = int32();
	mortonCodeAndBelongsLevel3D(coverAABB, _level, _area.min, _unitSize, _maxIndex, &mortonCode, &belongsLevel);
	createCollisionListAll(_octree.getIndex(mortonCode, belongsLevel), outCollisionList);
}

void LinearOctreeManager::drawWireFrame(const Vector4 & color) const {
	SceneRendererManager::debugDrawBox(_area.center(), _area.size(), Quaternion::identity, color);
}

void LinearOctreeManager::updateObject(const OctreeObjectPtr & object) {
	decltype(auto) aabb = object->coverAABB();

	//assert(_area.contains(aabb) && "八分木の管理している空間外にオブジェクトが出ました。");
	// 管理できる空間外に出たオブジェクトは判定しない
	if (!_area.contains(aabb)) {
		detachObject(object);
		return;
	}

	if (!object->getIsActive()) {
		detachObject(object);
		return;
	}

	auto belongsLevel = int32();
	auto mortonCode = int32();
	mortonCodeAndBelongsLevel3D(aabb, _level, _area.min, _unitSize, _maxIndex, &mortonCode, &belongsLevel);

	const auto index = _octree.getIndex(mortonCode, belongsLevel);

	_octree[index].add(object);

	cellSetUp(index);
}

void LinearOctreeManager::detachObject(const OctreeObjectPtr & object) {
	if (!object->isAttached()) return;
	object->getCell()->remove(object);
	object->detach();
}

void LinearOctreeManager::createCollisionPairList(int32 index, CollisionPairList * colliderList, CollisionStack * colliderStack) const {
	decltype(auto) objects = _octree[index].attachedObjects();

	for (auto itr1 = begin(objects); itr1 != end(objects); ++itr1) {
		// 1. 現在の空間のオブジェクト同士で衝突判定
		for (auto itr2 = next(itr1); itr2 != end(objects); ++itr2)
			colliderList->emplace_back(*itr1, *itr2);

		// 2. 親空間(スタック)との衝突判定
		for (const auto & obj : *colliderStack)
			colliderList->emplace_back(*itr1, obj);
	}

	auto objCount = 0;
	// 3. 現在の空間の登録オブジェクトをスタックに追加
	for (const auto & object : objects) {
		colliderStack->push_back(object);
		objCount++;
	}

	// 4. スタックをもって子空間に移動
	for (auto i = int32(0); i < BranchCount; i++) {
		const auto nextIndex = _octree.getChildIndex(index, i);
		if (nextIndex >= _octree.getSize()) continue;
		if (!_octree[nextIndex].needsDetection()) continue;

		createCollisionPairList(nextIndex, colliderList, colliderStack);
	}

	// 5. スタックから登録した分だけオブジェクトを外す
	for (auto i = 0; i < objCount; i++)
		colliderStack->pop_back();
}

void LinearOctreeManager::createCollisionListAll(int32 index, CollisionList * outCollisionList) const {
	createCollisionListChild(index, outCollisionList);
	if (!index) return;
	createCollisionListParent(_octree.getParentIndex(index), outCollisionList);
}

void LinearOctreeManager::createCollisionList(int32 index, CollisionList * outCollisionList) const {
	decltype(auto) objects = _octree[index].attachedObjects();
	for (const auto & object : objects)
		outCollisionList->push_back(object);
}

void LinearOctreeManager::createCollisionListChild(int32 index, CollisionList * outCollisionList) const {
	createCollisionList(index, outCollisionList);

	for (auto i = 0; i < BranchCount; i++) {
		const auto childIndex = _octree.getChildIndex(index, i);
		if (childIndex >= _octree.getSize()) continue;
		if (!_octree[childIndex].needsDetection()) continue;
		createCollisionListChild(childIndex, outCollisionList);
	}
}

void LinearOctreeManager::createCollisionListParent(int32 index, CollisionList * outCollisionList) const {
	if (!index) return;

	createCollisionList(index, outCollisionList);

	createCollisionListParent(_octree.getParentIndex(index), outCollisionList);
}

void LinearOctreeManager::cellSetUp(int32 cellIndex) {
	_octree[cellIndex].setNeedsDetection(true);
	if (!cellIndex) return;
	cellSetUp(_octree.getParentIndex(cellIndex));
}
