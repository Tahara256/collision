#include <Collider/Implementation/SpatialPartition/SpaceCell.h>
#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

using namespace SpatialPartition;
using namespace std;

SpaceCell::SpaceCell() :
	_needsDetection(false) {
}

void SpaceCell::add(const OctreeObjectPtr & data) {
	// 自身に登録されている場合は処理なし
	const auto already = data->isAttachedTo(this);
	if (already) return;

	if (data->isAttached()) {
		// 今登録されている空間から離す
		data->getCell()->remove(data);
	}

	// この空間に登録
	_attachedObjects.push_back(data);
	data->setCell(this);
}

void SpaceCell::remove(const OctreeObjectPtr & data) {
	_attachedObjects.remove(data);
}

const list<OctreeObjectPtr> & SpaceCell::attachedObjects() const {
	return _attachedObjects;
}

void SpaceCell::setNeedsDetection(bool needs) {
	_needsDetection = needs;
}

bool SpaceCell::needsDetection() const {
	return _needsDetection;
}
