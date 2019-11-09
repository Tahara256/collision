#include <Collider/Implementation/SpatialPartition/SpaceCell.h>
#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

using namespace SpatialPartition;
using namespace std;

SpaceCell::SpaceCell() :
	_needsDetection(false) {
}

void SpaceCell::add(const OctreeObjectPtr & data) {
	// ���g�ɓo�^����Ă���ꍇ�͏����Ȃ�
	const auto already = data->isAttachedTo(this);
	if (already) return;

	if (data->isAttached()) {
		// ���o�^����Ă����Ԃ��痣��
		data->getCell()->remove(data);
	}

	// ���̋�Ԃɓo�^
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
