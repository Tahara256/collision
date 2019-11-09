#include <Collider/Implementation/SpatialPartition/SpaceCell.h>
#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

using namespace SpatialPartition;
using namespace std;

SpaceCell::SpaceCell() :
	_needsDetection(false) {
}

void SpaceCell::add(const OctreeObjectPtr & data) {
	// Ž©g‚É“o˜^‚³‚ê‚Ä‚¢‚éê‡‚Íˆ—‚È‚µ
	const auto already = data->isAttachedTo(this);
	if (already) return;

	if (data->isAttached()) {
		// ¡“o˜^‚³‚ê‚Ä‚¢‚é‹óŠÔ‚©‚ç—£‚·
		data->getCell()->remove(data);
	}

	// ‚±‚Ì‹óŠÔ‚É“o˜^
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
