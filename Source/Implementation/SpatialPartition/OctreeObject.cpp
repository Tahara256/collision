#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

using namespace SpatialPartition;

OctreeObject::OctreeObject() : _cell(nullptr) {
}

SpaceCell * OctreeObject::getCell() const {
	return _cell;
}

void OctreeObject::setCell(SpaceCell * cell) {
	_cell = cell;
}

void OctreeObject::detach() {
	_cell = nullptr;
}

bool OctreeObject::isAttachedTo(const SpaceCell * cell) const {
	return cell == _cell;
}

bool OctreeObject::isAttached() const {
	return _cell;
}

bool OctreeObject::needsUpdate() const {
	return !(getIsStatic() && isAttached());
}
