#pragma once

#include <list>

#include <Collider/Implementation/SpatialPartition/OctreeObjectPtr.h>

namespace SpatialPartition {

/// <summary> �������ꂽ�ŏ���� </summary>
class SpaceCell {

public:

	SpaceCell();

	void add(const OctreeObjectPtr & data);

	void remove(const OctreeObjectPtr & data);

	const std::list<OctreeObjectPtr> & attachedObjects() const;

	void setNeedsDetection(bool needs);

	bool needsDetection() const;

private:

	std::list<OctreeObjectPtr> _attachedObjects;

	bool _needsDetection;

};

} // namespace SpatialPartition
