#pragma once

class AABB;

namespace SpatialPartition {

class SpaceCell;

/// <summary> デコレーターで空間にアタッチできるように </summary>
class OctreeObject {

public:

	virtual const AABB coverAABB() const = 0;

	virtual bool getIsActive() const = 0;

	virtual bool getIsStatic() const = 0;


	OctreeObject();

	virtual ~OctreeObject() = default;

	SpaceCell * getCell() const;

	void setCell(SpaceCell * cell);

	void detach();

	bool isAttachedTo(const SpaceCell * cell) const;

	bool isAttached() const;

	bool needsUpdate() const;

private:

	SpaceCell * _cell;

};

} // namespace SpatialPartition
