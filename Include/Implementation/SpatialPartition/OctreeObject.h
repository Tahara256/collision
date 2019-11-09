#pragma once

class AABB;

namespace SpatialPartition {

class SpaceCell;

/// <summary> �f�R���[�^�[�ŋ�ԂɃA�^�b�`�ł���悤�� </summary>
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
