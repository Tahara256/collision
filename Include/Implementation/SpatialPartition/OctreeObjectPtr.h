#pragma once

#include <memory>

namespace SpatialPartition {

class OctreeObject;

/// <summary> ��ԂɃA�^�b�`�ł���I�u�W�F�N�g�ւ̃|�C���^ </summary>
using OctreeObjectPtr = std::shared_ptr<OctreeObject>;

} // namespace SpatialPartition
