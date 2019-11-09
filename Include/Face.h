#pragma once

#include <cstddef>
#include <array>

#include <LMath.h>

/// <summary> �|���S�� </summary>
template <std::size_t VertexCount>
using Face = std::array<Vector3, VertexCount>;

/// <summary> �O�p�|���S�� </summary>
using Triangle = std::array<Vector3, 3>;

/// <summary> ���_�ւ̃C���f�b�N�X�ŕ\�������O�p�|���S�� </summary>
template <typename IndexType>
using TriangleIndexed = std::array<IndexType, 3>;
