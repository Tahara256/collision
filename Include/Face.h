#pragma once

#include <cstddef>
#include <array>

#include <LMath.h>

/// <summary> ポリゴン </summary>
template <std::size_t VertexCount>
using Face = std::array<Vector3, VertexCount>;

/// <summary> 三角ポリゴン </summary>
using Triangle = std::array<Vector3, 3>;

/// <summary> 頂点へのインデックスで表現した三角ポリゴン </summary>
template <typename IndexType>
using TriangleIndexed = std::array<IndexType, 3>;
