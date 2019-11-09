#pragma once

/// <summary> 衝突形状の種類 </summary>
enum class CollisionShapeType {

	/// <summary> 球体 </summary>
	Sphere,

	/// <summary> カプセル </summary>
	Capsule,

	/// <summary> 立方体(OBB) </summary>
	Box,

	/// <summary> 三角ポリゴン </summary>
	Triangle,

	/// <summary> 地形 </summary>
	Terrain

};
