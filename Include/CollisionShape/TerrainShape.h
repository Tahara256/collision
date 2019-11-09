#pragma once

#include "CollisionShape.h"
#include <Collider/Implementation/UniformGridMesh.h>

/// <summary> 地形の衝突形状クラス </summary>
class TerrainShape final : public CollisionShape {

public:

	/// <param name="terrainMesh"> 地形のメッシュ </param>
	TerrainShape(UniformGridMesh && terrainMesh);

	/// <summary> ワールド座標系における形状を覆う AABB を取得します。 </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> 形状の種類を取得します。 </summary>
	CollisionShapeType getType() const override;

	/// <summary> 自身にラインをキャストします。 </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override { return false; }

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override { return false; }

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> 地形のメッシュを取得します。 </summary>
	const UniformGridMesh & getTerrainMesh() const;

	/// <summary> 地形のメッシュを設定します。 </summary>
	void setTerrainMesh(const UniformGridMesh & mesh);

private:

	UniformGridMesh _terrainMesh;

};
