#pragma once

#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Face.h>

class TriangleShape final : public CollisionShape {

public:

	TriangleShape(const Triangle & localTriangle);

	/// <summary> ワールド座標系における形状を覆う AABB を取得します。 </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> 形状の種類を取得します。 </summary>
	CollisionShapeType getType() const override;

	/// <summary> 自身にラインをキャストします。 </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override { return false; }

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override { return false; };

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ワールド座標系に変換した三角形を取得します。 </summary>
	const Triangle getWorldTriangle(const TransformQ & transform) const;

	/// <summary> ローカル座標系の三角形を取得します。 </summary>
	const Triangle & getLocalTriangle() const;

	/// <summary> ローカル座標系の三角形を再設定します。 </summary>
	void setLocalTriangle(const Triangle & localTriangle);

private:

	Triangle _baseTriangle;

};
