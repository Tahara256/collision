#pragma once

#include "CollisionShape.h"
#include "../OBB.h"

/// <summary> 直方体の衝突形状クラス </summary>
class BoxShape final : public CollisionShape {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="localBox"> ローカル座標系の直方体 </param>
	BoxShape(const OBB & localBox);

	/// <summary> ワールド座標系における形状を覆う AABB を取得します。 </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> 形状の種類を取得します。 </summary>
	CollisionShapeType getType() const override;

	/// <summary> 自身にラインをキャストします。 </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override;

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ワールド座標系に変換した直方体を取得します。 </summary>
	const OBB getWorldBox(const TransformQ & transform) const;

	/// <summary> ローカル座標系の直方体を取得します。 </summary>
	const OBB & getLocalBox() const;

	/// <summary> ローカル座標系の直方体を設定します。 </summary>
	void setLocalBox(const OBB & localBox);

private:

	OBB _baseBox;

};
