#pragma once

#include "CollisionShape.h"
#include "../Capsule.h"

/// <summary> カプセルの衝突形状クラス </summary>
class CapsuleShape final : public CollisionShape {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="localCapsule"> ローカル座標系のカプセル </param>
	CapsuleShape(const Capsule & localCapsule);

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

	/// <summary> より正確な衝突情報を取得する際の誤差許容値(距離の二乗) </summary>
	float toleranceError() const override { return _tolerance; }

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ワールド座標系に変換したカプセルを取得します。 </summary>
	const Capsule getWorldCapsule(const TransformQ & transform) const;

	/// <summary> ローカル座標系のカプセルを取得します。 </summary>
	const Capsule & getLocalCapsule() const;

	/// <summary> ローカル座標系のカプセルを再設定します。 </summary>
	void setLocalCapsule(const Capsule & localCapsule);

private:

	Capsule _baseCapsule;
	float _tolerance;

};
