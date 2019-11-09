#pragma once

#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Sphere.h>

/// <summary> 球体の衝突形状クラス </summary>
class SphereShape final : public CollisionShape {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="localSphere"> ローカル座標系の球体 </param>
	SphereShape(const Sphere & localSphere);

	/// <summary> ワールド座標系における形状を覆う AABB を取得します。 </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> 形状の種類を取得します。 </summary>
	CollisionShapeType getType() const override;

	/// <summary> 自身にラインをキャストします。 </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override;

	/// <summary> 自身と三角形と衝突判定をします。 </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override;

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ワールド座標系に変換した球体を取得します。 </summary>
	const Sphere getWorldSphere(const TransformQ & transform) const;

	/// <summary> ローカル座標系の球体を取得します。 </summary>
	const Sphere & getLocalSphere() const;

	/// <summary> ローカル座標系の球体を設定します。 </summary>
	void setLocalSphere(const Sphere & localSphere);

private:

	Sphere _baseSphere;

};
