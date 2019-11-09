#pragma once

#include <Collider/Face.h>

enum class CollisionShapeType;
class Vector4;
class AABB;
class Segment;
class UniformGridMesh;
class LinecastContact;
struct TransformQ;

/// <summary>
/// <para>衝突形状クラス</para>
/// <para>一つのインスタンスを複数のコライダーで共有できます。</para>
/// </summary>
class CollisionShape {

public:

	/// <summary> ワールド座標系における形状を覆う AABB を取得します。 </summary>
	virtual const AABB coverAABB(const TransformQ & transform) const = 0;

	/// <summary> 形状の種類を取得します。 </summary>
	virtual CollisionShapeType getType() const = 0;

	/// <summary> 自身にラインをキャストします。 </summary>
	virtual bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const = 0;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	virtual bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const = 0;

	/// <summary> 自身と三角形の衝突判定をします。 </summary>
	virtual bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const = 0;

	/// <summary> より正確な衝突情報を取得する際の誤差許容値(距離の二乗) </summary>
	virtual float toleranceError() const { return std::numeric_limits<float>::max(); }

	/// <summary> ワールド座標系における形状をワイヤーフレームで描画します。 </summary>
	virtual void drawWireFrame(const TransformQ & transform, const Vector4 & color) const {};

	/// <summary> 自身とメッシュの衝突判定をします。 </summary>
	bool intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh) const;

	/// <summary> 自身とメッシュの衝突判定をします。 </summary>
	bool intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const;


	virtual ~CollisionShape() = default;

};
