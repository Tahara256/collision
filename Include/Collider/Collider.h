#pragma once

#include <Collider/Implementation/SpatialPartition/OctreeObject.h>

#include <LMath.h>
#include <Util/Design/NonMoveCopyable.h>
#include <Collider/CollisionShape/CollisionShapePtr.h>
#include <Collider/Collider/CollisionCallbackManager.h>
#include <Collider/CollisionFilter/CollisionLayerType.h>
#include <Collider/Collider/ColliderID.h>

enum class CollisionShapeType;
class LinecastContact;
class AABB;
class Segment;
class UniformGridMesh;

/// <summary>
/// <para>衝突オブジェクトクラス</para>
/// <para>形状・ワールド変換を保持します。</para>
/// </summary>
class Collider : public SpatialPartition::OctreeObject, public NonMoveCopyable {

public:

	/// <summary> コンストラクタ </summary>
	/// <param name="collisionShape"> 初期のコライダーの形状 </param>
	Collider(const CollisionShapePtr & collisionShape);

	/// <summary> コライダーの形状を設定します。 </summary>
	/// <param name="collisionShape"> コライダーの形状 </param>
	void setCollisionShape(const CollisionShapePtr & collisionShape);

	/// <summary> コライダーの姿勢を設定します。(scaleは適応されません) </summary>
	/// <param name="transform"> コライダーの姿勢 </param>
	void setTransform(const TransformQ & transform);

	void setPreTransform(const TransformQ & preTransform);

	/// <summary> 衝突のマスクに使用するレイヤーを設定します。 </summary>
	void setCollisionLayer(CollisionLayerType collisionLayer);

	/// <summary> 衝突のマスクを設定します。 </summary>
	void setCollisionMask(CollisionLayerType collisionMask);

	/// <summary> コライダーが衝突中のコールバックを追加します。 </summary>
	void addOnCollisionStay(const OnCollisionCallback & onCollisionStay);

	/// <summary> コライダーが衝突し始めたときのコールバックを追加します。 </summary>
	void addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter);

	/// <summary> コライダーが衝突をやめたときのコールバックを追加します。 </summary>
	void addOnCollisionExit(const OnCollisionCallback & onCollisionExit);

	/// <summary> トリガーのコライダーが衝突中のコールバックを追加します。 </summary>
	void addOnTriggerStay(const OnTriggerCallback & onTriggerStay);

	/// <summary> トリガーのコライダーが衝突し始めたときのコールバックを追加します。 </summary>
	void addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter);

	/// <summary> トリガーのコライダーが衝突をやめたときのコールバックを追加します。 </summary>
	void addOnTriggerExit(const OnTriggerCallback & onTriggerExit);

	/// <summary> コライダーの形状を取得します。 </summary>
	const CollisionShapePtr & getCollisionShape() const;

	/// <summary> コライダーの姿勢を取得します。 </summary>
	const TransformQ & getTransform() const;

	const TransformQ & getPreTransform() const;

	/// <summary> 形状を覆う AABB を取得します。 </summary>
	const AABB coverAABB() const override;

	/// <summary> コライダーの形状の種類を取得します。 </summary>
	CollisionShapeType getCollisionShapeType() const;

	/// <summary> 衝突のマスクに使用するレイヤーを取得します。 </summary>
	CollisionLayerType getCollisionLayer() const;

	/// <summary> 衝突のマスクを取得します。 </summary>
	CollisionLayerType getCollisionMask() const;

	/// <summary> IDを取得します。 </summary>
	ColliderID getID() const;

	/// <summary> 有効かどうかを取得します。 </summary>
	bool getIsActive() const override;

	/// <summary> 静的オブジェクトかを取得します。 </summary>
	bool getIsStatic() const override;

	/// <summary> トリガーかを取得します。 </summary>
	bool getIsTrigger() const;

	/// <summary> 有効かどうかを設定します。 </summary>
	void setIsActive(bool isActive);

	/// <summary> 静的オブジェクトかを設定します。 </summary>
	void setIsStatic(bool isStatic);

	/// <summary> トリガーかを設定します。 </summary>
	void setIsTrigger(bool isTrigger);

	/// <summary> コライダーの形状をワイヤーフレームで描画します。 </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> 自身にラインをキャストします。 </summary>
	bool linecast(const Segment & line, LinecastContact * outInfo = nullptr) const;

	/// <summary> メッシュとの衝突判定 </summary>
	bool intersectMesh(const UniformGridMesh & mesh) const;

	/// <summary> メッシュとの衝突判定 </summary>
	bool intersectMesh(const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const;

	/// <summary> コライダー衝突関連のコールバックを呼び出します。 </summary>
	void notifyOnCollisionStay(const Contact & info);
	void notifyOnCollisionEnter(const Contact & info);
	void notifyOnCollisionExit(const Contact & info);

	/// <summary> トリガーのコライダー衝突関連のコールバックを呼び出します。 </summary>
	void notifyOnTriggerStay(const TriggerContact & info);
	void notifyOnTriggerEnter(const TriggerContact & info);
	void notifyOnTriggerExit(const TriggerContact & info);

private:

	CollisionShapePtr			_collisionShape;
	TransformQ					_transform;
	TransformQ					_preTransform;
	CollisionLayerType			_collisionLayer;
	CollisionLayerType			_collisionMask;
	ColliderID					_id;
	bool						_isActive;
	bool						_isStatic;
	bool						_isTrigger;
	CollisionCallbackManager	_callbackManager;

public:

	virtual ~Collider() = default;

};
