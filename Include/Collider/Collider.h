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
/// <para>�Փ˃I�u�W�F�N�g�N���X</para>
/// <para>�`��E���[���h�ϊ���ێ����܂��B</para>
/// </summary>
class Collider : public SpatialPartition::OctreeObject, public NonMoveCopyable {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="collisionShape"> �����̃R���C�_�[�̌`�� </param>
	Collider(const CollisionShapePtr & collisionShape);

	/// <summary> �R���C�_�[�̌`���ݒ肵�܂��B </summary>
	/// <param name="collisionShape"> �R���C�_�[�̌`�� </param>
	void setCollisionShape(const CollisionShapePtr & collisionShape);

	/// <summary> �R���C�_�[�̎p����ݒ肵�܂��B(scale�͓K������܂���) </summary>
	/// <param name="transform"> �R���C�_�[�̎p�� </param>
	void setTransform(const TransformQ & transform);

	void setPreTransform(const TransformQ & preTransform);

	/// <summary> �Փ˂̃}�X�N�Ɏg�p���郌�C���[��ݒ肵�܂��B </summary>
	void setCollisionLayer(CollisionLayerType collisionLayer);

	/// <summary> �Փ˂̃}�X�N��ݒ肵�܂��B </summary>
	void setCollisionMask(CollisionLayerType collisionMask);

	/// <summary> �R���C�_�[���Փ˒��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionStay(const OnCollisionCallback & onCollisionStay);

	/// <summary> �R���C�_�[���Փ˂��n�߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter);

	/// <summary> �R���C�_�[���Փ˂���߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnCollisionExit(const OnCollisionCallback & onCollisionExit);

	/// <summary> �g���K�[�̃R���C�_�[���Փ˒��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerStay(const OnTriggerCallback & onTriggerStay);

	/// <summary> �g���K�[�̃R���C�_�[���Փ˂��n�߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter);

	/// <summary> �g���K�[�̃R���C�_�[���Փ˂���߂��Ƃ��̃R�[���o�b�N��ǉ����܂��B </summary>
	void addOnTriggerExit(const OnTriggerCallback & onTriggerExit);

	/// <summary> �R���C�_�[�̌`����擾���܂��B </summary>
	const CollisionShapePtr & getCollisionShape() const;

	/// <summary> �R���C�_�[�̎p�����擾���܂��B </summary>
	const TransformQ & getTransform() const;

	const TransformQ & getPreTransform() const;

	/// <summary> �`��𕢂� AABB ���擾���܂��B </summary>
	const AABB coverAABB() const override;

	/// <summary> �R���C�_�[�̌`��̎�ނ��擾���܂��B </summary>
	CollisionShapeType getCollisionShapeType() const;

	/// <summary> �Փ˂̃}�X�N�Ɏg�p���郌�C���[���擾���܂��B </summary>
	CollisionLayerType getCollisionLayer() const;

	/// <summary> �Փ˂̃}�X�N���擾���܂��B </summary>
	CollisionLayerType getCollisionMask() const;

	/// <summary> ID���擾���܂��B </summary>
	ColliderID getID() const;

	/// <summary> �L�����ǂ������擾���܂��B </summary>
	bool getIsActive() const override;

	/// <summary> �ÓI�I�u�W�F�N�g�����擾���܂��B </summary>
	bool getIsStatic() const override;

	/// <summary> �g���K�[�����擾���܂��B </summary>
	bool getIsTrigger() const;

	/// <summary> �L�����ǂ�����ݒ肵�܂��B </summary>
	void setIsActive(bool isActive);

	/// <summary> �ÓI�I�u�W�F�N�g����ݒ肵�܂��B </summary>
	void setIsStatic(bool isStatic);

	/// <summary> �g���K�[����ݒ肵�܂��B </summary>
	void setIsTrigger(bool isTrigger);

	/// <summary> �R���C�_�[�̌`������C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const Vector4 & color) const;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	bool linecast(const Segment & line, LinecastContact * outInfo = nullptr) const;

	/// <summary> ���b�V���Ƃ̏Փ˔��� </summary>
	bool intersectMesh(const UniformGridMesh & mesh) const;

	/// <summary> ���b�V���Ƃ̏Փ˔��� </summary>
	bool intersectMesh(const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const;

	/// <summary> �R���C�_�[�Փˊ֘A�̃R�[���o�b�N���Ăяo���܂��B </summary>
	void notifyOnCollisionStay(const Contact & info);
	void notifyOnCollisionEnter(const Contact & info);
	void notifyOnCollisionExit(const Contact & info);

	/// <summary> �g���K�[�̃R���C�_�[�Փˊ֘A�̃R�[���o�b�N���Ăяo���܂��B </summary>
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
