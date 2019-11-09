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
/// <para>�Փˌ`��N���X</para>
/// <para>��̃C���X�^���X�𕡐��̃R���C�_�[�ŋ��L�ł��܂��B</para>
/// </summary>
class CollisionShape {

public:

	/// <summary> ���[���h���W�n�ɂ�����`��𕢂� AABB ���擾���܂��B </summary>
	virtual const AABB coverAABB(const TransformQ & transform) const = 0;

	/// <summary> �`��̎�ނ��擾���܂��B </summary>
	virtual CollisionShapeType getType() const = 0;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	virtual bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const = 0;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	virtual bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const = 0;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	virtual bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const = 0;

	/// <summary> ��萳�m�ȏՓˏ����擾����ۂ̌덷���e�l(�����̓��) </summary>
	virtual float toleranceError() const { return std::numeric_limits<float>::max(); }

	/// <summary> ���[���h���W�n�ɂ�����`������C���[�t���[���ŕ`�悵�܂��B </summary>
	virtual void drawWireFrame(const TransformQ & transform, const Vector4 & color) const {};

	/// <summary> ���g�ƃ��b�V���̏Փ˔�������܂��B </summary>
	bool intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh) const;

	/// <summary> ���g�ƃ��b�V���̏Փ˔�������܂��B </summary>
	bool intersectMesh(const TransformQ & transform, const UniformGridMesh & mesh, Vector3 * outMyHitPos, Vector3 * outMeshHitPos) const;


	virtual ~CollisionShape() = default;

};
