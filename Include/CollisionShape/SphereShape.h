#pragma once

#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Sphere.h>

/// <summary> ���̂̏Փˌ`��N���X </summary>
class SphereShape final : public CollisionShape {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="localSphere"> ���[�J�����W�n�̋��� </param>
	SphereShape(const Sphere & localSphere);

	/// <summary> ���[���h���W�n�ɂ�����`��𕢂� AABB ���擾���܂��B </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> �`��̎�ނ��擾���܂��B </summary>
	CollisionShapeType getType() const override;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override;

	/// <summary> ���g�ƎO�p�`�ƏՓ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override;

	/// <summary> ���[���h���W�n�ɂ�����`������C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ���[���h���W�n�ɕϊ��������̂��擾���܂��B </summary>
	const Sphere getWorldSphere(const TransformQ & transform) const;

	/// <summary> ���[�J�����W�n�̋��̂��擾���܂��B </summary>
	const Sphere & getLocalSphere() const;

	/// <summary> ���[�J�����W�n�̋��̂�ݒ肵�܂��B </summary>
	void setLocalSphere(const Sphere & localSphere);

private:

	Sphere _baseSphere;

};
