#pragma once

#include "CollisionShape.h"
#include "../Capsule.h"

/// <summary> �J�v�Z���̏Փˌ`��N���X </summary>
class CapsuleShape final : public CollisionShape {

public:

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="localCapsule"> ���[�J�����W�n�̃J�v�Z�� </param>
	CapsuleShape(const Capsule & localCapsule);

	/// <summary> ���[���h���W�n�ɂ�����`��𕢂� AABB ���擾���܂��B </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> �`��̎�ނ��擾���܂��B </summary>
	CollisionShapeType getType() const override;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override;

	/// <summary> ��萳�m�ȏՓˏ����擾����ۂ̌덷���e�l(�����̓��) </summary>
	float toleranceError() const override { return _tolerance; }

	/// <summary> ���[���h���W�n�ɂ�����`������C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ���[���h���W�n�ɕϊ������J�v�Z�����擾���܂��B </summary>
	const Capsule getWorldCapsule(const TransformQ & transform) const;

	/// <summary> ���[�J�����W�n�̃J�v�Z�����擾���܂��B </summary>
	const Capsule & getLocalCapsule() const;

	/// <summary> ���[�J�����W�n�̃J�v�Z�����Đݒ肵�܂��B </summary>
	void setLocalCapsule(const Capsule & localCapsule);

private:

	Capsule _baseCapsule;
	float _tolerance;

};
