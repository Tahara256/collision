#pragma once

#include <Collider/CollisionShape/CollisionShape.h>
#include <Collider/Face.h>

class TriangleShape final : public CollisionShape {

public:

	TriangleShape(const Triangle & localTriangle);

	/// <summary> ���[���h���W�n�ɂ�����`��𕢂� AABB ���擾���܂��B </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> �`��̎�ނ��擾���܂��B </summary>
	CollisionShapeType getType() const override;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override { return false; }

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override { return false; };

	/// <summary> ���[���h���W�n�ɂ�����`������C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> ���[���h���W�n�ɕϊ������O�p�`���擾���܂��B </summary>
	const Triangle getWorldTriangle(const TransformQ & transform) const;

	/// <summary> ���[�J�����W�n�̎O�p�`���擾���܂��B </summary>
	const Triangle & getLocalTriangle() const;

	/// <summary> ���[�J�����W�n�̎O�p�`���Đݒ肵�܂��B </summary>
	void setLocalTriangle(const Triangle & localTriangle);

private:

	Triangle _baseTriangle;

};
