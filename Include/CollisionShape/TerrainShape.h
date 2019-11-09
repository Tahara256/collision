#pragma once

#include "CollisionShape.h"
#include <Collider/Implementation/UniformGridMesh.h>

/// <summary> �n�`�̏Փˌ`��N���X </summary>
class TerrainShape final : public CollisionShape {

public:

	/// <param name="terrainMesh"> �n�`�̃��b�V�� </param>
	TerrainShape(UniformGridMesh && terrainMesh);

	/// <summary> ���[���h���W�n�ɂ�����`��𕢂� AABB ���擾���܂��B </summary>
	const AABB coverAABB(const TransformQ & transform) const override;

	/// <summary> �`��̎�ނ��擾���܂��B </summary>
	CollisionShapeType getType() const override;

	/// <summary> ���g�Ƀ��C�����L���X�g���܂��B </summary>
	bool linecast(const TransformQ & transform, const Segment & segment, LinecastContact * outInfo = nullptr) const override;

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle) const override { return false; }

	/// <summary> ���g�ƎO�p�`�̏Փ˔�������܂��B </summary>
	bool intersectTriangle(const TransformQ & transform, const Triangle & triangle, Vector3 * outMyHitPos, Vector3 * outTriangleHitPos) const override { return false; }

	/// <summary> ���[���h���W�n�ɂ�����`������C���[�t���[���ŕ`�悵�܂��B </summary>
	void drawWireFrame(const TransformQ & transform, const Vector4 & color) const override;

	/// <summary> �n�`�̃��b�V�����擾���܂��B </summary>
	const UniformGridMesh & getTerrainMesh() const;

	/// <summary> �n�`�̃��b�V����ݒ肵�܂��B </summary>
	void setTerrainMesh(const UniformGridMesh & mesh);

private:

	UniformGridMesh _terrainMesh;

};
