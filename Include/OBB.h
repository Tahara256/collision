#pragma once

#include <array>

#include <LMath.h>
#include <Collider/Face.h>

class Segment;
class AABB;

/// <summary> �L���o�E���f�B���O�{�b�N�X </summary>
class OBB {

public:

	/// <summary> OBB�̒��_�� </summary>
	static constexpr int VertexCount = 8;

	/// <summary> OBB�̐����� </summary>
	static constexpr int SegmentCount = 12;

	/// <summary> OBB�̖ʐ� </summary>
	static constexpr int FaceCount = 6;

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="center"> ���S���W </param>
	/// <param name="size"> ���[�J���̊e���̕� </param>
	/// <param name="rotation"> ��] </param>
	OBB(const Vector3 & center, const Vector3 & size, const Quaternion & rotation);

	/// <summary> ���S���W���擾���܂��B </summary>
	const Vector3 & center() const;

	/// <summary> x, y, z �����̑傫��(���a���@)���擾���܂��B </summary>
	const Vector3 size() const;

	/// <summary> x, y, z �����̔����̑傫��(���a���@)���擾���܂��B </summary>
	const Vector3 & halfSize() const;

	/// <summary> ��]���擾���܂��B </summary>
	const Quaternion & rotation() const;

	/// <summary> �t��]���擾���܂��B </summary>
	const Quaternion invRotation() const;

	/// <summary> �O�����̒P�ʃx�N�g�����擾���܂��B </summary>
	const Vector3 forward() const;

	/// <summary> �E�����̒P�ʃx�N�g�����擾���܂��B </summary>
	const Vector3 right() const;

	/// <summary> ������̒P�ʃx�N�g�����擾���܂��B </summary>
	const Vector3 up() const;

	/// <summary> ���S����O�̖ʂւ̃x�N�g�����擾���܂��B </summary>
	const Vector3 toForwardFace() const;

	/// <summary> ���S����E�̖ʂւ̃x�N�g�����擾���܂��B </summary>
	const Vector3 toRightFace() const;

	/// <summary> ���S�����̖ʂւ̃x�N�g�����擾���܂��B </summary>
	const Vector3 toUpFace() const;

	/// <summary> ���[���h���W���烍�[�J�����W�ɕϊ����܂��B </summary>
	const Vector3 inverseTransformPoint(const Vector3 point) const;

	/// <summary> ���[�J�����W���烏�[���h���W�ɕϊ����܂��B </summary>
	const Vector3 transformPoint(const Vector3 point) const;

	/// <summary> �_�܂ł̋������擾���܂��B </summary>
	float distance(const Vector3 & point, Vector3 * outClosestPoint = nullptr) const;

	/// <summary> �_�܂ł̍ŒZ�ƂȂ���W���擾���܂��B </summary>
	const Vector3 closestPoint(const Vector3 & point) const;

	/// <summary> ���s�ړ��E��]���Ă��Ȃ���Ԃ̒����̂ō��W���������񂾌��ʂ��擾���܂��B </summary>
	const Vector3 clampLocal(const Vector3 & position) const;

	/// <summary> �_�� OBB �͈͓̔��������ꍇ�����o���܂��B </summary>
	const Vector3 extrusionPoint(const Vector3 & point) const;

	/// <summary> OBB���\������������擾���܂��B </summary>
	const Segment segment(int axisIndex, int segIndex) const;

	/// <summary> OBB���\������������擾���܂��B </summary>
	const Segment segment(int index) const;

	const std::array<Segment, SegmentCount> segments() const;

	/// <summary> �C�ӂ̃x�N�g���Ƃ̓��ς��ł�������OBB��̒��_���擾���܂��B </summary>
	const Vector3 dotMinVertex(const Vector3 & v) const;

	/// <summary> �C�ӂ̃x�N�g���Ƃ̂Ȃ��Ȃ��ł�������OBB��̒��_���擾���܂��B </summary>
	const Vector3 dotMaxVertex(const Vector3 & v) const;

	float dotMin(const Vector3 & v) const;

	float dotMax(const Vector3 & v) const;

	/// <summary> �C�Ӄx�N�g���Ƃ̓��ς��ł��������n�_������OBB�̐������擾���܂��B </summary>
	const Segment dotMinSegment(int segAxis, const Vector3 & v) const;

	/// <summary> OBB�𕽍s�ړ� </summary>
	const OBB translate(const Vector3 & move) const;

	/// <summary> 
	/// <para>�ʂ��擾���܂��B</para>
	/// <para>�ʂ͖@���������猩�Ď��v���ɒ�`����Ă��܂��B</para>
	/// </summary>
	const Face<4> face(int index) const;

	const std::array<Face<4>, FaceCount> faces() const;

	/// <summary> ���_���擾���܂��B </summary>
	const Vector3 vertex(int index) const;

	/// <summary> ���_���ׂĂ��擾���܂��B </summary>
	const std::array<Vector3, VertexCount> vertices() const;

	/// <summary> ���g�����S�ɕ���AABB���擾���܂��B </summary>
	const AABB coverAABB() const;

	/// <summary> �C�ӎ��ɑ΂���OBB�𓊉e���܂��B </summary>
	float project(const Vector3 & axis) const;

private:

	Vector3 _center;
	Vector3 _halfSize;
	Quaternion _rotation;

};
