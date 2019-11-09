#include <Collider/Segment.h>
#include <Collider/Implementation/CollisionUtility.h>
#include <Collider/Line.h>
#include <Collider/AABB.h>

using namespace CollisionUtility;

// [0, 1]�Ŋۂ߂�
static float Saturate(float value) {
	return clamp(value, 0.0f, 1.0f);
}

Segment::Segment(const Vector3 & start, const Vector3 & end) :
	_start(start),
	_end(end) {
}

const Vector3 Segment::start() const {
	return _start;
}

const Vector3 Segment::end() const {
	return _end;
}

const Vector3 Segment::midpoint() const {
	return start() + halfVector();
}

const Vector3 Segment::vector() const {
	return end() - start();
}

const Vector3 Segment::halfVector() const {
	return vector() / 2.0f;
}

const Vector3 Segment::unitVector() const {
	return vector().normalize();
}

float Segment::length() const {
	return vector().length();
}

float Segment::sqrLength() const {
	return vector().sqrLength();
}

const Vector3 Segment::pointFromDistance(float distance) const {
	return start() + unitVector() * distance;
}

const Line Segment::asLine() const {
	return { start(), vector() };
}

const Segment Segment::translate(const Vector3 & vector) const {
	return { start() + vector, end() + vector };
}

const Segment Segment::rotate(const Quaternion & rotation, const Vector3 & origin) const {
	return translate(-origin).rotateOrigin(rotation).translate(origin);
}

const Segment Segment::rotateOrigin(const Quaternion & rotation) const {
	return { Quaternion::rotVector(rotation, start()), Quaternion::rotVector(rotation, end()) };
}

const Segment Segment::rotateStart(const Quaternion & rotation) const {
	return rotate(rotation, start());
}

void Segment::closest(const Segment & other, Vector3 * outMyShortest, Vector3 * outOtherShortest) const {
	decltype(auto) s1 = start();
	decltype(auto) s2 = other.start();

	// ���g���k�ނ��Ă���H
	if (sqrLength() < Mathf::Eps) {
		// ������k�ށH
		if (other.sqrLength() < Mathf::Eps) {
			// �_�Ɠ_�̋����̖��ɋA��
			*outMyShortest = s1;
			*outOtherShortest = s2;
			return;
		}
		else {
			// ���g�̎n�_�Ƒ���̍ŒZ���ɋA��
			*outMyShortest = s1;
			*outOtherShortest = other.closest(s2);
			return;
		}
	}
	// ���肪�k�ނ��Ă���H
	else if (other.sqrLength() < Mathf::Eps) {
		// ����̎n�_�Ǝ��g�̍ŒZ���ɋA��
		*outMyShortest = closest(s1);
		*outOtherShortest = s1;
		return;
	}

	decltype(auto) e1 = end();
	decltype(auto) e2 = other.end();
	auto p1 = Vector3();
	auto p2 = Vector3();

	// �������m�����s�������ꍇ
	if (isParallel(other)) {
		// ���s�Ȃ̂Ő����Ԃɐ�����������Ȃ炻�ꂪ�ŒZ����

		// s1���琂���������邩�m�F
		if (other.inRangePerpendicularFoot(s1, outOtherShortest)) {
			*outMyShortest = s1;
			return;
		}

		// s2���琂���������邩�m�F
		if (inRangePerpendicularFoot(s2, outMyShortest)) {
			*outOtherShortest = s2;
			return;
		}

		// e1���琂���������邩�m�F
		if (other.inRangePerpendicularFoot(e1, outOtherShortest)) {
			*outMyShortest = e1;
			return;
		}

		// �����Ԃɐ����������Ȃ��̂ŁA�ŒZ�����̐����̖��[��������
		closestEnd(other, outMyShortest, outOtherShortest);
		return;
	}

	// ���s����Ȃ������ꍇ
	// �������m�̍ŒZ����������_���擾
	asLine().closest(other.asLine(), &p1, &p2);

	// ������̈ʒu��[0, 1]�Ōv�Z
	const auto t1 = Vector3::dot(unitVector(), p1 - s1) / length();
	const auto t2 = Vector3::dot(other.unitVector(), p2 - s2) / other.length();

	// ����1��̍ŒZ����������1��ɑ��݂��邩
	const auto inRangeLine1 = inRange01(t1);
	// ����2��̍ŒZ����������2��ɑ��݂��邩
	const auto inRangeLine2 = inRange01(t2);

	// �ŒZ������������ɂ���ꍇ
	if (inRangeLine1 && inRangeLine2) {
		*outMyShortest = p1;
		*outOtherShortest = p2;
		return;
	}

	// �����ザ��Ȃ��ꍇ�����͈͓��Ɋۂ߁A�ēx��������
	p1 = s1 + vector() * Saturate(t1);
	p2 = s2 + other.vector() * Saturate(t2);

	// �Е��ɐ�����������Ȃ狅�ƃJ�v�Z���̏Փ˔���
	if (inRangeLine1) {
		*outMyShortest = closest(p2);
		*outOtherShortest = p2;
		return;
	}
	if (inRangeLine2) {
		*outMyShortest = p1;
		*outOtherShortest = other.closest(p1);
		return;
	}

	// �ǂ������O��Ă���ꍇ
	// �����̓_��������Е��̐����֐����������邩����
	// �������ꍇ�������ŒZ�����Ƃ���
	auto tp2 = Vector3();
	if (other.inRangePerpendicularFoot(p1, &tp2)) {
		*outMyShortest = p1;
		*outOtherShortest = tp2;
		return;
	}
	auto tp1 = Vector3();
	if (inRangePerpendicularFoot(p2, &tp1)) {
		*outMyShortest = tp1;
		*outOtherShortest = p2;
		return;
	}

	// �ǂ����������Ȃ��ꍇ�A�ŒZ�̒[�_���m�ŋ��̏Փ˔���
	closestEnd(other, &p1, &p2);
	*outMyShortest = p1;
	*outOtherShortest = p2;
}

void Segment::closestEnd(const Segment & other, Vector3 * myEnd, Vector3 * otherEnd) const {
	auto s1 = start();
	auto s2 = other.start();
	auto e1 = end();
	auto e2 = other.end();

	auto s1s2d = Vector3::distance(s1, s2);
	auto minD = s1s2d;
	*myEnd = s1;
	*otherEnd = s2;

	auto e1e2d = Vector3::distance(e1, e2);
	if (e1e2d < minD) {
		minD = e1e2d;
		*myEnd = e1;
		*otherEnd = e2;
	}

	auto s1e2d = Vector3::distance(s1, e2);
	if (s1e2d < minD) {
		minD = s1e2d;
		*myEnd = s1;
		*otherEnd = e2;
	}

	auto e1s2d = Vector3::distance(e1, s2);
	if (e1s2d < minD) {
		minD = e1s2d;
		*myEnd = e1;
		*otherEnd = s2;
	}
}

const Vector3 Segment::perpendicularFoot(const Vector3 & point) const {
	return asLine().perpendicularFoot(point);
}

bool Segment::inRangePerpendicularFoot(const Vector3 & point, Vector3 * outFootPoint) const {
	auto p = perpendicularFoot(point);
	if (outFootPoint) *outFootPoint = p;
	auto t = Vector3::dot(unitVector(), p - start()) / length();
	return inRange01(t);
}

bool Segment::isParallel(const Segment & segment) const {
	return asLine().isParallel(segment.asLine());
}

float Segment::distance(const Vector3 & point, Vector3 * outClosestPoint) const {
	return std::sqrtf(sqrDistance(point, outClosestPoint));
}

float Segment::sqrDistance(const Vector3 & point, Vector3 * outClosestPoint) const {
	decltype(auto) clo = closest(point);
	return Vector3::sqrDistance(clo, point);
}

bool Segment::isPlaneOutside(const Vector3 & planeNormal, const Vector3 & planePoint) const {
	if (Vector3::dot(planeNormal, start() - planePoint) < 0.0f) return false;
	if (Vector3::dot(planeNormal, end() - planePoint) < 0.0f) return false;
	return true;
}

const AABB Segment::coverAABB() const {
	decltype(auto) s = start();
	decltype(auto) e = end();

	auto minX = Mathf::cmin(s.x, e.x);
	auto minY = Mathf::cmin(s.y, e.y);
	auto minZ = Mathf::cmin(s.z, e.z);
	auto minP = Vector3(minX, minY, minZ);

	auto maxX = Mathf::cmax(s.x, e.x);
	auto maxY = Mathf::cmax(s.y, e.y);
	auto maxZ = Mathf::cmax(s.z, e.z);
	auto maxP = Vector3(maxX, maxY, maxZ);

	return { minP, maxP };
}

const Vector3 Segment::closest(const Vector3 & point) const {
	decltype(auto) s = start();
	decltype(auto) e = end();
	const auto startToPoint = point - s;
	const auto endToPoint = point - e;
	decltype(auto) v = vector();

	const auto dotS = Vector3::dot(v, startToPoint);
	// �_���n�_��
	if (dotS < 0.0f) return s;

	const auto dotE = Vector3::dot(-v, endToPoint);
	// �_���I�_��
	if (dotE < 0.0f) return e;

	// �_�������̓���
	return perpendicularFoot(point);
}
