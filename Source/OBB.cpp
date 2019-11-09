#include <Collider/OBB.h>

#include <cassert>

#include <Collider/Segment.h>
#include <Collider/AABB.h>

using namespace std;

OBB::OBB(const Vector3 & center, const Vector3 & size, const Quaternion & rotation) :
	_center(center),
	_halfSize(size / 2.0f),
	_rotation(rotation) {
}

const Vector3 & OBB::center() const {
	return _center;
}

const Vector3 OBB::size() const {
	return halfSize() * 2.0f;
}

const Vector3 & OBB::halfSize() const {
	return _halfSize;
}

const Quaternion & OBB::rotation() const {
	return _rotation;
}

const Quaternion OBB::invRotation() const {
	return rotation().inverse();
}

const Vector3 OBB::forward() const {
	return Quaternion::rotVector(rotation(), Vector3::forward);
}

const Vector3 OBB::right() const {
	return Quaternion::rotVector(rotation(), Vector3::right);
}

const Vector3 OBB::up() const {
	return Quaternion::rotVector(rotation(), Vector3::up);
}

const Vector3 OBB::toForwardFace() const {
	return forward() * halfSize().z;
}

const Vector3 OBB::toRightFace() const {
	return right() * halfSize().x;
}

const Vector3 OBB::toUpFace() const {
	return up() * halfSize().y;
}

const Vector3 OBB::inverseTransformPoint(const Vector3 point) const {
	return Quaternion::rotVector(invRotation(), point - center());
}

const Vector3 OBB::transformPoint(const Vector3 point) const {
	return Quaternion::rotVector(rotation(), point) + center();
}

float OBB::distance(const Vector3 & point, Vector3 * outClosestPoint) const {
	const auto lp = inverseTransformPoint(point);

	if (outClosestPoint) {
		const auto local = clampLocal(lp);
		*outClosestPoint = transformPoint(local);
	}

	const auto hs = halfSize();

	// 各ローカル軸でのはみ出し分を計算
	const auto ox = Mathf::cmax(Mathf::abs(lp.x) - hs.x, 0.0f);
	const auto oy = Mathf::cmax(Mathf::abs(lp.y) - hs.y, 0.0f);
	const auto oz = Mathf::cmax(Mathf::abs(lp.z) - hs.z, 0.0f);

	// はみ出し分のベクトルの長さがはみ出した距離
	return Vector3(ox, oy, oz).length();
}

const Vector3 OBB::closestPoint(const Vector3 & point) const {
	const auto lp = inverseTransformPoint(point);
	const auto local = clampLocal(lp);
	return transformPoint(local);
}

const Vector3 OBB::clampLocal(const Vector3 & position) const {
	const auto hs = halfSize();
	return { clamp(position.x, -hs.x, hs.x), clamp(position.y, -hs.y, hs.y), clamp(position.z, -hs.z, hs.z) };
}

const Vector3 OBB::extrusionPoint(const Vector3 & point) const {
	decltype(auto) local = inverseTransformPoint(point);
	const auto forward = Vector3::forward;
	const auto right = Vector3::right;
	const auto up = Vector3::up;

	const auto xSign = local.x > 0.0f;
	const auto xDistance = _halfSize.x - local.x * (xSign ? 1 : -1);

	// 0以下なら既に範囲外
	if (xDistance < 0.0f) return point;

	// 最短で押し出せる距離と軸を探す
	auto minD = xDistance;
	auto minAxis = right * (xSign ? 1.0f : -1.0f);

	const auto ySign = local.y > 0.0f;
	const auto yDistance = _halfSize.y - local.y * (ySign ? 1.0f : -1.0f);
	// 最短をy軸に更新
	if (yDistance < 0.0f) return point;
	if (yDistance < minD) {
		minD = yDistance;
		minAxis = up * (ySign ? 1.0f : -1.0f);
	}

	const auto zSign = local.z > 0.0f;
	const auto zDistance = _halfSize.z - local.z * (zSign ? 1.0f : -1.0f);
	// 最短をz軸に変更
	if (zDistance < 0.0f) return point;
	if (zDistance < minD) {
		minD = zDistance;
		minAxis = forward * (zSign ? 1.0f : -1.0f);
	}

	// ローカル座標系で押し出してワールド座標系に戻して返す
	return transformPoint(local + minAxis * minD);
}

Segment const OBB::segment(int axisIndex, int segIndex) const {
	auto segAxis = Vector3();
	auto axis1 = Vector3();
	auto axis2 = Vector3();

	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();

	switch (axisIndex) {
	case 0:
		segAxis = r;
		axis1 = u;
		axis2 = f;
		break;
	case 1:
		segAxis = u;
		axis1 = r;
		axis2 = f;
		break;
	case 2:
		segAxis = f;
		axis1 = u;
		axis2 = r;
		break;
	default:
		assert(!"axisIndexには 0～2 の整数を渡してください。");
		return {};
	}

	decltype(auto) c = center();

	switch (segIndex) {
	case 0:
	{
		const auto t = c + axis1 + axis2;
		return { t + segAxis, t - segAxis };
	}
	case 1:
	{
		const auto t = c + axis1 - axis2;
		return { t + segAxis, t - segAxis };
	}
	case 2:
	{
		const auto t = c - axis1 + axis2;
		return { t + segAxis, t - segAxis };
	}
	case 3:
	{
		const auto t = c - axis1 - axis2;
		return { t + segAxis, t - segAxis };
	}
	default:
		assert(!"segIndexには 0～3 の整数を渡してください。");
		return {};
	}
}

const Segment OBB::segment(int index) const {
	return segment(index / 4, index % 4);
}

const std::array<Segment, OBB::SegmentCount> OBB::segments() const {
	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();
	decltype(auto) c = center();

	const auto cr = c + r;
	const auto cl = c - r;

	const auto cru = cr + u;
	const auto crd = cr - u;

	const auto clu = cl + u;
	const auto cld = cl - u;

	const auto cruf = cru + f;
	const auto crub = cru - f;

	const auto crdf = crd + f;
	const auto crdb = crd - f;

	const auto cluf = clu + f;
	const auto club = clu - f;

	const auto cldf = cld + f;
	const auto cldb = cld - f;

	return {
		Segment(cruf, cluf),
		Segment(crub, club),
		Segment(crdf, cldf),
		Segment(crdb, cldb),

		Segment(cruf, crdf),
		Segment(crub, crdb),
		Segment(cluf, cldf),
		Segment(club, cldb),

		Segment(cruf, crub),
		Segment(crdf, crdb),
		Segment(cluf, club),
		Segment(cldf, cldb),
	};
}

const Vector3 OBB::dotMinVertex(const Vector3 & vec) const {
	auto minDot = numeric_limits<float>::max();
	auto minVertex = Vector3();

	decltype(auto) vertexArray = vertices();

	for (auto i = 0; i < VertexCount; i++) {
		decltype(auto) v = vertexArray[i];
		const auto tempDot = Vector3::dot(vec, v);

		if (tempDot > minDot) continue;

		minDot = tempDot;
		minVertex = v;
	}

	return minVertex;
}

const Vector3 OBB::dotMaxVertex(const Vector3 & vec) const {
	auto maxDot = -numeric_limits<float>::max();
	auto maxVertex = Vector3();

	decltype(auto) vertexArray = vertices();

	for (auto i = 0; i < VertexCount; i++) {
		decltype(auto) v = vertexArray[i];
		const auto tempDot = Vector3::dot(vec, v);

		if (tempDot < maxDot) continue;

		maxDot = tempDot;
		maxVertex = v;
	}

	return maxVertex;
}

float OBB::dotMin(const Vector3 & v) const {
	decltype(auto) min = dotMinVertex(v);
	return Vector3::dot(v, min);
}

float OBB::dotMax(const Vector3 & v) const {
	decltype(auto) max = dotMaxVertex(v);
	return Vector3::dot(v, max);
}

const Segment OBB::dotMinSegment(int segAxis, const Vector3 & v) const {
	auto minDot = numeric_limits<float>::max();
	auto minSeg = Segment();

	constexpr auto SegmentCountPerAxis = SegmentCount / 3;

	for (auto i = 0; i < SegmentCountPerAxis; i++) {
		decltype(auto) seg = segment(segAxis, i);
		const auto tempDot = Vector3::dot(v, seg.start());

		if (tempDot > minDot) continue;

		minDot = tempDot;
		minSeg = seg;
	}

	return minSeg;
}

const OBB OBB::translate(const Vector3 & move) const {
	return { center() + move, size(), rotation() };
}

const array<Vector3, 4> OBB::face(int index) const {
	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();
	decltype(auto) c = center();

	switch (index) {
	case 0:
	{
		// 前面
		const auto cf = c + f;
		const auto cfu = cf + u;
		const auto cfd = cf - u;
		return { cfu + r, cfu - r, cfd - r, cfd + r };
	}
	case 1:
	{
		// 後面
		const auto cb = c - f;
		const auto cbu = cb + u;
		const auto cbd = cb - u;
		return { cbu - r, cbu + r, cbd + r,	cbd - r };
	}
	case 2:
	{
		// 上面
		const auto cu = c + u;
		const auto cuf = cu + f;
		const auto cub = cu - f;
		return { cuf - r, cuf + r, cub + r, cub - r };
	}
	case 3:
	{
		// 下面
		const auto cd = c - u;
		const auto cdf = cd + f;
		const auto cdb = cd - f;
		return { cdf + r, cdf - r, cdb - r, cdb + r };
	}
	case 4:
	{
		// 右面
		const auto cr = c + r;
		const auto cru = cr + u;
		const auto crd = cr - u;
		return { cru - f, cru + f, crd + f, crd - f };
	}
	case 5:
	{
		// 左面
		const auto cl = c - r;
		const auto clu = cl + u;
		const auto cld = cl - u;
		return { clu + f, clu - f, cld - f, cld + f };
	}
	default:
		assert(!"引数には 0～5 の整数を渡してください。");
		return{};
	}
}

const std::array<std::array<Vector3, 4>, OBB::FaceCount> OBB::faces() const {
	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();
	decltype(auto) c = center();

	const auto cr = c + r;
	const auto cl = c - r;

	const auto cru = cr + u;
	const auto crd = cr - u;

	const auto clu = cl + u;
	const auto cld = cl - u;

	const auto cruf = cru + f;
	const auto crub = cru - f;

	const auto crdf = crd + f;
	const auto crdb = crd - f;

	const auto cluf = clu + f;
	const auto club = clu - f;

	const auto cldf = cld + f;
	const auto cldb = cld - f;

	return {
		// 前面
		std::array<Vector3, 4>{ cruf, cluf, cldf, crdf },
		// 後面
		std::array<Vector3, 4>{ club, crub, crdb, cldb },
		// 上面
		std::array<Vector3, 4>{ cluf, cruf, crub, club },
		// 下面
		std::array<Vector3, 4>{ crdf, cldf, cldb, crdb },
		// 右面
		std::array<Vector3, 4>{ crub, cruf, crdf, crdb },
		// 左面
		std::array<Vector3, 4>{ cluf, club, cldb, cldf }
	};
}

const Vector3 OBB::vertex(int index) const {
	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();

	switch (index) {
	case 0:	return center() + r + u + f;
	case 1:	return center() + r + u - f;
	case 2:	return center() + r - u + f;
	case 3:	return center() + r - u - f;
	case 4:	return center() - r + u + f;
	case 5:	return center() - r + u - f;
	case 6:	return center() - r - u + f;
	case 7:	return center() - r - u - f;
	default:
		assert(!"引数には 0～7 の整数を渡してください。");
		return {};
	}
}

const std::array<Vector3, OBB::VertexCount> OBB::vertices() const {
	decltype(auto) r = toRightFace();
	decltype(auto) u = toUpFace();
	decltype(auto) f = toForwardFace();
	decltype(auto) c = center();

	const auto cr = c + r;
	const auto cl = c - r;

	const auto cru = cr + u;
	const auto crd = cr - u;

	const auto clu = cl + u;
	const auto cld = cl - u;

	return { cru + f, cru - f, crd + f,	crd - f, clu + f, clu - f, cld + f, cld - f };
}

const AABB OBB::coverAABB() const {
	decltype(auto) c = center();
	const auto x = project(Vector3::right);
	const auto y = project(Vector3::up);
	const auto z = project(Vector3::forward);
	const auto v = Vector3(x, y, z);
	return { c - v, c + v };
}

float OBB::project(const Vector3 & axis) const {
	return Mathf::abs(Vector3::dot(axis, toForwardFace())) + Mathf::abs(Vector3::dot(axis, toRightFace())) + Mathf::abs(Vector3::dot(axis, toUpFace()));
}
