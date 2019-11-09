#pragma once

#include <LMath.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> ラインキャスト接触情報クラス </summary>
class LinecastContact {

public:

	/// <summary> 衝突位置を取得します。 </summary>
	const Vector3 getPosition() const;

	/// <summary> ラインの始点から衝突位置までの距離を取得します。 </summary>
	float getDistance() const;

	/// <summary> 衝突相手のIDを取得します。 </summary>
	ColliderID getOtherID() const;

	/// <summary> 衝突位置を設定します。 </summary>
	void setPosition(const Vector3 & position);

	/// <summary> 距離を設定します。 </summary>
	void setDistance(float distance);

	/// <summary> 衝突相手のIDを設定します。 </summary>
	void setOtherID(ColliderID otherID);

	LinecastContact();

private:

	Vector3		_position;
	float		_distance;
	ColliderID	_otherID;

};
