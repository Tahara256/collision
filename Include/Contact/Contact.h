#pragma once

#include <LMath.h>
#include <Collider/Collider/ColliderID.h>

/// <summary> 接触情報クラス </summary>
class Contact {

public:

	/// <summary> 衝突位置を取得します。 </summary>
	const Vector3 getPosition() const;

	/// <summary> 押し出しベクトルを取得します。 </summary>
	const Vector3 getExtrusionVector() const;

	/// <summary> 衝突相手のIDを取得します。 </summary>
	ColliderID getOtherID() const;

	/// <summary> 衝突位置を設定します。 </summary>
	void setPosition(const Vector3 & hitPosition);

	/// <summary> 押し出しベクトルを設定します。 </summary>
	void setExtrusionVector(const Vector3 & extrusionVector);

	/// <summary> 衝突相手のIDを設定します。 </summary>
	void setOtherID(ColliderID otherID);

private:

	Vector3		_hitPosition;
	Vector3		_extrusionVector;
	ColliderID	_otherID;

};
