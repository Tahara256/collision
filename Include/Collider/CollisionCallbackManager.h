#pragma once

#include <list>
#include <Collider/Collider/CollisionCallback.h>
#include <Collider/Collider/ColliderID.h>

class CollisionCallbackManager {

public:

	/// <summary> コンストラクタ </summary>
	CollisionCallbackManager();

	/// <summary> コライダー衝突中のコールバックを追加します。 </summary>
	void addOnCollisionStay(const OnCollisionCallback & onCollisionStay);

	/// <summary> コライダー衝突し始めたときのコールバックを追加します。 </summary>
	void addOnCollisionEnter(const OnCollisionCallback & onCollisionEnter);

	/// <summary> コライダーが衝突をやめたときのコールバックを追加します。 </summary>
	void addOnCollisionExit(const OnCollisionCallback & onCollisionExit);

	/// <summary> トリガーのコライダー衝突中のコールバックを追加します。 </summary>
	void addOnTriggerStay(const OnTriggerCallback & onTriggerStay);

	/// <summary> トリガーのコライダー衝突し始めたときのコールバックを追加します。 </summary>
	void addOnTriggerEnter(const OnTriggerCallback & onTriggerEnter);

	/// <summary> トリガーのコライダーが衝突をやめたときのコールバックを追加します。 </summary>
	void addOnTriggerExit(const OnTriggerCallback & onTriggerExit);

	void notifyOnCollisionStay(const Contact & info);
	void notifyOnCollisionEnter(const Contact & info);
	void notifyOnCollisionExit(const Contact & info);

	void notifyOnTriggerStay(const TriggerContact & info);
	void notifyOnTriggerEnter(const TriggerContact & info);
	void notifyOnTriggerExit(const TriggerContact & info);

private:

	/// <summary> コライダーの衝突関連のコールバックを追加します。 </summary>
	void addOnCollision(OnCollisionCallback & dest, const OnCollisionCallback & source);

	/// <summary> トリガーのコライダーの衝突関連のコールバックを追加します。 </summary>
	void addOnTrigger(OnTriggerCallback & dest, const OnTriggerCallback & source);

	OnCollisionCallback		_onCollisionStay;
	OnCollisionCallback		_onCollisionEnter;
	OnCollisionCallback		_onCollisionExit;

	OnTriggerCallback		_onTriggerStay;
	OnTriggerCallback		_onTriggerEnter;
	OnTriggerCallback		_onTriggerExit;

};
