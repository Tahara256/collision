#pragma once

#include <Collider/Contact/Contact.h>
#include <Util/RefPtr.h>

class ColliderComponent;

/// <summary> �Փˏ��N���X </summary>
class Collision {

public:

	/// <summary> �Փˑ�����擾���܂��B </summary>
	const RefPtr<ColliderComponent> getOther() const;

	/// <summary> �ڐG�����擾���܂��B </summary>
	const Contact getContact() const;

	/// <summary> �R���X�g���N�^ </summary>
	/// <param name="contact"> �ڐG��� </param>
	explicit Collision(const Contact & contact);

private:

	Contact _contact;

};
