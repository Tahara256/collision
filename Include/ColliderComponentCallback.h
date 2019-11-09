#pragma once

#include <functional>
#include <Util/RefPtr.h>

class Collision;
class ColliderComponent;

/// <summary> �Փˎ��R�[���o�b�N </summary>
using OnCollision = std::function<void(const Collision &)>;

/// <summary> �g���K�[�Փˎ��R�[���o�b�N </summary>
using OnTrigger = std::function<void(RefPtr<ColliderComponent>)>;
