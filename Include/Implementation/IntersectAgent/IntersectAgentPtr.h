#pragma once

#include <memory>

/// <summary> 衝突処理代理クラス </summary>
class IntersectAgent;

/// <summary> 衝突処理代理クラスへのポインタ </summary>
using IntersectAgentPtr = std::shared_ptr<IntersectAgent>;
