# collision

## 注意

ポートフォリオ用。他人のライブラリに依存しているのでコンパイル出来ません。

## 使用可能な形状

- 球
- カプセル
- OBB
- 地形(三角形の集合)
- 線(キャストのみ)

## 衝突判定のダブルディスパッチ

[CollisionDispatcher](/Include/Implementation/CollisionDispatcher.h)クラスにコライダーを2つ渡すと適切なコリジョン判定関数を呼び出す。

#### 実装
形状の種類をキーとした二次元配列で判定処理を予め保持しておく。
渡されたコライダーの形状の種類を使って二次元配列にアクセスする事で、適切な判定処理が呼ばれる。

## 衝突判定のフィルター

- グローバルなマトリクスによるフィルター
[CollisionFilter](/Include/CollisionFilter/CollisionFilter.h)インタフェースを実装し、
[CollisionManager](/Include/CollisionManager.h)の`setCollisionFilter()`メンバ関数から設定することで衝突判定のフィルタリングがされる。

- コライダーごとの個別のフィルター

## 二種類の判定

- 衝突判定のみ
- 衝突判定と衝突情報(座標・押出し)計算

## 衝突判定の組み合わせ最適化

- 八分木
- 均一格子法
