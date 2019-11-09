# collision

## 注意

ポートフォリオ用。他人のライブラリに依存しているのでコンパイル出来ません。

## 使用可能な形状

- 球
- カプセル
- OBB
- 地形(三角形の集合)
- 線(キャストのみ)

## [Collider](/Include/Collider/Collider.h)クラス

衝突判定を行うオブジェクト。  
形状抽象クラス(`CollisionShape`)とワールド変換情報(`TransformQ`)を持っている。  
衝突関連のコールバックはここに登録する。
![クラス図](/uml/collider-class-diagram.png)

## [CollisionShape](/Include/CollisionShape/CollisionShape.h)クラス

衝突判定用の形状を抽象的に扱うためのクラス。  
ワールド変換情報は持たず、変換処理のみを持つ。  
そのため、同じ形状を使用する際には1つのインスタンスを共有することができる。
![クラス図](/uml/collision-shape-class-diagram.png)

## 衝突判定のダブルディスパッチ

[CollisionDispatcher](/Include/Implementation/CollisionDispatcher.h)クラスにコライダーを2つ渡すと適切なコリジョン判定関数を呼び出す。

#### 実装
形状の種類をキーとした二次元配列で判定処理を予め保持しておく。
渡されたコライダーの形状の種類を使って二次元配列にアクセスする事で、適切な判定処理が呼ばれる。

## 衝突判定のフィルター

- グローバルなフィルター  
[CollisionFilter](/Include/CollisionFilter/CollisionFilter.h)インタフェースを実装し、  
[CollisionManager](/Include/CollisionManager.h)の`setCollisionFilter()`メンバ関数から設定することで衝突判定のフィルタリングがされる。  

コライダーのレイヤーのマトリクスによる判定のフィルターは[NormalCollisionFilter](/Include/CollisionFilter/BuiltIn/NormalCollisionFilter.h)にで実装。

- コライダーごとの個別のフィルター  
[Collider](/Include/Collider/Collider.h)の`setCollisionMask()`メンバ関数からマスクを登録することで、個別に衝突判定をフィルタリングすることができる。

## 二種類の判定

- 衝突判定のみ
- 衝突判定と衝突情報(座標・押出し)計算

## 衝突判定の組み合わせ最適化

### 八分木  
登録されている全コライダー同士の衝突判定とラインキャストのブロードフェーズに使用。  
基本[まるぺけ](http://marupeke296.com/COL_3D_No15_Octree.html)さんを参考に実装。  
ラインキャストで必要になったAABBをトラバースする機能を追加で実装。  

#### 使い方
[LinearOctreeManager](/Include/Implementation/SpatialPartition/LinearOctreeManager.h)クラスにコライダーを追加後、  
`getCollisionPairList()`メンバ関数で衝突する可能性のあるコライダーの組み合わせリストが、  
`traverseAABB()`メンバ関数で指定AABBと衝突する可能性のあるコライダーのリストが取得できる。
![クラス図](/uml/spatial-partition-class-diagram.png)

### 均一格子法  
地形コライダーが持っている三角形群と、その他のコライダーの衝突判定のブロードフェーズに使用。  
地形データ(テクスチャ)を均一格子と見立てて、AABBと重なる部分のテクスチャ情報を取得し、三角形を作成。  
その三角形とのみ衝突判定を行う。

