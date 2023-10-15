---
title: "ROS2を深く理解する：launchファイル編４　action"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 前提知識

- launchの概念
  - launchファイル中に、やりたい処理（＝アクション）をやりたい順序で記載する。アクションの中で最も基本的なものは、nodeを動かす為のexecutableを起動するアクション（Nodeアクション/LifecycleNodeアクション）
    - １つのexecutableから複数のnodeを起動する場合は、componentを用いることが推奨。その場合のlaunchファイルの書き方は下記
      - [Launching-composable-nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)


# 公式ドキュメント

- TBD

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

## ROS2のlaunchファイル
- Nodeアクション
  - executableを実行するのに使用する。名前はNodeアクションとなっているが、Nodeを作成しないexecutableの実行にも使用できる。
  - packageとexecutable_nameを指定することで、executableの実行を指示するのが基本。
  - 起動時の動きをコントロールする為の方法は下記の２つが通り通り
    - Nodeパラメータを与えて起動する（Nodeを作成するexecutableに限って有効）
    - executable引数を与えて起動する（Nodeを作成しないexecutableでも有効）
    - ROSシステム引数を与えて起動する
  - Nodeパラメータを与えるのは、parametersに値を指定する。
  - executable引数を与えるのは、argumentsに値を指定する。この値は、executable実行時のエントリポイント（C++であれば通常main関数）の引数に渡される。
  - ROSシステム引数を与えるのは、ros_argumentsに値を指定する。
    - ROSシステム引数とは、ros2 runコマンドで-ros-args以下に与えることができる引数。remappingやNodeパラメータの指定等。
    - しかし、Nodeアクションには、生でROSシステム引数を指定する代わりに、remappingやNodeパラメータを指定する為の方法が別途用意されているので、事実上明示的にros_argumentsを使う用途はほとんどない。
