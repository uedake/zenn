---
title: "ROS2を深く理解する：launchファイル編６　ComposableNodeContainerアクション"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象
本記事では、ROS2のlaunch機能が提供するComposableNodeContainerアクションを解説します。

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 公式ドキュメント
- [Launching-composable-nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)
  - componentを用いて１つのexecutableから複数のノードを起動する方法が解説されている

# 解説


# （参考）ソースの確認

