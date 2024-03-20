---
title: "ROS2の各種識別名"
emoji: "📘"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、ROS2を理解する上で欠かせないノード名・トピック名等の各種識別名についてまとめています。

# 識別名

識別名の指定方法は、絶対指定（`/`で始まる文字列）と相対指定（`\`で始まらない文字列）の２つの方法があります。

| 識別名 | 絶対指定 | 相対指定 | 途中での`/` | 例 | 解説 |
| ---- | ---- | ---- | ---- | ---- | ---- |
| ノード名 | ×：不可 | 〇：可 | ×：不可 | nodeA | 必ず相対指定 |
| ノード名前空間 | 〇：可 | ×：不可 | 〇：可 | /nsX/nsY | 相対指定した場合は自動的に先頭に`/`が挿入され絶対指定とみなされます |
| 完全修飾ノード名 | 〇：可 | ×：不可 | 〇：可 | /nsX/nsY/nodeA | ノード名とノード名前空間を結合して生成されます |
| トピック名 | 〇：可 | 〇：可 | 〇：可 | local/topicN<br/>/global/topicM | 絶対指定・相対指定の両方が可能 |
| 完全修飾トピック名 | 〇：可 | ×：不可 | 〇：可 | /nsX/local/topicN<br/>/global/topicM | トピック名が相対指定の場合はノード名前空間を結合して生成 |