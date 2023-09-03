---
title: "（書きかけ）ROS2のリモート開発環境の構築"
emoji: "🧱"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、ROS2を用いたシステム開発をリモートで行う場合のノウハウを記載します。
ここでいうリモート開発は下記を意味します。

- コーディングやGUI確認するのは手元のPC
  - rviz2やgazeboも動かしたいよね！
- 実際のROS2プログラムが動くのはインターネットを介してアクセスするサーバ上
  - 複数人が同じサーバにアクセスしてチーム開発することにも対応

ストレスなく開発するにはどうしたらよいか？ノウハウをまとめます。

# 前提
- ROS2 humbleで確認していますが、特にdistributionに依存しない内容（のはず）

# 前提知識
- dockerについて知っていること
- sshについて知っていること
- x11転送について知っていること

# 公式ドキュメント

# docker + ROS環境の構築

- 下記をみればGUIを使う前提でどのようにdockerを構成・使用すればいいのかわかる
    - [公式情報](http://wiki.ros.org/docker/Tutorials/GUI)
- 上記公式で書かれている様々な方法の違いを理解するのに役立つ参考情報
    - [Dockerでuid/gid指定可能かつsudo実行可能なユーザとしてコンテナを起動する](https://qiita.com/yama07/items/a521234dc91f923ba655)
- 上記公式で紹介されている「docker上のROSにVNCで接続」したいを手っとリ速くやるには、下記の非公式イメージを使うと良さそう
    - [tiryoh/ros2-desktop-vnc](https://hub.docker.com/r/tiryoh/ros2-desktop-vnc)
- 公式では紹介ない方法として「docker上のROSにRDPで接続」したい場合は、下記のイメージにROSを追加すればよいかも
    - [docker-ubuntu-lxde XRDP](https://hub.docker.com/r/yama07/docker-ubuntu-lxde)
- でもやっぱり公式のROSイメージ使いたいよね？という場合
