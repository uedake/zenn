---
title: "（書きかけ）ROS2を深く理解する：launchファイル編"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

    - １つのnodeを動かすexecutableを起動するには、launchファイル中でNodeアクション/LifecycleNodeアクション(python形式)もしくはNodeタグ/LifecycleNodeタグ（xml形式）を用いて指示する。
      - NodeアクションやNodeタグは複数のNodeを動かすexecutableの起動を想定していない（例えば複数Nodeのname変更に対応していない）
    - １つのexecutableから複数のNodeを起動する場合は、componentを用いることが推奨。その場合のlaunchファイルの書き方は下記
      - [Launching-composable-nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)


## ROS2のlaunchファイル
- python形式のlaunchファイル
    - アクション（＝launch時に実行される処理）を実行順に記述する。
    - ビルド時変数とlaunch時変数の違いに注意する事
        - launchファイル中で記述する通常の変数はビルド時変数であり、colcon build実行時に確定する。
        - LaunchConfiguration関数の戻り値として得られる変数（substitution変数）はlaunch時変数であり、値はlaunch時に確定する。
    - launch時変数の使い方
        - LaunchConfiguration関数に変数名とデフォルト値を指定し、substitution変数を得る
        - 値の書き込み方法
            - launchファイル内で書き込む
                - SetLaunchConfigurationアクションを使用する
                    - SetLaunchConfiguration("変数名",値)　で書き込む
                    - 変数名=値　と書けないので見にくい
            - ユーザがlaunch時に引数として指定できるようにする
                - DeclareLaunchArgumentアクションを使用する
        - launch時変数は、global変数のように振る舞うので名前の衝突に注意すること
            - launchファイルから他のlaunchファイルをincludeした場合、これらのlaunchファイル間で同名のlaunch時変数は共有される
        - 多くの場合、LaunchConfigurationはDeclareLaunchArgumentアクションとセットで用いる。
        - DeclareLaunchArgumentアクションを伴わずにLaunchConfiguration単独で用いる用途
            - 他のlaunch時変数から計算される値を一時的に格納するlaunch時変数を作る場合
            - 再帰的に自分自身をincludeするlaunchファイルの制御を行う為の変数を作る場合
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

