---
title: "ROS2を理解する上でのポイント"
emoji: "📘"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-09-03 22:13"
---

# 解説対象

本記事では、ROS2を理解する上で頭にいれておいた方がよい基本概念を説明します。

# 前提
- 調査はROS2 humble時の実装に基づいていますが、基本的にdistributionに依存しない内容です。
- 調査はc++側の実装（rclcpp）に基づいていますが、基本的にクライアント言語に依存しない内容です。

# 公式ドキュメント

- [Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)
  - かなりあっさりで物足りません

# 参考情報

- [ROS2に関する基本情報](https://qiita.com/NeK/items/18ff9e9c443295d5d805)
  - よくまとまっています
- [駆け抜けるROS2](https://zenn.dev/hakuturu583/articles/ros2_turtorial)
  - ROS2ではnodeをcomponent形式で開発すべきことが解説されている
- [ROS 2のAPIの使い方](https://gbiggs.github.io/rosjp_ros2_intro/ros2_basics.html)
  - node周りの詳細な解説がありますが、特にexecutorの理解の参考になります
- [ROS 2のプロセス内（intra-process）通信を理解する](https://www.youtalk.jp/2017/06/05/ros2-intra-process.html)
  - パフォーマンスを上げるのに理解必須のプロセス内通信についての参考になります
- [ROS2でプロセス内通信によるゼロコピーを試す](https://qiita.com/shigeharu_shibahata/items/0c5fb3963150403af57c)
  - パフォーマンスを上げるにはゼロコピーの理解も必要

# ROS2を理解する上でのポイント

- 概念として、配布単位（package）、プログラム実行単位（executable/library）、論理処理単位（node）をちゃんと区別して把握することが重要です。
- ROS2を使用して作られるソフトウェアは、下記の構成をとります。
  - ソフトウェアは、ファイル群としての存在であるpackageを単位として配布されます
    - packageは、機能を実現する為の一連のプログラム（executable/library）ファイルや各種データファイル一式を提供します
    - 通常ロボットシステムは、複数のpackageを組み合わせて実現します
  - ソフトウェアは、ファイルとしての存在であるexecutableもしくはlibraryを単位として実行されます
    - executableは、直接実行可能なプログラムを提供します。基本的にはnodeを構築し動作させる役目を果たします。
    - libraryは、他のexecutableから読み込まれて間接的に実行されるプログラムを提供します。基本的にはnodeを他のexecutableにロードする役目を果たします。
    - 通常ロボットシステムは、複数のpackageに含まれる複数のexecutable（やlibraryをロードしたexecutable）を実行することで動作します。
  - ソフトウェアは、論理的な存在であるnodeを単位として処理を行います
    - nodeは、情報を所定のルールに従って入力・処理・出力する機能を提供します
    - 通常ロボットシステムは、複数のpackageに含まれる複数のexecutable（やlibraryをロードしたexecutable）が実行されることで複数のnodeが構築され動作します。

# ROS2におけるプログラミングとは

ROS2におけるプログラミングは、カジュアルな（＝ほとんどプログラミングしない）スタイルからゴリゴリの大規模開発スタイルまで、幅広く使える汎用性と拡張性を備えています。カジュアルな順番に開発スタイルを並べてみると下記のような雰囲気です。

| スタイル | 独自パッケージの開発 | 独自executableの開発 | カスタムnodeの開発 | 説明 |
| ---- | ---- | ---- | ---- | ---- | 
| 既存パッケージ利用のみ | × | × | × | 既存パッケージで足りる場合は、パッケージを利用する為の設定（launchファイル、Nodeパラメータを指定する為のconfigファイル）だけをつくれば良いです。作成したlaunchファイル等は必ずしもパッケージ化しなくても構いません |
| launchパッケージ開発のみ | 〇 | × | × | ほとんど上記と変わりませんが、launchファイルを誰でも使えるように配布する場合パッケージ化しましょう|
| 独自処理パッケージ開発（シンプル） | 〇 | 〇 | × | 実際には多くのケースで既存パッケージの利用だけでは望むシステムは作れないので、独自処理を行えるようexecutableを作成します。独自処理を実装するにあたり、ROS2が提供するNodeやLifecycleNodeを使うだけどうにかなる場合はカスタムノードを作成する必要はありません。しかし、独自処理がnodeの中にカプセル化されずexecutable上に実装されることになるため、複雑なシステムには向きません|
| 独自処理パッケージ開発（カスタムノードあり） | 〇 | 〇 | 〇 | 複雑なシステムをつくる場合、ROS2が提供するNodeやLifecycleNodeを直接使うのでなく、それらを継承したカスタムnodeを実装し使用したくなります。カスタムnodeを作れば独自処理をnodeの中に埋め込め、オブジェクト指向で機能を実装でき、見通しが良くなります。|

# 重要概念

## nodeとは

- nodeは、ROS2上で実行する処理を担うオブジェクト（論理処理単位）です
- 標準nodeとカスタムnode
  - ros2には、標準node（NodeもしくはLifecycleNodeというクラス）が定義されています。独自の処理を持ったnodeを作るためにこれら標準nodeをそのまま使用してもよいですが、標準nodeを継承したnode（＝カスタムnode）を作成してもよいです
  - 使い分け
    - 独自処理をnodeの中にカプセル化したいならカスタムnodeを作成します
    - ライフサイクルイベントのハンドリングを行いたいならカスタムnodeを作成します
    - どちらも不要であれば、NodeもしくはLifecycleNodeを直接用います
      - ただしこの場合独自処理（例えば、nodeにservice serverを追加しserviceがコールされた時の処理を実装する）は、nodeクラス外（例えばexecutableで実行される関数中）に記述することになり、object指向の考え方からすると全容を捉えにくい構成になることに注意
  - カスタムnodeを作成する場合のベストプラクティス
    - カスタムnodeの定義は、executable中で定義することもlibrary中で定義することもどちらも可能であるが、下記を推奨
      - library中でcomponent形式で定義する
      - executable側からlibrary中のカスタムnodeをロードして利用する
    - カスタムnodeを特定のexecutableに密結合させずに分離することで柔軟な運用が可能になります

### nodeを動かす手段

- nodeは必ずexecutor上で動く
  - rclcpp:spin(node)で実行した場合でも、内部でSingleThreadedExecutorが生成されその中でnodeが動いています
- nodeを動かすexecutorの実行を開始する方法は以下のどちらか
  - nodeを動かす処理を記述したexecutableを実行することでnodeの処理を開始する
  - nodeを動かす処理を記述したlibraryをロードするexecutableを実行することでnodeの処理を開始する

例えば、c++から他のプログラムファイルで定義されたカスタムノード（my_namespace::MyNode）を動かす場合

```cpp
rclcpp::executors::SingleThreadedExecutor exec;
auto my_node = std::make_shared<my_namespace::MyNode>(options);
exec.add_node(my_node);
```

### node間の通信方法
nodeは他nodeや外部プロセスと情報をやりとりする為の仕組みを有する


- 仕組みは、topic通信・service通信・action通信・nodeパラメータ通信の４種類
  - 全て「通信」であり、異なるプロセス上・異なるマシン上ののnode間で情報をやりとりできる（プロセス間通信）
  - ※ros2_contorolで使用するcommand interfaceやstate interfaceは通信ではない（pluginによる結合である）
- これらの通信は、プロセス間通信・プロセス内通信のどちらになるか？
  - 異なるプロセスにあるnode間の通信
    - プロセス間通信になる
  - 同一プロセス内にあるnode間の通信
    - componentとして実装して、executer上で実行した場合には自動的にプロセス内通信となる模様
    - 自動的にプロセス内通信になるわけではない？（未確認）
- プロセス内通信のメリット
  - プロセス間通信のようなエンコード・TCP通信・デコードが発生しない
  - さらに、ひと手間加えると通信メッセージをメモリ上でコピーすることも避けること（＝ゼロコピー）も可能
  - 詳しくは「参考情報」内のリンクを参照

nodeが持つ通信IFによる情報のやりとりを全部羅列すると

- nodeに外部から入ってくる情報
  - 能動的
    - 他nodeのパラメータをリードして値を得る
    - 他nodeのserviceをコールし結果を得る
    - 他nodeのactionをコールしフィードバックと結果を得る
  - 受動的
    - subscribeしているtopicからROSメッセージを得る
    - 他nodeや外部プロセスからnodeパラメータの初期値を設定される／書き換えられる
    - 他nodeや外部プロセスからのserviceをコールされ引数を受け取る
    - 他nodeや外部プロセスからのactionをコールされゴールを受け取る
- nodeから外部に出ていく情報
  - 能動的
    - topicへROSメッセージをpublishする
    - 他nodeのnodeパラメータを書き換える
    - 他nodeのserviceをコールするときに引数を渡す
    - 他nodeのactionをコールするときゴールを渡す
  - 受動的
    - 他nodeや外部プロセスからnodeパラメータをリードされ値を返す
    - 他nodeや外部プロセスからnodeが持つserviceをコールされ結果を返す
    - 他nodeや外部プロセスからnodeが持つactionをコールされフィードバックと結果を返す

## executorとは

- nodeを実行する為のスレッドプールを表すオブジェクト
- 1executor = 1 thread or 複数 thread
- 3種類あり
  - SingleThreadedExecutor
  - MultiThreadedExecutor
  - StaticSingleThreadedExecutor
- 基本的には、1 executable=1 executorだが、 executable内で複数のexecutorを作成することも可能。

## executable/libraryとは
- executable/libraryという概念はROS2特有のものでなく、c/c++の世界（CMake）の用語。ROS2で初めてc++を学び始めた人は、先にCMakeについて学んでおく必要がある。
  - executableもlibraryも、CMakeにおけるbuild targetの単位であり、CMakeist.txt中でビルドする為の情報（使用するソースファイル等）を指定しておき、colcon buildを行うことで生成される。
- executableとは
  - ユーザーもしくは他のexecutableから実行開始可能なプログラム（一群のソースコードから得られる）の単位。
  - エントリーポイントなる関数（通常はmain）を持つ。
  - プロセスとの関係
    - 1 executable = 1 process
- libraryとは
  - executableや他のlibraryからロード可能なプログラム（一群のソースコードから得られる）の単位。
  - エントリーポイントなる関数を持たない。
    - libraryにはクラスを何でも定義できるが、外部から使用して欲しいクラスは所定の方式（=plugin形式）で公開する

### ROS2におけるexecutable
- nodeとの関係
  - 必ずしもnodeを動かす必要はないが、基本的にはnodeを動かすexecutableを作成し、executableを起動して所定の目的を実現するのがROS2のスタイル。
  - 1 executable = N nodes
    - シンプルなケースでは、1executable = 1nodeで作成することが多いが、複数のnodeを実行することもできる。
    - 複数のnodeを1 executable(=1 process)上で実行することで、node間の通信をプロセス内通信とすることができる。
- 使用方法
  - launchの対象となれる。
  - executableは通常の実行可能プログラムであり、直接コマンドラインから実行・プログラム中から実行も可能であるが、ROS2ではlaunchファイルから実行するのが通常のスタイル

### ROS2におけるlibrary
- nodeとの関係
  - カスタムnodeをplugin形式で公開する為に使用する
  - plugin形式の特別な形態としてcomponent形式がある。plugin形式では任意のクラスを公開できるが、component形式で公開できるのはnodeだけ。
  - 基本的にROS2では、nodeはpluginでなく全てcomponent形式とすることが推奨されている。
    - [Components vs. Plugins in ROS 2](https://ubuntu.com/blog/components-vs-plugins-in-ros-2)
    - ただし、ros2_cotrolにおけるcontrollerやHW componentは、component形式でなく通常のplugin形式
  - component形式のメリット
    - nodeを同じプロセス内に配置（同じexecutable上に配置）しnode間の通信をプロセス内通信にすることを、簡単に実装できる
    - componentは、自分で作成したexecutable上で実行することもできるが、rclcpp_componentsパッケージ内に標準で用意されているexecutableであるcomponent_containerを使用しても実行できる
  - component形式でNodeを公開する方法
    - CMakeLists.txtで下記を呼ぶことだけ（呼ばない場合通常のplugin形式になる）
      - rclcpp_components_register_nodes
- 使用方法
  - libraryがcomponent形式で作成された場合、launchの単位となることが可能
  - libraryは実行可能プログラムではないので、直接コマンドラインから実行・プログラム中から実行はできないが、component形式で作成されていればlaunchファイルから起動できる。
    - その場合、1componentが乗ったexecutableが作成され実行される。

