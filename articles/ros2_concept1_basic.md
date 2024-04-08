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

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

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
  - ROS2ではROSノードをcomponent形式で開発すべきことが解説されている
- [ROS 2のAPIの使い方](https://gbiggs.github.io/rosjp_ros2_intro/ros2_basics.html)
  - ROSノード周りの詳細な解説がありますが、特にexecutorの理解の参考になります
- [ROS 2のプロセス内（intra-process）通信を理解する](https://www.youtalk.jp/2017/06/05/ros2-intra-process.html)
  - パフォーマンスを上げるのに理解必須のプロセス内通信についての参考になります
- [ROS2でプロセス内通信によるゼロコピーを試す](https://qiita.com/shigeharu_shibahata/items/0c5fb3963150403af57c)
  - パフォーマンスを上げるにはゼロコピーの理解も必要

# ROS2におけるプログラミングとは

本記事は、ROS2におけるプログラミングスタイルが複数あることを理解するのが目的です。

ROS2におけるプログラミングは、カジュアルな（＝ほとんどプログラミングしない）スタイルからゴリゴリの大規模開発スタイルまで、幅広く使える汎用性と拡張性を備えています。カジュアルな順番に開発スタイルを並べてみると下記のような雰囲気です。

| スタイル | 独自パッケージの開発 | 独自executableの開発 | カスタムノードクラスの開発 | 説明 |
| ---- | ---- | ---- | ---- | ---- | 
| 既存パッケージ利用のみ | × | × | × | 既存パッケージで足りる場合は、パッケージを利用する為の設定である、launchファイル、ノードパラメータ設定ファイル（configファイル）だけをつくれば良いです。作成したlaunchファイル等は必ずしもパッケージ化しなくても構いません |
| launchパッケージ開発のみ | 〇 | × | × | ほとんど上記と変わりませんが、launchファイルを誰でも使えるように配布する場合パッケージ化しましょう|
| 独自処理パッケージ開発（シンプル） | 〇 | 〇 | × | 実際には多くのケースで既存パッケージの利用だけでは望むシステムは作れないので、独自処理を行えるようexecutableを作成します。独自処理を実装するにあたり、ROS2が提供する`Node`クラスや`LifecycleNode`クラスを使うだけでどうにかなる場合はカスタムROSノードクラスを作成する必要はありません。しかし、独自処理が`Node`や`LifecycleNode`の中にカプセル化されずexecutable上に実装されることになるため、複雑なシステムには向きません|
| 独自処理パッケージ開発（カスタムノードクラスあり） | 〇 | 〇 | 〇 | 複雑なシステムをつくる場合、ROS2が提供する`Node`や`LifecycleNode`を直接使うのでなく、それらを継承したカスタムROSノードクラスを実装し使用したくなります。カスタムROSノードクラスを作れば独自処理を中に埋め込め、オブジェクト指向で機能を実装でき、見通しが良くなります。カスタムROSノードクラスは直接executable上に定義せずlibraryに分離して定義するのが推奨です|

以下では上記を理解する為の概念を説明していきます

# ROS2を理解する上でのポイント

概念として、配布単位（package）、プログラム実行単位（executable/library）、論理処理単位（ROSノード）をちゃんと区別して把握することが重要です。

通常ROS2を用いたロボットシステムは、複数のpackageに含まれる複数のexecutableを実行することで複数のROSノードを生成し動作します。

- ソフトウェアは、ファイル群としての存在であるpackageを単位として配布されます
  - packageは、機能を実現する為の一連のプログラム（executable/library）ファイルや各種リソースファイル一式を提供します
- ソフトウェアは、実行可能ファイルとしての存在であるexecutableを単位として実行します
  - executableは、直接実行可能なプログラムです。基本的にはROSノードを生成し動作させる役目を果たします
  - libraryは、他のexecutableから読み込まれて間接的に実行されるプログラムです。libraryは他のexecutableにロードされ実行されます。例えばlibraryで定義しているROSノードクラスは、libraryをロードしたexecutable上で使用できるようになります
- ソフトウェアは、論理的な存在であるROSノードを単位として処理を行います
  - ROSノードは、情報を所定のルールに従って入力・処理・出力します

下記で重要なポイントに絞って説明していきます。

# ROSノードとは

- ROSノードは、ROS1やROS2上で実行する処理を担うオブジェクト（論理処理単位）です
  - ROSノードを単にノードと呼ぶことも多いですが、〇〇ノードという他の種類のノードも文脈によっては存在しうる為、区別するためにはROSノードと呼びます（例えば、移動ロボットのナビゲーションを行う文脈では、BTノードというノードが登場しますが、BTノードはROSノードではありません）
  - ROSノードは、executable上でROSノードクラスをインスタンス化して生成します
- 標準ROSノードクラスとカスタムROSノードクラス
  - ROS2には、標準ROSノードクラス（`Node`クラスもしくは`LifecycleNode`クラス）が定義されています。独自の処理を持ったROSノードを作るためにこれら標準ROSノードクラスをそのまま使用してもよいですが、標準ROSノードクラスを継承したクラス（＝カスタムROSノードノード）を作成して使用してもよいです
  - 使い分け
    - 独自処理のコードをカプセル化したいならカスタムROSノードクラスを作成します
    - ライフサイクルイベントのハンドリングをする場合は、カスタムROSノードクラスを作成します
    - どちらも不要であれば、標準ROSノードクラスを直接用います
      - ただしこの場合独自処理（例えば、ROSノードにserviceサーバーを追加しserviceがコールされた時の処理を実装する）は、ROSノードクラス外（例えばexecutableで実行される関数中）に記述することになり、オブジェクト指向の考え方からすると全容を捉えにくい構成になることに注意が必要です
  - カスタムROSノードクラスを作成する場合のベストプラクティス
    - カスタムROSノードクラスの定義は、executable中で定義することもlibrary中で定義することもどちらも可能であるが、下記が推奨です
      - library中でcomponent形式で定義する
      - executable側からlibrary中のカスタムROSノードクラスをロードして利用する
    - カスタムROSノードクラスを特定のexecutableに密結合させずに分離することで柔軟な運用が可能になります

ノード周りの詳細な解説は下記を参照ください

https://zenn.dev/uedake/articles/ros2_node1_basic

## ROSノードを動かす手段

- ROSノードは必ずexecutor上で動く
  - `rclcpp:spin(node)`で実行した場合でも、内部で`SingleThreadedExecutor`が生成されその中でROSノードが動いています
- ROSノードを動かすexecutorの実行を開始する方法は以下のどちらか
  - ROSノードを動かす処理を記述したexecutableを実行することでROSノードの処理を開始する
  - ROSノードを動かす処理を記述したlibraryをロードするexecutableを実行することでROSノードの処理を開始する

例えば、c++から他のプログラムファイルで定義されたカスタムROSノードクラス（`my_namespace::MyNode`）を動かす場合

```cpp
rclcpp::executors::SingleThreadedExecutor exec;
auto my_node = std::make_shared<my_namespace::MyNode>(options);
exec.add_node(my_node);
```

# executorとは

- ROSノードを実行する為のスレッドプールを表すオブジェクト
- 1executor = 1 thread or 複数 thread
- 3種類あり
  - SingleThreadedExecutor
  - MultiThreadedExecutor
  - StaticSingleThreadedExecutor
- 基本的には、1 executable=1 executorだが、 executable内で複数のexecutorを作成することも可能。

# executable/libraryとは
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

## ROS2におけるexecutable
- ROSノードとの関係
  - 必ずしもROSノードを動かす必要はないが、基本的にはROSノードを動かすexecutableを作成し、executableを起動して所定の目的を実現するのがROS2のスタイル。
  - 1 executable = N ROSノード
    - シンプルなケースでは、1executable = 1ROSノードで作成することが多いが、複数のROSノードを実行することもできる。
    - 複数ROSノードを1 executable(=1 process)上で実行することで、ROSノード間の通信をプロセス内通信とすることができる。
- 使用方法
  - launchの対象となれる。
  - executableは通常の実行可能プログラムであり、直接コマンドラインから実行・プログラム中から実行も可能であるが、ROS2ではlaunchファイルから実行するのが通常のスタイル

## ROS2におけるlibrary
- ROSノードとの関係
  - カスタムROSノードクラスをplugin形式で公開する為に使用する
  - plugin形式の特別な形態としてcomponent形式がある。plugin形式では任意のクラスを公開できるが、component形式で公開できるのはROSノードクラスだけ。
  - 基本的にROS2では、ROSノードクラスはpluginでなく全てcomponent形式とすることが推奨されている。
    - [Components vs. Plugins in ROS 2](https://ubuntu.com/blog/components-vs-plugins-in-ros-2)
    - ただし、ros2_cotrolにおけるcontrollerやHW componentは、component形式でなく通常のplugin形式
  - component形式のメリット
    - ROSノードを同じプロセス内に配置（同じexecutable上に配置）しROSノード間の通信をプロセス内通信にすることを、簡単に実装できる
    - componentは、自分で作成したexecutable上で実行することもできるが、rclcpp_componentsパッケージ内に標準で用意されているexecutableであるcomponent_containerを使用しても実行できる
  - component形式でROSノードクラスを公開する方法
    - CMakeLists.txtで`rclcpp_components_register_nodes`を呼ぶ（呼ばない場合通常のplugin形式になる）
- 使用方法
  - libraryがcomponent形式で作成された場合、launchの単位となることが可能
  - libraryは実行可能プログラムではないので、直接コマンドラインから実行・プログラム中から実行はできないが、component形式で作成されていればlaunchファイルから起動できる。
    - その場合、1componentが乗ったexecutableが作成され実行される。


# ROS2におけるパラメータ

ROS2においてパラメータや引数と呼べるモノは複数あります。

※パラメータ(parameter)という用語と引数(argument)という用語は明確に区別せず、動作を決定するために外部から与えられる変という意味で用いています

**表1:ROS2における各種パラメータの一覧**

| 概念 | 目的 | 形式 | 値の型 | 
| ---- | ---- | ---- | ---- |
| ノードパラメータ | ROSノードの動作を変更する | 名前がついている値の組（key-value dictionary形式） | bool, int64, double, string, byte[], bool[], int64[], double[], string[] |
| コマンドラインROS引数 | グローバルROS引数を生成する | コマンドライン引数(フラグと値の配列) | string |
| グローバルROS引数 | executable中の全ROSノードの初期化処理を制御する | `rcl_arguments_impl_s`構造体 | `rcl_params_t`他多数 |
| ローカルROS引数 | 個別のROSノードの初期化処理を制御する | `rcl_arguments_impl_s`構造体 | `rcl_params_t`他多数 |
| launch引数 | launch fileの動作を変更する | 名前がついている値の組（key-value dictionary形式） | str | 
| xacro実行引数 | URDFの生成時に可変値を与える | 名前がついている値の組（key-value dictionary形式） | str |
| xacroマクロ引数 | マクロの実行時に可変値を与える | 名前がついている値の組（key-value dictionary形式） | str（※１）,xmlブロック |

※１：xacroマクロ引数の値は内部的には単なるstr型として受け渡される（マクロを呼ぶ側が`hoge="1"`を入力した場合、これはstr型の`"1"`として受け取られる）。ただし、`${hoge + 1}`等の形式でxacroマクロ引数を使用する時に`hoge`部分が置換される時に`eval()`によって値が解釈される（`"1"`という文字列はintの`1`と解釈される）。そして最終的に`${hoge + 1}`の結果はint型の`2`となる（これを受け取る側では文字列として解釈し、str型の`"2"`となる）

**表2:ROS2における各種パラメータのアクセス性**

| 概念 | 宣言要否 | 読み出し | 書き換え | 初期値 |
| ---- | ---- | ---- | ---- | ---- |
| ノードパラメータ | 原則、明示的に宣言しておいた値のみ受け取れる | 自ROSノードの値はフィールド参照で読み出し可能。他ROSノードの値はROSサービスを使用して読みだし可能（制限することも可能） | 自ROSノードの値はフィールド参照で書き込み可能。他ROSノードの値はROSサービスを使用して書き込み可能（制限することも可能） | executable実行時にコマンドラインROS引数を用いてグローバル初期値を設定可能な他、executable実装時のコード中でローカル初期値を設定可能 |
| コマンドラインROS引数 | 不要 | 自プロセスの値をexecutableのmain関数の引数argvから読み出せる | 不可 | executable実行時のコマンドライン引数として値を指定する |
| グローバルROS引数 | 不要 | 自プロセスの値をグローバルデフォルトコンテキストから読みだせる | 不可（ローカルROS引数で上書きは可能） | コマンドラインROS引数から生成される |
| ローカルROS引数 | 不要 | 自ROSノードの値をフィールド参照で読みだせる | 不可 | executable実装時のコード中（`Node`のconstructorへの引数）で実装する |
| launch引数 | 必要。`DeclareLaunchArgument`アクションで宣言する | launchファイル中のどこでも読み出せる | 不可 | launchコマンドのコマンドライン引数として値を指定する | 
| xacro実行引数 | 必要。`xacro:arg`において`name=`で宣言する | xacroファイル中（includeしたxacroファイル含む）のどこでも読み出せる | 不可 | xacroコマンドのコマンドライン引数として値を指定する |
| xacroマクロ引数 | 必要。`xacro:macro`において `params=`で宣言する | 宣言したマクロ中でのみ読み出せる。 | 不可 | マクロ呼び出し時に値を指定する |


ノードパラメータは単に「パラメータ」と呼ばれることもありますが
この記事では取り違えないようにノードパラメータと呼んでいます。

# （参考）ソースの確認

## ノードパラメータの値として取れる型

ノードパラメータの値として取れる型は下記の通りrclレポジトリで定義されています。

[rcl_yaml_param_parser/types.h](https://github.com/ros2/rcl/blob/humble/rcl_yaml_param_parser/include/rcl_yaml_param_parser/types.h)

```h:rcl_yaml_param_parser/types.h
/// variant_t stores the value of a parameter
/*
 * Only one pointer in this struct will store the value
 * \typedef rcl_variant_t
 */
typedef struct rcl_variant_s
{
  bool * bool_value;  ///< If bool, gets stored here
  int64_t * integer_value;  ///< If integer, gets stored here
  double * double_value;  ///< If double, gets stored here
  char * string_value;  ///< If string, gets stored here
  rcl_byte_array_t * byte_array_value;  ///< If array of bytes
  rcl_bool_array_t * bool_array_value;  ///< If array of bool's
  rcl_int64_array_t * integer_array_value;  ///< If array of integers
  rcl_double_array_t * double_array_value;  ///< If array of doubles
  rcutils_string_array_t * string_array_value;  ///< If array of strings
} rcl_variant_t;
```

