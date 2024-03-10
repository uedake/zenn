---
title: "ROS2を深く理解する：ノード編２　ノード名とノード名前空間"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-09-02 22:53"
---

# 解説対象

本記事では、ROS2のノードを扱う上で非常に重要なノード名とノード名前空間について解説します。ノード名とノード名前空間は、システム上に存在するノードを一意に識別する為に使用される文字列です。公式チュートリアルでもノード名について若干説明はありますがノード名前空間についてはほとんど説明がありません。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

## 目標

本記事の目標は、ノード名・ノード名前空間・完全修飾ノード名のフォーマット及び相互関係を理解することです。これらは、ノードが多数起動するシステムを構築するためには理解が必須です。ROSでは完全修飾ノード名が衝突しないようにする必要があり、どのようにノード名やノード名前空間を割り振るかを正しく設計できることが大切です。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（`rclcpp::Node`）とライフサイクルを持つノード（`rclcpp_lifecycle::LifecycleNode`）の２種類がありますが、ノード名とノード名前空間の扱いに関しては完全に同じ実装であり違いはありません。

# 公式ドキュメント

`Node`のconstructorのAPI-referenceを見ても説明は、下記だけ。
```
Create a new node with the specified name.

Parameters:
node_name – [in] Name of the node.
namespace_ – [in] Namespace of the node.
options – [in] Additional options to control creation of the node.

Throws:
InvalidNamespaceError – if the namespace is invalid
```

`namespace_`にどんな指定ができるのかよくわかりません。（ネストした名前空間指定できるの？）

- [launchファイルの解説](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
  - 基本的なことしか書かれていない
- [大きなプロジェクトでのlaunchファイル](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
  - 基本的なことしか書かれていない


# 解説

## ノード名・ノード名前空間とは何か？

ノード名とノード名前空間は、システム上に存在するノードを一意に識別する為に使用される文字列です。ノード実装の深いところ（rmwノード部分）において実装されています。

ノード名とは
- `Node`のconstructor引数`node_name`の値をremapした値がノード名になる
- 相対指定（`/`以外で始まる）のみが可能
  - トピック名等は絶対指定（`/`で始まる）も可能だがノード名では絶対指定はできない
- 255文字以内でかつ`^[A-z_][A-z0-9_]*$`である必要がある

ノード名前空間とは
- `Node`のconstructor引数`namespace_`の値をremapした値がノード名前空間になる
- 必ず絶対指定（`/`で始まる）。`/`で始まっていないときは自動的に先頭に`/`が挿入される。
- `/`を間に複数回含むことができる(例：`/a/b/c`)
- 245文字以内でかつ`^/([A-z_][A-z0-9_]*(/[A-z_][A-z0-9_]*)*)?$`である必要がある

## 完全修飾ノード名とは何か？

完全修飾ノード名（fully qualified node name）は、ノード名前空間とノード名を結合した名前です。システム全体でユニークである必要があります。

- ノード名前空間が`/`の場合
  - 「`/`+ノード名」が完全修飾ノード名になる
- それ以外の場合
  - 「ノード名前空間+`/`+ノード名」が完全修飾ノード名になる

※完全修飾ノード名は、必ず`/`で始まることになる

## ノードサブ名前空間とは何か？

通常のノード（オリジナルノードと呼ばれる）では、上記のノード名・ノード名前空間のみを意識しておけばOKです。しかし、サブノードと呼ばれるノードではさらに「ノードサブ名前空間」という概念も存在します。

サブノードとは
- オリジナルノードの`create_sub_node()`を呼ぶことで生成できる
- オリジナルノードと同一の構造を持つ（同じノードクラスから生成される）
- オリジナルノードとrclノード・rmwノードを共有する
  - すなわち、実体としてはオリジナルノードとサブノードは全部合わせて１つのノードでしかない
  - オリジナルノードとサブノードは同じ完全修飾ノード名をもつ
- 概念的には、オリジナルノードの別名（エイリアス）だと考えて良い。
  - ノードのインタフェースの生成（パブリッシャー・サブスクライバー・サービス・クライアント等）を相対指定（`/`で始まらない）のインタフェース名（トピック名・サービス名・アクション名）で行うときに、下記の違いが生じる
    - オリジナルノードを通して生成すると、引数で指定した名前がremap前のインタフェース名となる
    - サブノードを通して生成すると、引数で指定した名前の前に自動的にノードサブ名前空間が入った名前がremap前のインタフェース名となる
- つまりノードに対して生成するインタフェース名に共通のprefix（ノードサブ名前空間）をつけたい場合に使用するのがサブノード
- 別にサブノードを使用しなくてもインタフェース名に共通のprefixをつけることはできるが、サブノードを使用した方が意図が明確になるというメリットがある。
- ただしサブノードという概念を知らない人には理解しにくいコードになる可能性もある

ノードサブ名前空間は
- 通常ノード（＝オリジナルノード）では空文字
- サブノードでは空文字以外
  - `create_sub_node`の引数`sub_namespace`の値を親ノードのノードサブ名前空間の後に`/`で連結した値がノードサブ名前空間になる
  - 必ず相対指定（`/`で始らない）
  - `/`を間に複数回含むことができる(例：`a/b/c`)

## ノード名・ノード名前空間の決め方

ノード名・ノード名前空間をどのように使って完全修飾ノード名が一意になるようにするかは任意性があります。実装者がルールを自分で決めて運用するとよいです。

特に同じexecutableを複数起動することが想定される場合にはどのようにremapされるのかを意識してノード名・ノード名前空間を決めなければなりません。

なお、remapについては下記記事で解説しています

https://zenn.dev/uedake/articles/ros2_node3_remap

同じexecutableを複数起動する場合、下記のどちらかが必要です
- ノード名前空間をremapすることで完全修飾ノード名の衝突をさける
- ノード名をremapすることで完全修飾ノード名の衝突をさける

通常は前者の方法を用いることが良いです。

下記にそれぞれ例示します。
例では、下記２つのノードを起動するexecutableを想定してみます
- ノードクラスXから生成するノード１つ起動する（＝ノードX）
- ノードクラスYから生成するノードを１つ起動する（＝ノードY）

### 同じexecutableを複数起動する際にノード名前空間をremapする想定

- executableの作り方
  - ノードクラスXを「ノード名=`"nodeX"`、ノード名前空間=`"/"`」で起動する（＝ノードX）
  - ノードクラスYを「ノード名=`"nodeY"`、ノード名前空間=`"/"`」で起動する（＝ノードY）

executabelの起動として、
- １セット目（１回目のexecutableの実行）
  - 名前空間を/my_namesapace1にremapして起動
- ２セット目（２回目のexecutableの実行）
  - 名前空間を/my_namesapace2にremapして起動
とすれば、起動するノードの完全修飾ノード名は下記になります

```yaml
/my_namesapace1/nodeX
/my_namesapace1/nodeY
/my_namesapace2/nodeX
/my_namesapace2/nodeY
```

名前空間をremapするだけで完全修飾ノード名の衝突が避けられています。
通常はこの方法で問題ないです。

### 同じexecutableを複数起動する際にノード名をremapする想定

同じexecutableを複数起動したいが、全部同じノード名前空間を使いたいという場合にはノード名のremapを使用することになります。下記２つのexecutableを検討してみます

1. executableの作り方1
    - ノードクラスXを「ノード名=`"nodeX"`、ノード名前空間=`"/"`」で起動する（＝ノードX）
    - ノードクラスYを「ノード名=`"nodeY"`、ノード名前空間=`"/"`」で起動する（＝ノードY）
2. executableの作り方1
    - ノードクラスXを「ノード名=`"nodeX"`、ノード名前空間=`"/"`」で起動する（＝ノードX）
    - ノードクラスYを「ノード名=`"nodeY"`、ノード名前空間=ノードXのremap後の完全修飾ノード名」で、`NodeOption`として`use_global_arguments(false)`として起動する（remapの影響を受けなくするため）（＝ノードY）

executableの作り方1の場合、
- １セット目（１回目のexecutableの実行）
  - それぞれノード名をnodeX1とnodeY1にremap、名前空間を/my_namesapaceにremapして起動
- ２セット目（２回目のexecutableの実行）
  - それぞれノード名をnodeX2とnodeY2にremap、名前空間を/my_namesapaceにremapして起動
とすれば、起動するノードの完全修飾ノード名は下記になります

```yaml
/my_namesapace/nodeX1
/my_namesapace/nodeY1
/my_namesapace/nodeX2
/my_namesapace/nodeY2
```

完全修飾ノード名の衝突は避ける為に、ノード名を２つremapしなければけません。

一方で、executableの作り方2の場合、
- １セット目（１回目のexecutableの実行）
  - ノード名nodeXをnodeX1にremap、名前空間を/my_namesapaceにremapして起動
- ２セット目（２回目のexecutableの実行）
  - ノード名nodeXをnodeX2にremap、名前空間を/my_namesapaceにremapして起動
とすれば、起動するノードの完全修飾ノード名は下記になります

```yaml
/my_namesapace/nodeX1
/my_namesapace/nodeX1/nodeY
/my_namesapace/nodeX2
/my_namesapace/nodeX2/nodeY
```

ノード名を１つremapすれば済みます。例ではノードが２個ですが、もっとノードの数が増えて複雑になった場合はこのパターンの方が好ましいかもしれません。ノード名前空間をうまく使い見通しをよくすることが重要になりますので、各自工夫をしたいところです。


# （参考）ソースの確認

上記の解説内容について実際にソースコードを追って確認していきます。

## Nodeの実装を確認する

スタート地点として`Node`のconstructorを見てみましょう。
色々やっていますが、引数で渡された`node_name`と`namespace_`に注目すると、`node_base_`の初期化にしか使われていません。

:::message
ここでは`Node`の実装のみ表示していますが`LifecycleNode`の実装もまったく同じです
:::

[node.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node.cpp)
```cpp:node.cpp
Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  const NodeOptions & options)
: node_base_(new rclcpp::node_interfaces::NodeBase(
      node_name,
      namespace_,
      options.context(),
      *(options.get_rcl_node_options()),
      options.use_intra_process_comms(),
      options.enable_topic_statistics())),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  node_logging_(new rclcpp::node_interfaces::NodeLogging(node_base_.get())),
  node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get(), node_timers_.get())),
  node_services_(new rclcpp::node_interfaces::NodeServices(node_base_.get())),
  node_clock_(new rclcpp::node_interfaces::NodeClock(
      node_base_,
      node_topics_,
      node_graph_,
      node_services_,
      node_logging_
    )),
  node_parameters_(new rclcpp::node_interfaces::NodeParameters(
      node_base_,
      node_logging_,
      node_topics_,
      node_services_,
      node_clock_,
      options.parameter_overrides(),
      options.start_parameter_services(),
      options.start_parameter_event_publisher(),
      // This is needed in order to apply parameter overrides to the qos profile provided in
      // options.
      get_parameter_events_qos(*node_base_, options),
      options.parameter_event_publisher_options(),
      options.allow_undeclared_parameters(),
      options.automatically_declare_parameters_from_overrides()
    )),
  node_time_source_(new rclcpp::node_interfaces::NodeTimeSource(
      node_base_,
      node_topics_,
      node_graph_,
      node_services_,
      node_logging_,
      node_clock_,
      node_parameters_,
      options.clock_qos(),
      options.use_clock_thread()
    )),
  node_waitables_(new rclcpp::node_interfaces::NodeWaitables(node_base_.get())),
  node_options_(options),
  sub_namespace_(""),
  effective_namespace_(create_effective_namespace(this->get_namespace(), sub_namespace_))
{
  // we have got what we wanted directly from the overrides,
  // but declare the parameters anyway so they are visible.
  rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions
  {
    QosPolicyKind::Depth,
    QosPolicyKind::Durability,
    QosPolicyKind::History,
    QosPolicyKind::Reliability,
  },
    node_parameters_,
    node_topics_->resolve_topic_name("/parameter_events"),
    options.parameter_event_qos(),
    rclcpp::detail::PublisherQosParametersTraits{});
}
```

## NodeBaseの実装を確認する

次に`NodeBase`のconstructorを見てみます。

引数の`node_name`と`namespace_`は`rcl_node_init()`に渡されてrclノードとしての値設定に使用されます。rclノード、rmwノードの意味がわからない人は下記記事を確認ください。

https://zenn.dev/uedake/articles/ros2_node1_basic

[node_base.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node_interfaces/node_base.cpp)

```cpp:node_base.cpp抜粋
NodeBase::NodeBase(
  const std::string & node_name,
  const std::string & namespace_,
  rclcpp::Context::SharedPtr context,
  const rcl_node_options_t & rcl_node_options,
  bool use_intra_process_default,
  bool enable_topic_statistics_default)
: context_(context),
  use_intra_process_default_(use_intra_process_default),
  enable_topic_statistics_default_(enable_topic_statistics_default),
  node_handle_(nullptr),
  default_callback_group_(nullptr),
  associated_with_executor_(false),
  notify_guard_condition_(context),
  notify_guard_condition_is_valid_(false)
{
  // Create the rcl node and store it in a shared_ptr with a custom destructor.
  std::unique_ptr<rcl_node_t> rcl_node(new rcl_node_t(rcl_get_zero_initialized_node()));

  std::shared_ptr<std::recursive_mutex> logging_mutex = get_global_logging_mutex();

  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
    // TODO(ivanpauno): /rosout Qos should be reconfigurable.
    // TODO(ivanpauno): Instead of mutually excluding rcl_node_init with the global logger mutex,
    // rcl_logging_rosout_init_publisher_for_node could be decoupled from there and be called
    // here directly.
    ret = rcl_node_init(
      rcl_node.get(),
      node_name.c_str(), namespace_.c_str(),
      context_->get_rcl_context().get(), &rcl_node_options);
  }
  if (ret != RCL_RET_OK) {
    //エラー処理省略
  }
// 後略
```

## rcl_node_init()の実装を確認する

`rcl_node_init()`が処理の本丸です。
次に`rcl_node_init()`の実装を見てみましょう。

- ノード名前空間は空文字にはならない。引数`namespace_`が空文字の時は`/`とみなされる。
- ノード名前空間は必ず`/`で始まる。引数`namespace_`が`/`で始まっていないときは先頭に`/`が挿入される。
- ノード名前空間が満たすべき規則は`rmw_validate_namespace()`でチェックされる
- ノード名が満たすべき規則は`rmw_validate_node_name()`でチェックされる
- ノード名とノード名前空間のremapが適用される
- remap後のノード名とノード名前空間は`rmw_create_node()`に渡されrmwノードの生成に使用される。生成されたrmwノードはrclノードのメンバ`impl->rmw_node_handle`に参照が保存される。
- rmwノードの中でノード名とノード名前空間をノードを一意に識別するために用いているが、本記事では解説外

[node.c](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/node.c)

```c:node.c抜粋
rcl_ret_t
rcl_node_init(
  rcl_node_t * node,
  const char * name,
  const char * namespace_,
  rcl_context_t * context,
  const rcl_node_options_t * options)
{

  // 略

  // Make sure the node name is valid before allocating memory.
  int validation_result = 0;
  ret = rmw_validate_node_name(name, &validation_result, NULL);

  //エラー処理省略

  // Process the namespace.
  size_t namespace_length = strlen(namespace_);
  const char * local_namespace_ = namespace_;
  bool should_free_local_namespace_ = false;
  // If the namespace is just an empty string, replace with "/"
  if (namespace_length == 0) {
    // Have this special case to avoid a memory allocation when "" is passed.
    local_namespace_ = "/";
  }

  // If the namespace does not start with a /, add one.
  if (namespace_length > 0 && namespace_[0] != '/') {
    local_namespace_ = rcutils_format_string(*allocator, "/%s", namespace_);
    RCL_CHECK_FOR_NULL_WITH_MSG(
      local_namespace_,
      "failed to format node namespace string",
      ret = RCL_RET_BAD_ALLOC; goto cleanup);
    should_free_local_namespace_ = true;
  }
  // Make sure the node namespace is valid.
  validation_result = 0;
  ret = rmw_validate_namespace(local_namespace_, &validation_result, NULL);

  //エラー処理省略

  // Remap the node name and namespace if remap rules are given
  rcl_arguments_t * global_args = NULL;
  if (node->impl->options.use_global_arguments) {
    global_args = &(node->context->global_arguments);
  }
  ret = rcl_remap_node_name(
    &(node->impl->options.arguments), global_args, name, *allocator,
    &remapped_node_name);
  if (RCL_RET_OK != ret) {
    goto fail;
  } else if (NULL != remapped_node_name) {
    name = remapped_node_name;
  }
  char * remapped_namespace = NULL;
  ret = rcl_remap_node_namespace(
    &(node->impl->options.arguments), global_args, name,
    *allocator, &remapped_namespace);
  if (RCL_RET_OK != ret) {
    goto fail;
  } else if (NULL != remapped_namespace) {
    if (should_free_local_namespace_) {
      allocator->deallocate((char *)local_namespace_, allocator->state);
    }
    should_free_local_namespace_ = true;
    local_namespace_ = remapped_namespace;
  }

  // compute fully qualfied name of the node.
  if ('/' == local_namespace_[strlen(local_namespace_) - 1]) {
    node->impl->fq_name = rcutils_format_string(*allocator, "%s%s", local_namespace_, name);
  } else {
    node->impl->fq_name = rcutils_format_string(*allocator, "%s/%s", local_namespace_, name);
  }

  // node logger name
  node->impl->logger_name = rcl_create_node_logger_name(name, local_namespace_, allocator);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node->impl->logger_name, "creating logger name failed", goto fail);

  RCUTILS_LOG_DEBUG_NAMED(
    ROS_PACKAGE_NAME, "Using domain ID of '%zu'", context->impl->rmw_context.actual_domain_id);

  node->impl->rmw_node_handle = rmw_create_node(
    &(node->context->impl->rmw_context),
    name, local_namespace_);

  // 後略

```

## validation関数の実装を確認する

最後に`rmw_validate_node_name()`と`rmw_validate_namespace()`を見ましょう。

`rmw_validate_node_name()`では、

- ノード名はアルファベットもしくは_で始まること
- ノード名はアルファベット数字もしくは_で構成されること
- ノード名は255文字以内であること

がチェックされます。正規表現っぽく書けば`^[A-z_][A-z0-9_]*$`です。
この規則は[ros1の時のルール](http://wiki.ros.org/ROS/Concepts)と若干違うようです。
ROS2の時のノード名のルールがドキュメント上どこにあるかは見つかりませんでした（[Concepts](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)あたりに書いておいてほ欲しい・・・）

`rmw_validate_namespace()`では、

- ノード名前空間は完全修飾トピック名のルールを満たすこと
- ノード名前空間は245文字以内であること
がチェックされます。

完全修飾トピック名のルールは、`rmw_validate_full_topic_name()`において

- `/`で始まること
- `/`である場合を除き`/`で終わらないこと
- アルファベット数字もしくは`_`もしくは`/`で構成されること
- `/`の直後はアルファベットもしくは`_`であること
- 247文字以内であること

がチェックされます。正規表現っぽく書くと`^/([A-z_][A-z0-9_]*(/[A-z_][A-z0-9_]*)*)?$`

ソースコードは省略（リンクのみ）

https://github.com/ros2/rmw/blob/humble/rmw/src/validate_node_name.c#L23-L91

https://github.com/ros2/rmw/blob/humble/rmw/src/validate_namespace.c#L27-L123

https://github.com/ros2/rmw/blob/humble/rmw/src/validate_full_topic_name.c#L23-L127

## ノードサブ名前空間の実装を理解する

ノードサブ名前空間は、インタフェースを生成する時に`extend_name_with_sub_namespace()`関数を用いて考慮されます。ソースコード中で`extend_name_with_sub_namespace()`が使用されているところを検索すれば、ノードサブ名前空間が影響を与える藩にがわかります。

ノードサブ名前空間が影響を与える代表的なところをピックアップすると下記です

- パブリッシャーやサブスクリプションの作成（Node::create_publsher, Node::create_subscription）
- サービスサーバーやサービスクライアントの作成（Node::create_service, Node::create_client）
- ノードパラメータの取得（Node::get_parameter）

これらで使用されるインタフェース名（remap前）は`extend_name_with_sub_namespace()`の戻り値が使用されます。

処理は単純であり、インタフェース名が相対指定（`/`でも`~`でも始まらない）である時にインタフェース名の前にノードサブ名前空間+`/`を挿入しています

[node_impl.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node_impl.hpp)

```cpp:node_impl.hpp
RCLCPP_LOCAL
inline
std::string
extend_name_with_sub_namespace(const std::string & name, const std::string & sub_namespace)
{
  std::string name_with_sub_namespace(name);
  if (sub_namespace != "" && name.front() != '/' && name.front() != '~') {
    name_with_sub_namespace = sub_namespace + "/" + name;
  }
  return name_with_sub_namespace;
}
```