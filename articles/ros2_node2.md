---
title: "ROS2を深く理解する：Node編②node名とnode名前空間"
emoji: "📘"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-09-02 22:53"
---

# 解説対象

本記事では、ROS2のNodeを扱ううえで非常に重要なnode名とnode名前空間について解説します。公式チュートリアルでもnode名について若干説明はありますがnode名前空間についてはほとんど説明がありません（[launchファイルの解説](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)・[大きなプロジェクトでのlaunchファイル](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)のところにちょっとだけでてくるだけ）。API-reference見ても説明が不十分で正直よくわかりません。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（rclcpp::Node）とライフサイクルを持つノード（rclcpp_lifecycle::LifecycleNode）の２種類がありますが、node名とnode名前空間の扱いに関しては完全に同じ実装であり違いはありません。

# 公式ドキュメント

Nodeはconstruct時にnode_nameとnamespace_を指定できます

```cpp
explicit Node(const std::string &node_name, const std::string &namespace_, const NodeOptions &options = NodeOptions())
```

API-referenceを見ても説明は、下記だけ。
```
Create a new node with the specified name.

Parameters:
node_name – [in] Name of the node.
namespace_ – [in] Namespace of the node.
options – [in] Additional options to control creation of the node.

Throws:
InvalidNamespaceError – if the namespace is invalid
```

namespace_にどんな指定ができるのかよくわかりません。（ネストした名前空間指定できるの？）
ドキュメント見て良くわからないときはソースを見ましょう。

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

自分でソース見て確認したい人は、下記も参考にしてください。
https://zenn.dev/uedake/articles/9332ec4ff8e304

以下確認していきます。

スタート地点はNodeのconstructorです。
色々やっていますが、引数で渡されたnode_nameとnamespace_に注目すると、node_base_の初期化にしか使われていません。

:::message
ここではNodeの実装のみ表示していますがLifecycleNodeの実装もまったく同じです
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

次にNodeBaseを見てみます。
[node_base.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node_interfaces/node_base.cpp)

引数のnode_nameとnamespace_はrcl_node_init()に渡されてrcl nodeとしての値設定に使用されるのと、あとはrmw nodeとして値が正しいかのvalidation（rmw_validate_node_name()及びrmw_validate_namespace()）されるだけですね。とても読みやすいコードです。rcl node、rmw nodeの意味がわからない人は用語集を見てください。

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
    if (ret == RCL_RET_NODE_INVALID_NAME) {
      rcl_reset_error();  // discard rcl_node_init error
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_node_name(node_name.c_str(), &validation_result, &invalid_index);
      //エラー処理省略
    }

    if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      rcl_reset_error();  // discard rcl_node_init error
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_namespace(namespace_.c_str(), &validation_result, &invalid_index);
      // エラー処理省略
      }
    }
    throw_from_rcl_error(ret, "failed to initialize rcl node");
  }
// 後略
```

rcl_node_initここが処理の本丸です。下記のソースを読むとわかることは・・・

- node名前空間は空文字にはならない。引数namespace_が空文字の時は"/"とみなされる。
- node名前空間は必ず"/"で始まる。引数namespace_が"/"で始まっていないときは先頭に"/"が挿入される。
- node名が満たすべき規則はrmw_validate_node_name()でチェックされる
- node名前空間が満たすべき規則はrmw_validate_namespace()でチェックされる
- node名とnode名前空間のremapが適用される
- remap後のnode名とnode名前空間はrmw_create_node()に渡されrmw nodeの生成に使用される。生成されたrmw node はrcl nodeのメンバimpl->rmw_node_handle に参照が保存される。
- rmw nodeの中でnode名とnode名前空間をnodeを一意に識別するために用いているが、本記事では解説外

[node.c](https://github.com/ros2/rcl/blob/humble/rcl/src/node.c)

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
  if (ret != RMW_RET_OK) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return ret;
  }
  if (validation_result != RMW_NODE_NAME_VALID) {
    const char * msg = rmw_node_name_validation_result_string(validation_result);
    RCL_SET_ERROR_MSG(msg);
    return RCL_RET_NODE_INVALID_NAME;
  }

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
  if (ret != RMW_RET_OK) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    goto cleanup;
  }
  if (validation_result != RMW_NAMESPACE_VALID) {
    const char * msg = rmw_namespace_validation_result_string(validation_result);
    RCL_SET_ERROR_MSG_WITH_FORMAT_STRING("%s, result: %d", msg, validation_result);

    ret = RCL_RET_NODE_INVALID_NAMESPACE;
    goto cleanup;
  }

  // 略

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

最後にrmw_validate_node_name()とrmw_validate_namespace()を見ましょう。

rmw_validate_node_name()では、

- node名はアルファベットもしくは_で始まりること
- node名はアルファベット数字もしくは_で構成されること
- node名は255文字以内であること

がチェックされます。正規表現っぽく書けば`^[A-z_][A-z0-9_]*$`です。
この規則は[ros1の時のルール](http://wiki.ros.org/ROS/Concepts)と若干違うようです。
ROS2の時のnode名のルールがドキュメント上どこにあるかは見つかりませんでした（[Concepts](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)あたりに書いておいてほ欲しい・・・）

rmw_validate_namespace()では、

- node名前空間はtopic名のルールを満たすこと
- node名前空間は245文字以内であること
がチェックされます。

topic名のルールは、rmw_validate_full_topic_name()において

- /で始まること
- "/"である場合を除き/で終わらないこと
- アルファベット数字もしくは_もしくは/で構成されること
- /の直後はアルファベットもしくは_であること
- 247文字以内であること

がチェックされます。正規表現っぽく書くと`^/([A-z_][A-z0-9_]*(/[A-z_][A-z0-9_]*)*)?$`

ソースコードは省略（リンクのみ）

https://github.com/ros2/rmw/blob/humble/rmw/src/validate_node_name.c#L23-L91


https://github.com/ros2/rmw/blob/humble/rmw/src/validate_namespace.c#L27-L123

https://github.com/ros2/rmw/blob/humble/rmw/src/validate_full_topic_name.c#L23-L127

# まとめ

node名とnode名前空間は、node実装の深いところ（rmw node部分）においてシステム上に存在するnodeを一意に識別する為に使用されます。

node名は
- Nodeのconstructorに渡した引数node_nameがremapされた値がnode名になる。
- 許されるのは、255文字以内でかつ`^[A-z_][A-z0-9_]*$`

node名前空間は
- Nodeのconstructorに渡した引数namespace_（"/"で始まっていないときは先頭に"/"が挿入）がremapされた値がnode名前空間になる。
- 許されるのは、245文字以内でかつ`^/([A-z_][A-z0-9_]*(/[A-z_][A-z0-9_]*)*)?$`
- つまり、nestした名前空間(例：/a/b/c)も指定できる。

そしてfully qualified name（node名前空間とnode名を結合した名前）は、システム全体でユニークである必要があります。

fully qualified nameは下記で作られます。
- node名前空間が"/"の場合
  - "/"+node名
- それ以外の場合
  - node名前空間+"/"+node名