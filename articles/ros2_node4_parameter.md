---
title: "ROS2を深く理解する：Node編４　nodeパラメータ"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、ROS2のNodeを扱ううえで非常に重要なnodeパラメータについて解説します。nodeパラメータはnode起動時にnodeの振る舞いをコントロールする為に外部から渡される設定値です。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。

# 前提知識

ROS2においてパラメータや引数と呼べるモノは複数あります。

※パラメータ(parameter)という用語と引数(argument)という用語は明確に区別せず、動作を決定するために外部から与えられる変という意味で用いています

| 概念 | 目的 | 形式 | 宣言要否 | 読み出し | 書き換え | 外部からアクセス |
| ---- | ---- | ---- | ---- | ---- | ---- |---- |
| nodeパラメータ | nodeの動作を変更するための設定値 | 名前がついている値の組（key-value dictionary形式） | 原則、明示的に宣言しておいた値のみ受け取れる | nodeの起動時に初期値が渡され、nodeが生存している間は保持され、いつでも読みだし可能 | 可能 | 他のnodeからの読みだしや書き換えが可能（制限することも可能） |
| executable引数 | executableの動作を変更する為の値 | 値の配列（value list形式） | 不要 | executable起動時に受け取るコマンドライン引数（argv）であり、明示的に値を渡さない限り読みだせるのはmain関数の中だけ| 不可 | 不可 |
| launch引数 | launch fileの動作を変更するための値 | 名前がついている値の組（key-value dictionary形式） | 必要。DeclareLaunchArgumentアクションで宣言する | launchファイル中のどこでも読み出せる | 不可 | 不可 | 
| xacro実行引数 | 指定なし | 名前がついている値の組（key-value dictionary形式） | 必要。xacro:argにおいてname=で宣言する | xacroファイル中のどこでも読み出せる |  |
| xacroマクロ引数 | 指定なし | 名前がついている値の組（key-value dictionary形式） | 必要。xacro:macroにおいて params=で宣言する | 明示的に値を渡さない限り、宣言したマクロ中でのみ読み出せる。 | 不可 |

nodeの設定パラメータは単に「パラメータ」と呼ばれることもありますが
この記事では取り違えないように「nodeパラメータ」と呼んでいます。

## nodeパラメータとexecutable引数・launch引数の関係

- nodeパラメータの初期値は、executable引数の中で与えることができます。
- executableをlaunchファイルから起動する場合、launchファイルの書き方次第で、launch引数によってexecutable引数の値を変えることも可能です
  - つまり、やろうと思えば、launchファイル実行時にユーザが与える引数の違いによって、nodeパラメータの初期値を変えるという動作が実現可能です

# 公式ドキュメント

- TBD

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

## nodeパラメータの初期値について

実務上よくあるパターンとして「launchファイルの指定に応じてnodeパラメータに初期値を与えてnodeを起動する」ということをあります。

lauchファイルの仕組みは別記事で記載するのでここでは詳しく書きませんがlaunchファイルの仕組みは下記になっています

- launchファイルのnodeアクションにおいてparametersを指定することで、nodeパラメータの初期値を指定できます。
- parametersを指定した場合のnodeアクションは、executable引数として「--params-file <yaml_file_path>」を指定してexecutableを起動するという処理を行います。
- parametersにはyamlファイルのパスの指定、もしくは辞書（パラメータ名とパラメータ値の辞書）を指定できますが、下記の処理になっています
  - yamlファイルのパスが指定された場合、そのパスがexecutable引数の--params-fileとして渡される
  - 辞書が指定された場合、その内容が記載されたテンポラリのyamlファイルが生成され、そのパスがexecutable引数の--params-fileとして渡される
    - /tmp/launch_params_xxxxxxxx (xxxxxxxxはランダムな値)に一時的にyamlファイルが生成されている

以下、executable引数の--params-fileがどのように処理されるか見ていきます。

## Nodeのconstructorでnodeパラメータ初期値の指定を受け取る

大まかな流れは下記の通り。

1. nodeパラメータの初期値の指定が、Nodeのconstructorの引数optionsで渡されてきます。
2. 渡されたoptionsからoptions.get_rcl_node_options()で設定が取り出されてNodeBaseのconstructorへ渡され、メンバ変数node_base_（NodeBaseへのポインタ）が初期化されます
3. Nodeのメンバ変数node_parameters_（NodeParametersへのポインタ）がnode_base_及びパラメータの上書き設定（options.parameter_overrides()で得られる）を用いて初期化されます
  - パラメータの上書き設定とは、Nodeのconstructor引数のNodeOptionsで指定できます（parameter_overridesで指定）


[node.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node.cpp)
```cpp:node.cpp抜粋
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
// 中略
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
```

## nodeパラメータ初期値の指定がrcl nodeに保存されるまで

`NodeBase`のconstructorでrcl nodeが作成され`NodeBase`のメンバ変数`node_handle_`に参照が設定されます。このrcl nodeがnodeパラメータ初期値の指定情報を含んでいます。

`NodeBase`のconstructor処理は、別記事で解説していますので省略します。

https://zenn.dev/uedake/articles/ros2_node1_basic

- nodeパラメータ初期値の指定情報は、下記でアクセス可能となっています
  - rcl nodeオプションを取り出す
    - `NodeBase`のメソッド`get_rcl_node_handle()`より`get_rcl_node_handle()->impl->options`でrcl nodeオプション（`rcl_node_options_t`構造体）が得られる。
  - rcl nodeオプションからnodeパラメータ初期値の指定情報を取り出す
    - `arguments.impl->parameter_overrides`でnodeパラメータ初期値の指定情報が得られる

## nodeパラメータの初期化処理

nodeパラメータの初期化は、`NodeParameters`のconstructorで行われています。

1. nodeパラメータの上書きを行う
  - rclcpp::detail::resolve_parameter_overrides()関数を用いて上書きを行います
  - この関数の引数として下記が渡されます
    - constructorの引数parameter_overrides(std::vector<rclcpp::Parameter>型)
    - rcl nodeオプション（`rcl_node_options_t`構造体）の`arguments`から得られる`rcl_arguments_t`構造体
      - `rcl_arguments_t`構造体の`parameter_overrides`にはexecutable引数「--params-file <yaml_file_path>」で指定されたyamlファイルをparseした結果（nodeパラメータの初期値）が書き込まれている
2. 上書きされた結果が、メンバ変数parameter_overrides_（std::map<std::string, rclcpp::ParameterValue>型）に格納される
  - このパラメータの集合は、下記を含みます
    - executable起動時の「--params-file <yaml_file_path>」のyamlファイル中で定義される全てのパラメータ
    - Nodeを作る際のNodeOptionsにおけるparameter_overridesで指定した全てのパラメータ

[node_parameters.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp)
```cpp:node_parameters.cpp
NodeParameters::NodeParameters(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::vector<rclcpp::Parameter> & parameter_overrides,
  bool start_parameter_services,
  bool start_parameter_event_publisher,
  const rclcpp::QoS & parameter_event_qos,
  const rclcpp::PublisherOptionsBase & parameter_event_publisher_options,
  bool allow_undeclared_parameters,
  bool automatically_declare_parameters_from_overrides)
: allow_undeclared_(allow_undeclared_parameters),
  events_publisher_(nullptr),
  node_logging_(node_logging),
  node_clock_(node_clock)
{
  using MessageT = rcl_interfaces::msg::ParameterEvent;
  using PublisherT = rclcpp::Publisher<MessageT>;
  using AllocatorT = std::allocator<void>;
  // TODO(wjwwood): expose this allocator through the Parameter interface.
  rclcpp::PublisherOptionsWithAllocator<AllocatorT> publisher_options(
    parameter_event_publisher_options);
  publisher_options.allocator = std::make_shared<AllocatorT>();

  if (start_parameter_services) {
    parameter_service_ = std::make_shared<ParameterService>(node_base, node_services, this);
  }

  if (start_parameter_event_publisher) {
    // TODO(ivanpauno): Qos of the `/parameters_event` topic should be somehow overridable.
    events_publisher_ = rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
      node_topics,
      "/parameter_events",
      parameter_event_qos,
      publisher_options);
  }

  // Get the node options
  const rcl_node_t * node = node_base->get_rcl_node_handle();
  if (nullptr == node) {
    throw std::runtime_error("Need valid node handle in NodeParameters");
  }
  const rcl_node_options_t * options = rcl_node_get_options(node);
  if (nullptr == options) {
    throw std::runtime_error("Need valid node options in NodeParameters");
  }

  const rcl_arguments_t * global_args = nullptr;
  if (options->use_global_arguments) {
    auto context_ptr = node_base->get_context()->get_rcl_context();
    global_args = &(context_ptr->global_arguments);
  }
  combined_name_ = node_base->get_fully_qualified_name();

  parameter_overrides_ = rclcpp::detail::resolve_parameter_overrides(
    combined_name_, parameter_overrides, &options->arguments, global_args);

  // If asked, initialize any parameters that ended up in the initial parameter values,
  // but did not get declared explcitily by this point.
  if (automatically_declare_parameters_from_overrides) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    for (const auto & pair : this->get_parameter_overrides()) {
      if (!this->has_parameter(pair.first)) {
        this->declare_parameter(
          pair.first,
          pair.second,
          descriptor,
          true);
      }
    }
  }
}
```

### nodeパラメータの初期化処理のオプション

- 原則的には、nodeパラメータを使用する為には、nodeパラメータを持つnode側で事前に宣言が必要です。
  - 宣言されていないnodeパラメータをgetやsetしようとした時には原則例外になります。
    - ただしこの原則は、nodeオプション（allow_undeclared_parametersとautomatically_declare_parameters_from_overrides）で変更可能です
  - なお、宣言されていないnodeパラメータをnode生成時に渡す（--params-file <yaml_file_path>もしくはparameter_overridesを用いる）だけではエラーにはなりません。




# まとめ

- executable引数として「--params-file <yaml_file_path>」を与えることで、そのexecutable中で起動される全てのnodeにnodeパラメータの初期値を設定することができる
- 原則的には、nodeパラメータを使用する為には、nodeパラメータを持つnode側で事前に宣言が必要だが、nodeオプションであるallow_undeclared_parametersとautomatically_declare_parameters_from_overridesを設定することで動作を変更できる（下記表の通り）

[^1]: allow_undeclared_parameters
[^2]: automatically_declare_parameters_from_overrides
[^3]: ParameterNotDeclaredException

| フラグ[^1] | フラグ[^2] | Node生成時に渡された未宣言パラメータのget/set | Node生成時に渡されていない未宣言パラメータのget/set |
|---|---|---|---|
| **false** | **false** | 例外発生[^3] | 例外発生[^3] |
| **false** | **true** | 可能 | 例外発生[^3]|
| **true** | **false** | 可能：ただしgetで得られる値は明示的にsetした値のみ。値をsetするまでgetの結果はNOT_SET状態。※渡したパラメータの値は無視されるので注意 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|
| **true** | **true** | 可能 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|
