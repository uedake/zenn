---
title: "ROS2を深く理解する：ノード編４　ノードパラメータ"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-09-13 03:15"
---

# 解説対象

本記事では、ROS2のノードを扱う上で非常に重要なノードパラメータについて解説します。ノードパラメータはノードの振る舞いをコントロールする為に外部から渡される設定値です。主にノード起動時に設定されます。

本記事の目標は、ノード起動時のノードパラメータ初期値決定に関係する下記３つの項目を理解することです。
  1. ノードパラメータglobal初期値
  2. ノードパラメータlocal初期値
  3. ノードパラメータ上書き値

また、ノードパラメータの使用宣言に関係するノード初期化オプション
（`allow_undeclared_parameters`と`automatically_declare_parameters_from_overrides`）についても触れます。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。

# 前提知識

ROS2においてパラメータや引数と呼べるモノは複数あります。

ノードパラメータ／ROS引数／launch引数といった用語の概念は下記記事をみてください。

https://zenn.dev/uedake/articles/ros2_concept

## ノードパラメータとROS引数・launch引数の関係

- ノードパラメータの初期値は、ROS引数の中で与えることができます。
- executableをlaunchファイルから起動する場合、launchファイルの書き方次第で、launch引数によってコマンドラインROS引数の値を指定することが可能です
  - つまり、launchファイル実行時にユーザが与える引数の違いによって、ノードパラメータの初期値を変えるという動作が実現可能です

# 公式ドキュメント

- [About-Parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
  - 最初に読むのはここ
- [Using-Parameters-In-A-Class-CPP](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
  - パラメータの宣言・利用の方法がわかる。が、書いてあることは最小限（automatically_declare_parameters_from_overridesオプションについて触れられていなかったりする）

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

## ノードパラメータの初期値について

実務上よくあるパターンとして「launchファイルの指定に応じてノードパラメータに初期値を与えてノードを起動する」ということをあります。

launchファイルの仕組みは下記別記事で記載しています

https://zenn.dev/uedake/articles/ros2_launch4_node

その為、ここでは詳しく書きませんが下記になっています

- launchファイルの`Node`アクションにおいてparametersを指定することで、ノードパラメータの初期値を指定できます
  - parametersを指定した場合の`Node`アクションは、コマンドラインROS引数として`--params-file <yaml_file_path>`を指定してexecutableを起動するという処理を行います。
  - parametersにはyamlファイルのパスの指定、もしくは辞書（パラメータ名とパラメータ値の辞書）を指定できますが、下記の処理になっています
    - yamlファイルのパスが指定された場合、そのパスがコマンドラインROS引数`--params-file`として渡される
    - 辞書が指定された場合、その内容が記載されたテンポラリのyamlファイルが生成され、そのパスがコマンドラインROS引数`--params-file`として渡される
      - `/tmp/launch_params_xxxxxxxx` (xxxxxxxxはランダムな値)に一時的にyamlファイルが生成されている

以下、コマンドラインROS引数`--params-file`がどのように処理されるか見ていきます。

## ノードパラメータの初期値設定の流れ

わかりやすさの為に、先に結論を述べます。

- ノードパラメータ初期値に影響を与える値は下記３種類あります。値はどれもノード初期化オプション（`NodeOption`クラスのインスタンス。`Node`のconstructorの引数`options`）中に含まれます

| 値 | 存在場所 | 値の指定方法 |
| ---- | ---- | ---- |
| ノードパラメータglobal初期値 | `NodeOption`が参照しているグローバルデフォルトコンテキストのフィールド`global_arguments`＝グローバルROS引数。 | executableを実行する人が、実行時にコマンドラインROS引数を与えて指定する |
| ノードパラメータlocal初期値 | `NodeOption`のフィールド`arguments`＝ローカルROS引数。 | executableを実装する人が、`NodeOption`を明示的に作成して`Node`をconstructする時に指定可能 |
| ノードパラメータ上書き値 | `NodeOption`のフィールド`parameter_overrides` | executableを実装する人が、`NodeOption`を明示的に作成して`Node`をconstructする時に指定可能 |


- 指定値には優先度があり上記の順番で下にあるほうが強いです（上書きします）
  - ノードパラメータを上書きしたいなら、簡便な3番目の方法を通常は使用します。２番目の方法を使うべき場面は基本的にはありません

なお、グローバルデフォルトコンテキストとは何かについては別記事を参照ください。

https://zenn.dev/uedake/articles/ros2_node5_context

このノードパラメータ初期値決定の処理は、`resolve_parameter_overrides.cpp`を見るとわかります。

[resolve_parameter_overrides.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/detail/resolve_parameter_overrides.cpp)


```cpp:resolve_parameter_overrides.cpp
std::map<std::string, rclcpp::ParameterValue>
rclcpp::detail::resolve_parameter_overrides(
  const std::string & node_fqn,
  const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args,
  const rcl_arguments_t * global_args)
{
  std::map<std::string, rclcpp::ParameterValue> result;

  // global before local so that local overwrites global
  std::array<const rcl_arguments_t *, 2> argument_sources = {global_args, local_args};

  // Get fully qualified node name post-remapping to use to find node's params in yaml files

  for (const rcl_arguments_t * source : argument_sources) {
    if (!source) {
      continue;
    }
    rcl_params_t * params = NULL;
    rcl_ret_t ret = rcl_arguments_get_param_overrides(source, &params);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
    if (params) {
      auto cleanup_params = rcpputils::make_scope_exit(
        [params]() {
          rcl_yaml_node_struct_fini(params);
        });
      rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(params, node_fqn.c_str());

      if (initial_map.count(node_fqn) > 0) {
        // Combine parameter yaml files, overwriting values in older ones
        for (const rclcpp::Parameter & param : initial_map.at(node_fqn)) {
          result[param.get_name()] =
            rclcpp::ParameterValue(param.get_value_message());
        }
      }
    }
  }

  // parameter overrides passed to constructor will overwrite overrides from yaml file sources
  for (auto & param : parameter_overrides) {
    result[param.get_name()] =
      rclcpp::ParameterValue(param.get_value_message());
  }
  return result;
}
```

## Nodeの実装を確認する

ここから順を追って処理を追います。まず`Node`の実装を見てみましょう。下記がわかります。

1. `Node`のメンバ変数`node_base_`（`NodeBase`へのポインタ）を初期化する際、`options.get_rcl_node_options()`によってノード初期化オプションからrclノード初期化オプション（ノードパラメータ初期値を含む）が取り出され、`NodeBase`のconstructorに渡されます
2. `Node`のメンバ変数`node_parameters_`（`NodeParameters`へのポインタ）を初期化する際、`NodeParameters`のconstructorに下記が渡されます
    - ノードパラメータglobal初期値(`node_base_`内に含まれる)
    - ノードパラメータ上書き値(`options.parameter_overrides()`で得られる)

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

## NodeBaseの実装を確認する

`NodeBase`の実装は、別記事で解説していますので省略します。

https://zenn.dev/uedake/articles/ros2_node1_basic

## nodeパラメータの初期化処理を確認する

最後に`NodeParameters`の実装を見てみましょう。ノードパラメータの初期化は、`NodeParameters`のconstructorで行われています。

1. ノードパラメータの上書きを行う
  - 前述の`rclcpp::detail::resolve_parameter_overrides()`関数を用いて上書きを行っています
  - この関数の引数として下記が渡されます
    - global arguments(`rcl_arguments_t`構造体)
      - `NodeBase`から、`get_context()->get_rcl_context()->global_arguments`から得ている
      - ノードパラメータglobal初期値を含む（`parameter_overrides`内）
      - コマンドラインROS引数`--params-file <yaml_file_path>`で指定されたyamlファイルをparseした結果（ノードパラメータの初期値）が書き込まれている
    - local arguments(`rcl_arguments_t`構造体)
      - rclノード初期化オプション（`rcl_node_options_t`構造体）の`arguments`から得ている
      - ノードパラメータlocal初期値を含む（`parameter_overrides`内）
    - ノードパラメータの上書き値
      - constructorの引数`parameter_overrides`(`std::vector<rclcpp::Parameter>`型)から得ている
2. 上書きされた結果が、メンバ変数`parameter_overrides_`（`std::map<std::string, rclcpp::ParameterValue>`型）に格納される

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

### ノードパラメータの初期化処理のオプション

- 原則的には、ノードパラメータを使用する為には、ノードパラメータを持つノード側で事前に宣言が必要です。
  - 宣言されていないノードパラメータをgetやsetしようとした時には原則例外になります。
    - ただしこの原則は、ノード初期化オプション（`allow_undeclared_parameters`と`automatically_declare_parameters_from_overrides`）で変更可能です
  - なお、宣言されていないノードパラメータを初期値や上書き値としてノード生成時に渡す（`--params-file <yaml_file_path>`もしくは`parameter_overrides`を用いる）だけではエラーにはなりません。


# まとめ

- ノードパラメータの初期値は下記３種の値から決まる
  1. ノードパラメータglobal初期値
  2. ノードパラメータlocal初期値
  3. ノードパラメータ上書き値
- 上記３種の値は優先度があり、下に行くほど強い（上書きする）
- コマンドラインROS引数として`--params-file <yaml_file_path>`を与えることで、「ノードパラメータglobal初期値」を指定できる。「ノードパラメータglobal初期値」を与えれば、そのexecutable中で起動される全てのノードにノードパラメータの初期値を設定することができる
- ただし、「ノードパラメータlocal初期値」や「ノードパラメータ上書き値」をexecutable実装者が指定することで、ノード毎に上書きもできる。
- 原則的には、ノードパラメータを使用する為には、ノードパラメータを持つノード側で事前に宣言が必要だが、ノード初期化オプションである`allow_undeclared_parameters`と`automatically_declare_parameters_from_overrides`を設定することで動作を変更できる（下記表の通り）

[^1]: allow_undeclared_parameters
[^2]: automatically_declare_parameters_from_overrides
[^3]: ParameterNotDeclaredException

| フラグ[^1] | フラグ[^2] | ノード生成時に渡された未宣言パラメータのget/set | ノード生成時に渡されていない未宣言パラメータのget/set |
|---|---|---|---|
| **false** | **false** | 例外発生[^3] | 例外発生[^3] |
| **false** | **true** | 可能 | 例外発生[^3]|
| **true** | **false** | 可能：ただしgetで得られる値は明示的にsetした値のみ。値をsetするまでgetの結果はNOT_SET状態。※渡したパラメータの値は無視されるので注意 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|
| **true** | **true** | 可能 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|
