---
title: "ROS2を深く理解する：ROSノード編４　ノードパラメータ"
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

本記事では、ROSノードを扱う上で非常に重要なノードパラメータについて解説します。ノードパラメータはROSノードの振る舞いをコントロールする為に外部から渡される設定値です。主にROSノード起動時に設定されます。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

## 目標

本記事の目標は、ROSノード起動時のノードパラメータ初期値決定に関係する下記３つの項目を理解することです。
  1. ノードパラメータglobal初期値
  2. ノードパラメータlocal初期値
  3. ノードパラメータ上書き値

また、ノードパラメータの使用宣言に関係するノードオプション
（`allow_undeclared_parameters`と`automatically_declare_parameters_from_overrides`）についても触れます。

 独自ROSノードを設計する時に、ノードパラメータの正しい理解は非常に重要です。特にlaunchファイルとROSノードの間の処理のつながりが理解できると「ROSわかってきた感」が得られます

 本記事では自ROSノードのノードパラメータの読み書きのみを扱います。他ROSノードのノードパラメータの読み書きは別記事を参照ください。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ROSノードには、ライフサイクルを持たないROSノード（`rclcpp::Node`）とライフサイクルを持つROSノード（`rclcpp_lifecycle::LifecycleNode`）の２種類がありますが、ノードパラメータの扱いに関しては完全に同じ実装であり違いはありません。


# 公式ドキュメント

- [About-Parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
  - 最初に読むのはここ
- [Using-Parameters-In-A-Class-CPP](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
  - パラメータの宣言・利用の方法がわかる。が、書いてあることは最小限（automatically_declare_parameters_from_overridesオプションについて触れられていなかったりする）

# 解説

## ノードパラメータとは何か？

ノードパラメータとはROSノードの動作を変更する為の設定値です。
ノードパラメータを理解するには

- 宣言の方法
- 初期値の与え方
- 自ROSノードのノードパラメータの読み書きの方法
- 他ROSノードのノードパラメータの読み書きの方法

を理解する必要があります。

なお、ROS2においてパラメータや引数と呼べるモノはノードパラメータ／コマンドラインROS引数（＝グローバルROS引数）／ローカルROS引数／launch引数など複数ありますが、下記のような関係にあります。

- ノードパラメータとコマンドラインROS引数（＝グローバルROS引数）の関係
  - executable実行時のコマンドラインROS引数の１項目（`--params-file <yaml_file_path>`等）としてノードパラメータのglobal初期値を設定できる、という関係性がある
  - launchファイルの`Node`アクションの引数で`parameters`を設定した場合、コマンドラインROS引数（`--params-file <yaml_file_path>`）が指定されてexecutableが実行される
    - `parameters`にはyamlファイルのパスの指定、もしくは辞書（パラメータ名とパラメータ値の辞書）を指定できますが、下記の処理になっています
      - yamlファイルのパスが指定された場合、そのパスがコマンドラインROS引数`--params-file`として渡される
      - 辞書が指定された場合、その内容が記載されたテンポラリのyamlファイルが生成され、そのパスがコマンドラインROS引数`--params-file`として渡される
        - `/tmp/launch_params_xxxxxxxx` (xxxxxxxxはランダムな値)に一時的にyamlファイルが生成されている
- ノードパラメータとローカルROS引数の関係
  - executable実装におけるNode生成のオプションの中のローカルROS引数の１項目としてノードパラメータのlocal初期値を設定できる、という関係性がある
- ノードパラメータとlaunch引数の関係
  - 直接の関係ではないが、launchファイルの書き方次第ではlaunch引数によってコマンドラインROS引数の値を指定することが可能。 つまり、launchファイル実行時にユーザが与える引数の違いによって、ノードパラメータの初期値を変えるという動作も実現可能

ROS引数とは何かは下記記事をみてください。

https://zenn.dev/uedake/articles/ros2_concept

実務上よくあるパターンは「launchファイルにノードパラメータの初期値を記載してROSノードを起動する」というケースです。launchファイルの仕組みは下記記事も参考ください。

https://zenn.dev/uedake/articles/ros2_launch4_node


## ノードパラメータの宣言

原則ノードパラメータを使用する為には、ノードパラメータを持つROSノード側で事前に宣言が必要です。宣言を行う場所は下記のいずれかです。
- カスタムROSノードクラスのコンストラクタの中
- ROSノードを起動するexecutable中

ただし、ノードオプションである`allow_undeclared_parameters`と`automatically_declare_parameters_from_overrides`を設定することで未宣言パラメータの使用も可能になります（下記表の通り）

[^1]: allow_undeclared_parameters
[^2]: automatically_declare_parameters_from_overrides
[^3]: ParameterNotDeclaredException

| フラグ[^1] | フラグ[^2] | ROSノード生成時に渡された未宣言パラメータのget/set | ROSノード生成時に渡されていない未宣言パラメータのget/set |
|---|---|---|---|
| **false** | **false** | 例外発生[^3] | 例外発生[^3] |
| **false** | **true** | 可能 | 例外発生[^3]|
| **true** | **false** | 可能：ただしgetで得られる値は明示的にsetした値のみ。値をsetするまでgetの結果はNOT_SET状態。※渡したパラメータの値は無視されるので注意 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|
| **true** | **true** | 可能 | 可能：ただし値をsetするまでgetの結果はNOT_SET状態|

なお、未宣言パラメータについて、初期値や上書き値としてROSノード生成時に渡す（`--params-file <yaml_file_path>`もしくは`parameter_overrides`を用いる）だけではエラーにはなりません。


## ノードパラメータの初期値

- ノードパラメータの初期値は下記３種の値から決まります
- ３種の値は優先度があり、下に行くほど強くなります（上書きする）
  - ノードパラメータを固定したいなら、簡便なノードパラメータ上書き値の方法を通常は使用するのがよいです。ノードパラメータlocal初期値を使うべき場面は思いつきません

| 値 | 存在場所 | 値の指定方法 | 有効範囲 |
| ---- | ---- | ---- | ---- |
| ノードパラメータglobal初期値 | `NodeOption`が参照しているグローバルデフォルトコンテキストのフィールド`global_arguments`＝グローバルROS引数。 | executableを実行する人が、実行時にコマンドラインROS引数として`--params-file <yaml_file_path>`等で与えて指定する | executable中で起動される全ROSノード |
| ノードパラメータlocal初期値 | `NodeOption`のフィールド`arguments`＝ローカルROS引数。 | executableを実装する人が、`NodeOption`を明示的に作成して`Node`をconstructする時に指定可能 | 1つのROSノード |
| ノードパラメータ上書き値 | `NodeOption`のフィールド`parameter_overrides` | executableを実装する人が、`NodeOption`を明示的に作成して`Node`をconstructする時に指定可能 | 1つのROSノード |

なお、グローバルデフォルトコンテキストとは何かについては別記事を参照ください。

https://zenn.dev/uedake/articles/ros2_node5_context


## 自ノードのノードパラメータの読み書き
TBD

## 他ノードのノードパラメータの読み書き
TBD


# （参考）ソースの確認

## ノードパラメータの初期値設定の流れ

ノードパラメータの初期値は、「ノードパラメータglobal初期値」「ノードパラメータlocal初期値」「ノードパラメータ上書き値」の３種がありますが、どれもノードオプション（`NodeOption`クラスのインスタンス。`Node`のconstructorの引数`options`）から参照されます

ノードパラメータ初期値決定の処理は、`resolve_parameter_overrides.cpp`を見るとわかります。

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

1. `Node`のメンバ変数`node_base_`（`NodeBase`へのポインタ）を初期化する際、`options.get_rcl_node_options()`によってノードオプションからrclノードオプション（ノードパラメータ初期値を含む）が取り出され、`NodeBase`のconstructorに渡されます
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
      - rclノードオプション（`rcl_node_options_t`構造体）の`arguments`から得ている
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

