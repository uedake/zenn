---
title: "（書きかけ）ROS2を深く理解する：Node編４　nodeパラメータ"
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

# 公式ドキュメント

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

# まとめ

### アーキテクチャ
- executable起動時の引数として、「--params-file <yaml_file_path>」を付与することで、そのexecutable中で起動される全てのnodeにNodeパラメータを渡すことができる
    - launchファイルのNodeアクションの引数でparametersを指定した時、その内容は「--params-file <yaml_file_path>」の形に整えられexecutableの引数として渡される。
    - どうやら/tmp/launch_params_xxxxxxxx (xxxxxxxxはランダムな値)に一時的にyamlファイルが生成されて引き渡される様子
- executable起動時の引数として与えらえた「--params-file <yaml_file_path>」の処理は、下記の通り

1. Nodeはコンストラクトされる時に、メンバ変数node_base_（NodeBaseへのポインタ）の初期化、及びメンバ変数node_parameters_（NodeParametersへのポインタ）を初期化する。コンストラクタの引数であるNodeOptionsを利用している。重要なのは、下記のコール。
    - options.get_rcl_node_options()
    - options.parameter_overrides()

```
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
略
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

2. NodeOptions::get_rcl_node_options()はrcl/src/rcl/arguments.cのrcl_parse_arguments関数を呼ぶ
    - 結果、NodeOptionsのメンバ変数であるnode_options_（rcl_node_options_t型へのポインター）の　node_options_->arguments->impl->parameter_overridesに<yaml_file_path>はparseされた結果が書き込まれる
    - rcl_node_options_t型は下記の通り

```
typedef struct rcl_node_options_s
{
  rcl_allocator_t allocator;
  bool use_global_arguments;
  rcl_arguments_t arguments;
  bool enable_rosout;
  rmw_qos_profile_t rosout_qos;
} rcl_node_options_t

typedef struct rcl_arguments_s
{
  /// Private implementation pointer.
  rcl_arguments_impl_t * impl;
} rcl_arguments_t;

typedef struct rcl_arguments_impl_s
{
  /// Array of indices to unknown ROS specific arguments.
  int * unparsed_ros_args;
  /// Length of unparsed_ros_args.
  int num_unparsed_ros_args;

  /// Array of indices to non-ROS arguments.
  int * unparsed_args;
  /// Length of unparsed_args.
  int num_unparsed_args;

  /// Parameter override rules parsed from arguments.
  rcl_params_t * parameter_overrides;

  /// Array of yaml parameter file paths
  char ** parameter_files;
  /// Length of parameter_files.
  int num_param_files_args;

  /// Array of rules for name remapping.
  rcl_remap_t * remap_rules;
  /// Length of remap_rules.
  int num_remap_rules;

  /// Log levels parsed from arguments.
  rcl_log_levels_t log_levels;
  /// A file used to configure the external logging library
  char * external_log_config_file;
  /// A boolean value indicating if the standard out handler should be used for log output
  bool log_stdout_disabled;
  /// A boolean value indicating if the rosout topic handler should be used for log output
  bool log_rosout_disabled;
  /// A boolean value indicating if the external lib handler should be used for log output
  bool log_ext_lib_disabled;

  /// Enclave to be used.
  char * enclave;

  /// Allocator used to allocate objects in this struct
  rcl_allocator_t allocator;
} rcl_arguments_impl_t;
```

3. NodeBaseはコンストラクト時にメンバ変数node_handle_（rcl_node_tへのポインタ）を初期化する。この際にコンストラクタの引数で渡されるrcl_node_options（rcl_node_options_t型）をnode_handle_->impl->optionsに書き込む
    - 参考）rcl/src/rcl/node.cのrcl_node_init()関数

4. NodeParametersはコンストラクト時には下記が行われる
    - コンストラクタの引数node_base(NodeBaseへのポインタ)のnode_base->get_rcl_node_handle()->impl->optionsからoptions（rcl_node_options_t型）を取り出した上で、コンストラクタの引数parameter_overrides(std::vector<rclcpp::Parameter>型)と合わせてrclcpp::detail::resolve_parameter_overrides()関数に渡すことでパラメータの上書きを行う。
        - 上書きされた結果は、メンバ変数parameter_overrides_（std::map<std::string, rclcpp::ParameterValue>型）に格納される。
        - このパラメータの集合は、下記を含む
            - executable起動時の「--params-file <yaml_file_path>」のyamlファイル中で定義される全てのパラメータ
            - Nodeを作る際のNodeOptionsにおけるparameter_overridesで指定した全てのパラメータ
    - コンストラクタの引数automatically_declare_parameters_from_overridesがtrueである場合、下記を行う
        - メンバ変数parameter_overrides_に存在するパラメータ名の内でまだdeclareされていないパラメータ全てをdeclareする。

### allow_undeclared_parametersとautomatically_declare_parameters_from_overrides
- declareされていないパラメータをNode生成時に渡す（--params-file <yaml_file_path>もしくはparameter_overridesを用いる）のはエラーにならないが、declareされていないパラメータをgetやsetしようとした時には原則例外になる。この原則をallow_undeclared_parametersとautomatically_declare_parameters_from_overridesオプションで変更可能。
- allow_undeclared_parameters=false, automatically_declare_parameters_from_overrides=falseの場合（デフォルト）
    - Node生成時に渡された未declareパラメータ
        - get: 例外（ParameterNotDeclaredException）発生。
        - set: 例外（ParameterNotDeclaredException）発生。
    - Node生成時に渡されていない未declareパラメータ
        - get：　例外（ParameterNotDeclaredException）発生。
        - set：　例外（ParameterNotDeclaredException）発生。
- allow_undeclared_parameters=false, automatically_declare_parameters_from_overrides=trueの場合
    - Node生成時に渡された未declareパラメータ
        - get: 可能。渡された値を得られる。
        - set: 可能。
    - Node生成時に渡されていない未declareパラメータ
        - get：　例外（ParameterNotDeclaredException）発生。
        - set：　例外（ParameterNotDeclaredException）発生。
- allow_undeclared_parameters=true, automatically_declare_parameters_from_overrides=falseの場合
    - Node生成時に渡された未declareパラメータ
        - get: 可能。ただし値は(値をsetするまで)NOT_SET状態。※渡したパラメータの値は無視されるので注意。getで得られる値は明示的にsetした値のみ。
        - set: 可能。
    - Node生成時に渡されていない未declareパラメータ
        - get：　可能。ただし値は(値をsetするまで)NOT_SET状態。getで得られる値は明示的にsetした値のみ。
        - set：　可能。
- allow_undeclared_parameters=true, automatically_declare_parameters_from_overrides=trueの場合
    - Node生成時に渡された未declareパラメータ
        - get: 可能。渡された値を得られる。
        - set: 可能。
    - Node生成時に渡されていない未declareパラメータ
        - get：　可能。ただし値は(値をsetするまで)NOT_SET状態。getで得られる値は明示的にsetした値のみ。
        - set：　可能。