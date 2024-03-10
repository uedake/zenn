---
title: "ROS2を深く理解する：ノード編５　コンテキスト"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2024-02-04 15:50"
---

# 解説対象

本記事では、ROS2のノードを理解する上で重要なコンテキストの概念を解説します。ROS2では、ノードを生成するexecutableを実装するときに必ず`rclcpp::init()`を呼ぶようにしていると思います（意味がわからなくても決まりとして実行しているはず）。この関数がコンテキストを初期化している関数なのですが、このコンテキストとは何かを本記事では解説します。

## 目標
本記事の目標は、`ros2 node`コマンドでノードを起動するときやlaunchの`Node`アクションでノードを起動するときに指定するコマンドラインROS引数が、どのような仕組みでノードに影響を与えるのかを理解することです。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（`rclcpp::Node`）とライフサイクルを持つノード（`rclcpp_lifecycle::LifecycleNode`）の２種類がありますが、ノード名とノード名前空間の扱いに関しては完全に同じ実装であり違いはありません。

# 公式ドキュメント

TBD

# 解説

## グローバルデフォルトコンテキストとは何か？

グローバルデフォルトコンテキストとは、プロセス内でただ１つ存在するコンテキスト（`DefaultContext`クラスのインスタンス）で、プロセス中で共有される）であり、グローバルROS引数を保持するのが主たる役割です。コンテキストとは、ノードを生成する時に参照される情報の塊であり、ROS引数を内部に保持します。このコンテキストとして通常はexecutableから起動される全ノード共通でグローバルデフォルトコンテキストが使用されるようになっています。

- グローバルデフォルトコンテキストは、`rclcpp::init()`実行時に値が設定されます
- グローバルデフォルトコンテキストが保持するグローバルROS引数は、グローバルデフォルトコンテキストを参照する全ノードで使用されます。通常はexecutableから起動する全ノードがグローバルデフォルトコンテキストを参照する為、全ノードが同じグローバルROS引数を用いることになります。

グローバルROS引数はコマンドラインROS引数（`ros2 node`コマンドでexecutableを起動する時の引数として、`--ros-args`と記載した後に指定する所定のオプション）から生成され、グローバルデフォルトコンテキストのフィールド`global_arguments`に格納されます。ROS2では、このグローバルROS引数が実行時のノードの初期化において重要な働きをします。

## グローバルROS引数が保持する情報

- コマンドラインROS引数の指定は下記が存在し、これらの指定値をグローバルROS引数は保持します
  - `--param`：ノードパラメータのキーバリューを指定する
  - `--params-file`：ノードパラメータを記載したファイルへのパスを指定する
  - `--remap`：remapルールを指定する
  - `--enclave`：セキュリティ保護機能であるenclaveを用いる為のパスを指定する
  - `--log-level`：loggingするレベルを指定する
  - `--log-config-file`：loggingの設定ファイルへのパスを指定する
  - `--enable-xxx` or `--disable-xxx`：ログ等のON/OFFを指定する

`Node`アクションが記述されたlaunchファイルを実行することでもノードを生成するexecutableを実行できます。この時も、`Node`アクションに指定したオプションがグローバルROS引数として指定されてexecutableが実行されます。


# （参考）ソースの確認


グローバルデフォルトコンテキストにかかわる処理は、

1. グローバルデフォルトコンテキストの初期化
2. ノード生成時のグローバルデフォルトコンテキストの参照

の２段階を理解する必要があります。以下で順に解説します。

## グローバルデフォルトコンテキストの初期化を理解する

ノードを生成するexecutableを実装するときに必ず記載する`rclcpp::init()`の実装を見てみます。グローバルデフォルトコンテキストを取得して、そのメソッド`init()`にコマンドライン引数`argc`,`argv`を渡す処理をしています。

[utilities.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/utilities.cpp)

```cpp
void
init(
  int argc,
  char const * const * argv,
  const InitOptions & init_options,
  SignalHandlerOptions signal_handler_options)
{
  using rclcpp::contexts::get_global_default_context;
  get_global_default_context()->init(argc, argv, init_options);
  // Install the signal handlers.
  install_signal_handlers(signal_handler_options);
}
```

`get_global_default_context()`の実装を見てみると、グローバルデフォルトコンテキストがプロセス内でただ１つ存在するコンテキスト（`DefaultContext`クラスのインスタンス）で、プロセス中で共有される）であることがわかります。

[default_context.cpp](https://github.com/ros2/rclcpp/blob/c10764f4329f8c1494b68ff60ce55a3c74c8de9e/rclcpp/src/rclcpp/contexts/default_context.cpp)

```cpp
DefaultContext::SharedPtr
rclcpp::contexts::get_global_default_context()
{
  static DefaultContext::SharedPtr default_context = DefaultContext::make_shared();
  return default_context;
}
```

`DefaultContext`クラスの`init()`メソッドは、継承元クラスの`Context`クラスで定義されています。`Context::init()`メソッド中ではさらにコマンドライン引数`argc`,`argv`を渡し`rcl_init()`関数を呼んで`rcl_context_t`構造体を初期化した上でその結果を`rcl_context_`フィールドに保持しています。

[context.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/context.cpp)

```cpp
void
Context::init(
  int argc,
  char const * const * argv,
  const rclcpp::InitOptions & init_options)
{
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);
  if (this->is_valid()) {
    throw rclcpp::ContextAlreadyInitialized();
  }
  this->clean_up();
  rcl_context_t * context = new rcl_context_t;
  if (!context) {
    throw std::runtime_error("failed to allocate memory for rcl context");
  }
  *context = rcl_get_zero_initialized_context();
  rcl_ret_t ret = rcl_init(argc, argv, init_options.get_rcl_init_options(), context);
  if (RCL_RET_OK != ret) {
    delete context;
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl");
  }
  rcl_context_.reset(context, __delete_context);

  if (init_options.auto_initialize_logging()) {
    logging_mutex_ = get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex_);
    size_t & count = get_logging_reference_count();
    if (0u == count) {
      ret = rcl_logging_configure_with_output_handler(
        &rcl_context_->global_arguments,
        rcl_init_options_get_allocator(init_options.get_rcl_init_options()),
        rclcpp_logging_output_handler);
      if (RCL_RET_OK != ret) {
        rcl_context_.reset();
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to configure logging");
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "logging was initialized more than once");
    }
    ++count;
  }

  try {
    std::vector<std::string> unparsed_ros_arguments = detail::get_unparsed_ros_arguments(
      argc, argv, &(rcl_context_->global_arguments), rcl_get_default_allocator());
    if (!unparsed_ros_arguments.empty()) {
      throw exceptions::UnknownROSArgsError(std::move(unparsed_ros_arguments));
    }

    init_options_ = init_options;

    weak_contexts_ = get_weak_contexts();
    weak_contexts_->add_context(this->shared_from_this());
  } catch (const std::exception & e) {
    ret = rcl_shutdown(rcl_context_.get());
    rcl_context_.reset();
    if (RCL_RET_OK != ret) {
      std::ostringstream oss;
      oss << "While handling: " << e.what() << std::endl <<
        "    another exception was thrown";
      rclcpp::exceptions::throw_from_rcl_error(ret, oss.str());
    }
    throw;
  }
}
```

`rcl_init()`の処理は量がありますが、コマンドライン引数に関した処理は下記の抜粋箇所です。`rcl_parse_arguments()`関数を呼びコマンドライン引数を解析して得たグローバルROS引数を`context->global_arguments`に格納しています。

```cpp
rcl_ret_t
rcl_init(
  int argc,
  char const * const * argv,
  const rcl_init_options_t * options,
  rcl_context_t * context)
{

  // 略

  // Zero initialize global arguments.
  context->global_arguments = rcl_get_zero_initialized_arguments();

  // 略

  // Parse the ROS specific arguments.
  ret = rcl_parse_arguments(argc, argv, allocator, &context->global_arguments);
  if (RCL_RET_OK != ret) {
    fail_ret = ret;
    RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Failed to parse global arguments");
    goto fail;
  }

  // 略

```

`rcl_parse_arguments()`の中身は省略しますが、コマンドライン引数中に`--ros-args`を見つけた場合にそれ以降をグローバルROS引数とみなし取り出す処理をしています。

## ノード生成時のコンテキストの参照を理解する

`Node`のconstructorを呼ぶのは`rclcpp::init()`を呼んだ後とするのがROS2の決まりですが、それは`Node`のconstructorの処理中でグローバルデフォルトコンテキストを参照しているからです。

まず、ROS2の`Node`クラスのconstructor定義をみてみます。
引数`options`（ノードオプションと呼ぶ）は特に指定しない場合に、デフォルト値(`NodeOptions()`)が使用されることがわかります。

[node.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp)

```cpp
 RCLCPP_PUBLIC
  explicit Node(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions());

  RCLCPP_PUBLIC
  explicit Node(
    const std::string & node_name,
    const std::string & namespace_,
    const NodeOptions & options = NodeOptions());
```

`NodeOptions`のデフォルトでは、フィールド`context_`が`rclcpp::contexts::get_global_default_context()`で初期化されており、グローバルデフォルトコンテキストが設定されていることがわかります。

[node_options.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node_options.hpp)

```cpp
class NodeOptions
{

//略

private:
  rclcpp::Context::SharedPtr context_ {
    rclcpp::contexts::get_global_default_context()};
```

以上で各`Node`は、フィールド`context_`を通してグローバルROS引数に`context_->global_arguments`でアクセスできる仕組みになっていることがわかりました。
