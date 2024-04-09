---
title: "ROS2を深く理解する：インタフェース編１　基本構造"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、ROSノードとの間で情報入出力を行うインタフェースついて解説します。ROS2における基本となるインタフェースはROSトピック・ROSサービス・ROSアクションの３種類です。この他にノードパラメータ・ros2_controlインタフェース（command interface / state interface）等もインタフェースと考えられるため合わせて解説します。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいていますが、rclpy側も結局はrclで規定されるrclノード実装につながりますので、大部分は共通です。

# 公式ドキュメント

- [callback-groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)
  - callback groupについて解説あり。必読。

# 解説

## ROSトピックとは
- ROSトピックを介して情報を入出力するには、まずノードにて事前準備（パブリッシャーとサブスクリプションの生成）が必要です。その後にメッセージの送信（=publish実行）と受信（=サブスクリプションのコールバック実行）を行います。
  - パブリッシャーとは
    - ノードが保有する「特定のトピックへメッセージを送信（publish）する」仕組み。下記の２段階でメッセージを送信する。
      1. 事前準備：どのようなメッセージ型でどのトピック（トピック名）に送信するかを定義しておく
      2. メッセージ送信：事前に定義したメッセージ型を満たす具体的なメッセージを生成し送信する
  - サブスクリプションとは
    - ノードが保有する「特定のトピックからメッセージを受信する」仕組み。下記の２段階でメッセージを受信する。
      1. 事前準備：どのようなメッセージ型でどのトピック（トピック名）から受信するか、受信時にどのような処理を行うか（コールバック関数）を定義しておく
      2. メッセージ受信：メッセージがトピックにpublishされたタイミングで事前に定義したコールベック関数にメッセージが渡され実行される


## ノードの情報交換方法

ノードは下記の間での情報のやりとりを介して処理を行います
- ノードとノード間
- ノードとlaunchシステム間
- ノードとユーザ（コマンドラインインタフェース）間
- コントローラーノードとHWコンポーネント間
  - コントローラーノードとは、ros2_contorlで定義されるクラスを継承して作成されるクラス（ROS2コントローラー）が生成するノードであり、コントローラー管理ノードと同一プロセス中で実行されるノード。このプロセス中にプラグインとして読み込まれるクラスであるHWコンポーネントとコントローラーノードの間でHWへのコマンド送信やHWからの状態の受信が行われる


ノードは情報を入出力する為のIFとして下記の５種類を持ちます

| 情報入出力IF | 自プロセス内のノードとのIF | 他プロセス内のノードとのIF（プロセス間通信） | その他外部IF |
| ---- | ---- | ---- | ---- |
| トピック | 〇：トピック通信（メモリ渡し） | 〇：トピック通信 | - |
| サービス | 〇：サービス通信（メモリ渡し） | 〇：サービス通信 | - |
| アクション | 〇：サービス通信及びトピック通信（メモリ渡し） | 〇：サービス通信及びトピック通信 | - |
| ノードパラメータ | 〇：サービス通信（メモリ渡し） | 〇：サービス通信 | 〇：ROS引数による初期値設定 |
| command interface / state interface | 〇：読み書き（メモリ渡し） | - | - |

command interface / state interface以外は、異なるプロセス上・異なるマシン上のノード間でも情報をやりとりできるのがポイントです。command interfaceやstate interfaceは、ros2_contorolで使用するIFでありプロセス内の情報IFのみを持ちます（pluginによる結合です）

また、自プロセス内のノードとのIFはメモリ渡しになります

初歩的には１ノード=１プロセスとして、プロセスを分けてノードを実行することが多いですが、複数ノードを１プロセスで動作させることで効率的な動作にすることが可能です（プロセス間通信のようなエンコード・UDP通信・デコードが発生しない）。

- さらに、ひと手間加えると受け渡す情報をメモリ上でコピーすることを避けること（＝ゼロコピー）も可能。詳しくは「参考情報」内のリンクを参照

ノードが持つ通信IFによる情報のやりとりを全部羅列すると

- ノードに入ってくる情報

|  | 能動的（通信タイミングは自己決定） | 受動的（通信タイミングは不定） |
|---|---|---|
| トピック | - | サブスクライブしてメッセージを得る |
| サービス | 他ノードのサービスからレスポンスを得る | リクエスト引数を受け取る |
| アクション | 他ノードのアクションから結果を得る | ゴール引数を受け取る・フィードバックを得る |
| パラメータ | 他ノードのパラメータをリードする | ノードパラメータをライトされる/ROS引数によって初期値が設定される |
| state interface | HWコンポーネントからstateを得る | - |

- ノードから出ていく情報

|  | 能動的（通信タイミングは自己決定） | 受動的（通信タイミングは不定） |
|---|---|---|
| トピック | メッセージをパブリッシュする | - |
| サービス | 他ノードにリクエスト引数を渡す | レスポンスを返す |
| アクション | 他ノードにゴール引数を渡す/フィードバックを返す | 結果を返す |
| パラメータ | 他ノードのノードパラメータを書き換える | ノードパラメータをリードされ値を返す |
| command interface | HWコンポーネントにコマンドを送る | - |


# (参考)ソースの確認

まずは、サブスクリプションを生成する処理である`create_subscription()`メソッドの処理を追っていきます。処理の順に記載すると下記の流れになっています。

1. `Node::create_subscription()`
    - `rclcpp::create_subscription()`を呼ぶ
      - 引数の`topic_name`にノードサブ名前空間をつけて、`rclcpp::create_subscription()`に渡している。
        - ノードサブ名前空間は通常空文字である為、引数の`topic_name`そのものになる
      - 引数の`options`を`rclcpp::create_subscription()`に渡している
        - `options.callback_group`にはコールバックグループを指定でき重要な役割を果たす（後述）

[node_impl.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node_impl.hpp)

```cpp
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT>
Node::create_subscription(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const SubscriptionOptionsWithAllocator<AllocatorT> & options,
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat)
{
  return rclcpp::create_subscription<MessageT>(
    *this,
    extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
    qos,
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat);
}
```

2. `rclcpp::create_subscription()`
    - `rclcpp::detail::create_subscription()`を呼ぶだけ

[create_subscription.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/create_subscription.hpp)

```cpp
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType,
  typename NodeT>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeT & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  ),
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
    MessageMemoryStrategyT::create_default()
  )
)
{
  return rclcpp::detail::create_subscription<
    MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
    node, node, topic_name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
}
```

3. `rclcpp::detail::create_subscription()`
    - `rclcpp::create_subscription_factory()`を呼んで`SubscriptionFactory`を生成する。この時にメンバ変数`create_typed_subscription`に関数が設定される。
    - `NodeTopicInterface::create_subscription()`を呼んで`Subscription<MessageT, AllocatorT>`クラスのインスタンスを作成する
    - 上記で作成したインスタンスを`NodeTopicInterface::add_subscription()`に渡す
      - この時`options.callback_group`も取り出して渡している

[create_subscription.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/create_subscription.hpp)

```cpp
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT,
  typename NodeParametersT,
  typename NodeTopicsT,
  typename ROSMessageType = typename SubscriptionT::ROSMessageType>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeParametersT & node_parameters,
  NodeTopicsT & node_topics,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  ),
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
    MessageMemoryStrategyT::create_default()
  )
)
{
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics_interface = get_node_topics_interface(node_topics);

  // 略: subscription_topic_statsを得る処理

  auto factory = rclcpp::create_subscription_factory<MessageT>(
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat,
    subscription_topic_stats
  );

  const rclcpp::QoS & actual_qos = options.qos_overriding_options.get_policy_kinds().size() ?
    rclcpp::detail::declare_qos_parameters(
    options.qos_overriding_options, node_parameters,
    node_topics_interface->resolve_topic_name(topic_name),
    qos, rclcpp::detail::SubscriptionQosParametersTraits{}) :
    qos;

  auto sub = node_topics_interface->create_subscription(topic_name, factory, actual_qos);
  node_topics_interface->add_subscription(sub, options.callback_group);

  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}
```

4. `rclcpp::create_subscription_factory`
    - `SubscriptionFactory`が生成されるときにメンバ変数`create_typed_subscription`に下記の処理を行う関数が設定される
      - `Subscription<MessageT, AllocatorT>`のインスタンスを生成し、メンバ関数`post_init_setup()`を呼んだうえでインスタンスを戻り値として返す

```cpp
struct SubscriptionFactory
{
  // Creates a Subscription<MessageT> object and returns it as a SubscriptionBase.
  using SubscriptionFactoryFunction = std::function<
    rclcpp::SubscriptionBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      const rclcpp::QoS & qos)>;

  const SubscriptionFactoryFunction create_typed_subscription;
};

template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType
>
SubscriptionFactory
create_subscription_factory(
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat,
  std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>
  subscription_topic_stats = nullptr
)
{
  auto allocator = options.get_allocator();

  using rclcpp::AnySubscriptionCallback;
  AnySubscriptionCallback<MessageT, AllocatorT> any_subscription_callback(*allocator);
  any_subscription_callback.set(std::forward<CallbackT>(callback));

  SubscriptionFactory factory {
    // factory function that creates a MessageT specific SubscriptionT
    [options, msg_mem_strat, any_subscription_callback, subscription_topic_stats](
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      const rclcpp::QoS & qos
    ) -> rclcpp::SubscriptionBase::SharedPtr
    {
      using rclcpp::Subscription;
      using rclcpp::SubscriptionBase;

      auto sub = Subscription<MessageT, AllocatorT>::make_shared(
        node_base,
        rclcpp::get_message_type_support_handle<MessageT>(),
        topic_name,
        qos,
        any_subscription_callback,
        options,
        msg_mem_strat,
        subscription_topic_stats);
      // This is used for setting up things like intra process comms which
      // require this->shared_from_this() which cannot be called from
      // the constructor.
      sub->post_init_setup(node_base, qos, options);
      auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
      return sub_base_ptr;
    }
  };

  // return the factory now that it is populated
  return factory;
}
```

5. `NodeTopicInterface::create_subscription`
    - さきほど生成した関数`create_typed_subscription()`を呼ぶ。結果的に`Subscription<MessageT, AllocatorT>`のインスタンスを返す。型は`rclcpp::SubscriptionBase`にキャストされている。

[node_topics.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node_interfaces/node_topics.cpp)

```cpp
rclcpp::SubscriptionBase::SharedPtr
NodeTopics::create_subscription(
  const std::string & topic_name,
  const rclcpp::SubscriptionFactory & subscription_factory,
  const rclcpp::QoS & qos)
{
  // Create the MessageT specific Subscription using the factory, but return a SubscriptionBase.
  return subscription_factory.create_typed_subscription(node_base_, topic_name, qos);
}
```

6. `NodeTopics::add_subscription`
    - `CallbackGroup`クラスの`add_subscription()`を呼んで、引数で渡されてきたサブスクリプションを登録する
    - この時使用される`CallbackGroup`は、もともと`Node::create_subscription()`の引数の`options`で指定していなかった場合は、デフォルトコールバックグループ`node_base_->get_default_callback_group()`が使用される

[node_topics.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/node_interfaces/node_topics.cpp)

```cpp
void
NodeTopics::add_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  // Assign to a group.
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create subscription, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  callback_group->add_subscription(subscription);

  for (auto & key_event_pair : subscription->get_event_handlers()) {
    auto subscription_event = key_event_pair.second;
    callback_group->add_waitable(subscription_event);
  }

  auto intra_process_waitable = subscription->get_intra_process_waitable();
  if (nullptr != intra_process_waitable) {
    // Add to the callback group to be notified about intra-process msgs.
    callback_group->add_waitable(intra_process_waitable);
  }

  // Notify the executor that a new subscription was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on subscription creation: ") + ex.what());
  }
}

```