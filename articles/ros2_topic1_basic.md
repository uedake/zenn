---
title: "ROS2を深く理解する：topic通信編１　基本構造"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、ROS2のNode間通信の主役であるtopic通信について解説します。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいていますが、rclpy側も結局はrclで規定されるnode実装につながりますので、大部分は共通です。

# 前提知識
- topic通信を行うには、nodeはどのようなtopic通信を行うかの事前定義（publisherとsubscriptionの生成）をしたうえで、topicの送信（=publish実行）と受信（=subscriberのコールバック実行）が必要です。
  - publisherとは
    - nodeが保有する「特定のトピックへメッセージを送信する」仕組み。どのようなメッセージ型でどのトピック（トピック名）に送信するかを定義する
  - subscriptionとは
    - nodeが保有する「特定のトピックからメッセージを受信する」仕組み。どのようなメッセージ型でどのトピック（トピック名）から受信するか、受信時にどのような処理を行うか（コールバック関数）を定義する
  - publishとは
    - publisherが有する機能であり、事前に定義したメッセージ型を満たす具体的なメッセージを与えることで、メッセージを送信する

# 公式ドキュメント

- TBD

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

まずは、subscriptionを生成する処理である`create_subscription()`メソッドの処理を追っていきます。処理の順に記載すると下記の流れになっています。

1. `Node::create_subscription()`
    - ソース：[node_impl.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node_impl.hpp)
    - 引数を設定して`rclcpp::create_subscription()`を呼ぶだけ
2. `rclcpp::create_subscription()`
    - ソース：[create_subscription.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/create_subscription.hpp)
    - 引数を設定して`rclcpp::detail::create_subscription()`を呼ぶだけ
3. `rclcpp::detail::create_subscription()`
    - ソース：下記引用の箇所
    - `NodeTopicInterface::create_subscription()`を呼んで`subscription`クラスのインスタンスを作成した後に、`NodeTopicInterface::add_subscription()`にその`subscription`と`callback_group`を渡して呼ぶ


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


# まとめ

