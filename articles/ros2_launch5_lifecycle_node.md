---
title: "ROS2を深く理解する：launchファイル編５　LifecycleNodeアクション"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象
本記事では、ROS2のlaunch機能が提供する`LifecycleNode`アクションを解説します。`Node`アクションとの違いは`LifecycleNode`特有の引数が使えることです。

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 前提知識

- launchの概念
  - launchファイル中に、やりたい処理（＝アクション）をやりたい順序で記載する。

# 公式ドキュメント
[TBD]

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

- `LifecycleNode`アクションのソースをみてみます。まずは`__init__()`を見てみましょう。
  - 単に親クラスである`Node`アクションの`__init__()`を使用してるだけのようです

[lifecycle_node.py](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/lifecycle_node.py)

```py:lifecycle_node.py
class LifecycleNode(Node):
    """Action that executes a ROS lifecycle node."""

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        **kwargs
    ) -> None:

        # 略

        super().__init__(name=name, namespace=namespace, **kwargs)
        self.__logger = launch.logging.get_logger(__name__)
        self.__rclpy_subscription = None
        self.__current_state = \
            ChangeState.valid_states[lifecycle_msgs.msg.State.PRIMARY_STATE_UNKNOWN]

```

- 次にアクションの実行時の処理である`execute()`の実装及び使用されている`get_ros_node()`の実装を見てみます。下記の流れになっていることがわかります。
  1. node名が指定されているチェック
      - `LifecycleNode`アクションは`Node`アクションと異なり、node名の指定（`__init__()`での`name`の指定）が必須となっている
      - よってnodeを複数個起動するようなexecutableの実行に`LifecycleNode`アクションを使用することは想定されていない様子
  2. `get_ros_node()`でlaunch_rosノードを得る
      - `get_ros_node()`は`LaunchContext`毎にただ１つ存在する`ROSAdapter`クラスを得る関数です（初めて呼ばれた場合は`ROSAdapter`クラスを生成する）
      - `ROSAdapter`クラスは、`__init__()`時にNodeを１つ生成（node名は`'launch_ros_{}'.format(os.getpid())`）しexecutor上で`Node`を実行します。
      - この`Node`はlaunchファイルの`OnShutdown`イベントが呼ばれたタイミングで終了します
      - つまり、launchシステムは、ユーザーが意図的に起動するnode（指定のexecutableで生成される）以外に１つのnodeを裏で起動します
      - このnodeを本記事ではlaunch_rosノードと呼びます
  3. launch_rosノードにtopicのsubscriptionとserviceのclientを設定
      - 

[lifecycle_node.py](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/lifecycle_node.py)

```py:lifecycle_node.py
class LifecycleNode(Node):

        # 略

    def _on_transition_event(self, context, msg):
        try:
            event = StateTransition(action=self, msg=msg)
            self.__current_state = ChangeState.valid_states[msg.goal_state.id]
            context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))
        except Exception as exc:
            self.__logger.error(
                "Exception in handling of 'lifecycle.msg.TransitionEvent': {}".format(exc))

    def _call_change_state(self, request, context: launch.LaunchContext):
        while not self.__rclpy_change_state_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                return

        # Asynchronously wait so that we can periodically check for shutdown.
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        response_future = self.__rclpy_change_state_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                response_future.cancel()
                return

        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        if not response.success:
            self.__logger.error(
                "Failed to make transition '{}' for LifecycleNode '{}'".format(
                    ChangeState.valid_transitions[request.transition.id],
                    self.node_name,
                )
            )

    def _on_change_state_event(self, context: launch.LaunchContext) -> None:
        typed_event = cast(ChangeState, context.locals.event)
        if not typed_event.lifecycle_node_matcher(self):
            return None
        request = lifecycle_msgs.srv.ChangeState.Request()
        request.transition.id = typed_event.transition_id
        context.add_completion_future(
            context.asyncio_loop.run_in_executor(None, self._call_change_state, request, context))

    def execute(self, context: launch.LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)  # ensure self.node_name is expanded
        if '<node_name_unspecified>' in self.node_name:
            raise RuntimeError('node_name unexpectedly incomplete for lifecycle node')
        node = get_ros_node(context)
        # Create a subscription to monitor the state changes of the subprocess.
        self.__rclpy_subscription = node.create_subscription(
            lifecycle_msgs.msg.TransitionEvent,
            '{}/transition_event'.format(self.node_name),
            functools.partial(self._on_transition_event, context),
            10)
        # Create a service client to change state on demand.
        self.__rclpy_change_state_client = node.create_client(
            lifecycle_msgs.srv.ChangeState,
            '{}/change_state'.format(self.node_name))
        # Register an event handler to change states on a ChangeState lifecycle event.
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeState),
            entities=[launch.actions.OpaqueFunction(function=self._on_change_state_event)],
        ))
        # Delegate execution to Node and ExecuteProcess.
        return super().execute(context)
```

[ros_adapters.py](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/ros_adapters.py)

```py:ros_adapters.py
def get_ros_adapter(context: launch.LaunchContext):
    """
    Get the ROS adapter managed by the given launch context.

    If no adapter is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    if not hasattr(context.locals, 'ros_adapter'):
        ros_adapter = ROSAdapter()
        context.extend_globals({'ros_adapter': ros_adapter})
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=lambda *args, **kwargs: ros_adapter.shutdown()
        ))
    return context.locals.ros_adapter


def get_ros_node(context: launch.LaunchContext):
    """
    Get the ROS node managed by the given launch context.

    If no node is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    return get_ros_adapter(context).ros_node
```

# まとめ

