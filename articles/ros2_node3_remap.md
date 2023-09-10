---
title: "ROS2を深く理解する：Node編３　remap"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-09-03 03:11"
---

# 解説対象

本記事では、ROS2のNodeを扱ううえで非常に重要なremapについて解説します。remapはnode名とnode名前空間をnode起動時に書き換える処理です。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（rclcpp::Node）とライフサイクルを持つノード（rclcpp_lifecycle::LifecycleNode）の２種類がありますが、remapの扱いに関しては完全に同じ実装であり違いはありません。
- remapには、node名・node名前空間を対象にした置換だけでなく、topic名・service名を対象にしたものも存在しますが、ここではnode名・node名前空間の話に限定します。

# 前提知識

node名とnode名前空間について理解が曖昧な方はまず下記を読んでください。

https://zenn.dev/uedake/articles/ros2_node2_name

## なぜremapが必要なのか？

node名とnode名前空間はNodeを一意に識別する為の文字であり、Nodeを多数起動するときには一意な名前になるように配慮する必要があります。

でも、Nodeをconstructする時には自由にnode名とnode名前空間を指定できます。であれば「node名とnode名前空間をnode起動時に書き換える」なんて処理は一見不要にも思えます。

理解のカギとなるのは、executableとlaunchファイルからなる「nodeを起動する仕組み」です。Node自体は単なるクラスにしか過ぎずNodeを起動するには必ず「executableの実行」が起点となります（正確には実行されたexecutableの中でexecutorとNodeが作成され、executor上でNodeが実行されます）。このexecutableは実行プログラムでありNodeをconstructする時の引数として渡すnode名とnode名前空間は通常固定です。もちろんexecutableが取る実行時引数（args）で値を受け取って可変とすることも可能ですが、executable作る各人がそれぞれ別のルールでその処理をしだしたらNodeを使う側の人が大変苦労します。そんなことにならないよう、統一した仕組みで「executableから起動されるNode」のnode名とnode名前空間を書き換える為の方法が用意されました。

:::message alert
executableを自作する時「argsでnode名やnode名前空間を受け取る」なんて実装は絶対にしないように
:::

ROSの世界では、他人が作成したexecutableを起動してNodeを起動し連係させるという開発スタイルが当たり前ですので、remapの仕組みは必須になります。

また、remapの仕組みはlaunchファイルの仕組みとも相性よく設計されています。launchファイルによって多数のexecutableを起動することが可能ですが、統一されたremapの仕組みがあることでlaunchファイル中でも「このexecutable中のnodeはこの名前で起動して・・・」ということがスッキリと書けるようになっています。

# 公式ドキュメント

remapは結構ドキュメントが充実しています（が深く理解するには足りません）

コンセプト
http://design.ros2.org/articles/static_remapping.html

コマンドラインからのremappingの指定
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html?highlight=remapping

launchファイルからのremappingの指定
https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

でもいまいち理解しにくいのが
- remappingの指定はexecutableに対して行われる
- でもexecutable上で作成・実行されるNodeは１つとは限らない。複数のNodeが実行される場合もある
- remappingの指定は、どのNodeに対して適用されるの？？？

というあたり。（開発初期で1executable=1nodeで作成している間はこのあたり気にならないかも・・・）

ちなみに下記には1executableで複数nodeを起動する場合が記載されているが記述がcomponent形式でNodeを作成する場合に特化されていて、腹落ちしない。

https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html

気になったらソースを見ましょう。

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

## Nodeのconstructorでremappingの指定を受け取る

remappingの指定は、Nodeのconstructorの引数optionsで渡されてきます。渡す側の処理（launchファイルの仕組み等）は別記事にします。渡されたoptionsからoptions.context()及びoptions.get_rcl_node_options()で設定が取り出されてNodeBaseのconstructorへ渡されます。

解説の後ろで出てきますが、remapの指定はlocal指定とglobal指定の２つがあります。
- local指定
  - options.get_rcl_node_options()から得られるarguments(rcl_arguments_t型)に規定されたremapルール
- global指定
  - options.context()から得られるglobal_arguments(rcl_arguments_t型)に規定されたremapルール
  - options.get_rcl_node_options()から得られるuse_global_argumentsがtrueの場合のみ使用される

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
// 後略
```

## remappingの指定がremap処理に渡されるまで

`NodeBase`のconstructorでrcl nodeが作成され`NodeBase`のメンバ変数`node_handle_`に参照が設定されます。このrcl nodeがremappingの指定情報を含んでいます。

`NodeBase`のconstructor処理は、別記事で解説していますので省略します。

https://zenn.dev/uedake/articles/ros2_node1_basic

- remappingの指定情報は、下記でアクセス可能となっています
  - rcl nodeオプションを取り出す
    - `NodeBase`のメソッド`get_rcl_node_handle()`より`get_rcl_node_handle()->impl->options`でrcl nodeオプション（`rcl_node_options_t`構造体）が得られる。
  - rcl nodeオプションからremappingの指定情報を取り出す
    - `arguments.impl->remap_rules`でremappingの指定情報（`rcl_remap_t`型へのポインタ）が得られる


では、remappingの指定情報を表す`rcl_remap_t`型の定義を見てみましょう。

`rcl_remap_t`構造体は、`rcl_remap_impl_s`構造体のラッパーです。

[remap.h](https://github.com/ros2/rcl/blob/humble/rcl/include/rcl/remap.h)
```c:remap.h抜粋
typedef struct rcl_remap_impl_s rcl_remap_impl_t;

/// Hold remapping rules.
typedef struct rcl_remap_s
{
  /// Private implementation pointer.
  rcl_remap_impl_t * impl;
} rcl_remap_t;
```

`rcl_remap_impl_s`構造体には下記情報が設定されています

- 置換タイプ（remap対象がnode名なのか？node名前空間なのか？topic名なのか？service名なのか？）
- 対象とするnode名（NULLの場合すべてのnodeが対象となる）
- replacement指定文字列
- ※node名・node名前空間のremapにおいてmatchはNULL

[remap_impl.h](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/remap_impl.h)
```c:remap_impl.h抜粋
struct rcl_remap_impl_s
{
  /// Bitmask indicating what type of rule this is.
  rcl_remap_type_t type;
  /// A node name that this rule is limited to, or NULL if it applies to any node.
  char * node_name;
  /// Match portion of a rule, or NULL if node name or namespace replacement.
  char * match;
  /// Replacement portion of a rule.
  char * replacement;

  /// Allocator used to allocate objects in this struct
  rcl_allocator_t allocator;
};
```

## remap処理

rcl_node_init()では、受け取ったrcl_node_options_t構造体からrcl_arguments_t構造体を取り出してrcl_remap_node_name()及びrcl_remap_node_namespace()を呼び出してnode名とnode名前空間のremapを行います。

rcl_remap_node_name()及びrcl_remap_node_namespace()はどちらもrcl_remap_name()を呼び出します。

行われている処理は、
- 最初にlocalなrcl_arguments_t構造体で指定されるremapから適用可能なremapルールを探し
- 次にglobalなrcl_arguments_t構造体で指定されるremapから適用可能なremapルールを探し（localのほうでremapルールが見つかった場合はskipされる）

ここで適用可能なremapルールかの判定は、rcl_remap_impl_s構造体のnode_nameで決まります。node_nameがNULLでない場合、node_nameと一致するnode名をもつNodeのみが対象になります。適用可能なremapルールが複数ある場合、最初に見つかったルール１つのみが適用されます。

[remap.c](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/remap.c)

```c:remap.c抜粋
/// Remap from one name to another using rules matching a given type bitmask.
RCL_LOCAL
rcl_ret_t
rcl_remap_name(
  const rcl_arguments_t * local_arguments,
  const rcl_arguments_t * global_arguments,
  rcl_remap_type_t type_bitmask,
  const char * name,
  const char * node_name,
  const char * node_namespace,
  const rcutils_string_map_t * substitutions,
  rcl_allocator_t allocator,
  char ** output_name)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(node_name, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(output_name, RCL_RET_INVALID_ARGUMENT);
  if (NULL != local_arguments && NULL == local_arguments->impl) {
    local_arguments = NULL;
  }
  if (NULL != global_arguments && NULL == global_arguments->impl) {
    global_arguments = NULL;
  }
  if (NULL == local_arguments && NULL == global_arguments) {
    RCL_SET_ERROR_MSG("local_arguments invalid and not using global arguments");
    return RCL_RET_INVALID_ARGUMENT;
  }

  *output_name = NULL;
  rcl_remap_t * rule = NULL;

  // Look at local rules first
  if (NULL != local_arguments) {
    rcl_ret_t ret = rcl_remap_first_match(
      local_arguments->impl->remap_rules, local_arguments->impl->num_remap_rules, type_bitmask,
      name, node_name, node_namespace, substitutions, allocator, &rule);
    if (ret != RCL_RET_OK) {
      return ret;
    }
  }
  // Check global rules if no local rule matched
  if (NULL == rule && NULL != global_arguments) {
    rcl_ret_t ret = rcl_remap_first_match(
      global_arguments->impl->remap_rules, global_arguments->impl->num_remap_rules, type_bitmask,
      name, node_name, node_namespace, substitutions, allocator, &rule);
    if (ret != RCL_RET_OK) {
      return ret;
    }
  }
  // Do the remapping
  if (NULL != rule) {
    if (rule->impl->type & (RCL_TOPIC_REMAP | RCL_SERVICE_REMAP)) {
      // topic and service rules need the replacement to be expanded to a FQN
      rcl_ret_t ret = rcl_expand_topic_name(
        rule->impl->replacement, node_name, node_namespace, substitutions, allocator, output_name);
      if (RCL_RET_OK != ret) {
        return ret;
      }
    } else {
      // nodename and namespace rules don't need replacment expanded
      *output_name = rcutils_strdup(rule->impl->replacement, allocator);
    }
    if (NULL == *output_name) {
      RCL_SET_ERROR_MSG("Failed to set output");
      return RCL_RET_ERROR;
    }
  }
  return RCL_RET_OK;
}

/// Get the first matching rule in a chain.
/// \return RCL_RET_OK if no errors occurred while searching for a rule
static
rcl_ret_t
rcl_remap_first_match(
  rcl_remap_t * remap_rules,
  int num_rules,
  rcl_remap_type_t type_bitmask,
  const char * name,
  const char * node_name,
  const char * node_namespace,
  const rcutils_string_map_t * substitutions,
  rcutils_allocator_t allocator,
  rcl_remap_t ** output_rule)
{
  *output_rule = NULL;
  for (int i = 0; i < num_rules; ++i) {
    rcl_remap_t * rule = &(remap_rules[i]);
    if (!(rule->impl->type & type_bitmask)) {
      // Not the type of remap rule we're looking fore
      continue;
    }
    if (rule->impl->node_name != NULL && 0 != strcmp(rule->impl->node_name, node_name)) {
      // Rule has a node name prefix and the supplied node name didn't match
      continue;
    }
    bool matched = false;
    if (rule->impl->type & (RCL_TOPIC_REMAP | RCL_SERVICE_REMAP)) {
      // topic and service rules need the match side to be expanded to a FQN
      char * expanded_match = NULL;
      rcl_ret_t ret = rcl_expand_topic_name(
        rule->impl->match, node_name, node_namespace,
        substitutions, allocator, &expanded_match);
      if (RCL_RET_OK != ret) {
        rcl_reset_error();
        if (
          RCL_RET_NODE_INVALID_NAMESPACE == ret ||
          RCL_RET_NODE_INVALID_NAME == ret ||
          RCL_RET_BAD_ALLOC == ret)
        {
          // these are probably going to happen again. Stop processing rules
          return ret;
        }
        continue;
      }
      if (NULL != name) {
        // this check is to satisfy clang-tidy – name is always not null when type_bitmask is
        // RCL_TOPIC_REMAP or RCL_SERVICE_REMAP. That is guaranteed because rcl_remap_first_match
        // and rcl_remap_name are not public.
        matched = (0 == strcmp(expanded_match, name));
      }
      allocator.deallocate(expanded_match, allocator.state);
    } else {
      // nodename and namespace replacement apply if the type and node name prefix checks passed
      matched = true;
    }
    if (matched) {
      *output_rule = rule;
      break;
    }
  }
  return RCL_RET_OK;
}
```

# まとめ

上記のソースから考察すると下記がわかる

node名とnode名前空間のremapの処理では、
- 置換対象とするNodeを指定せず全Nodeを対象とすることも、node名を指定して特定のNodeのみを対象とすることもできる
- 置換対象とするNodeを指定した場合、指定するnode名を持つ全てのNodeが対象となる。node名前空間は関係ない。（下記例１）
- 1executable=1nodeの場合
  - 置換対象とするNodeを指定する必要はない
- 1executable=複数nodeの場合
  - Node名をremapする場合
    - 同じnode名前空間上に2以上のnodeがいる場合、名前衝突しやすいので注意を払う必要がある。（下記例２）
      - この衝突は、置換対象とするNodeを指定してremapことで回避しうる。（下記例３）
    - 同じnode名前空間上に2以上のnodeがいない場合、衝突の危険性はない。（下記例４）
  - Node名前空間をremapする場合
    - 同じnode名が存在する（別のnode名前空間に）executableでは、Node名前空間のremapをしてはいけない。必ず名前衝突する。（下記例５）
      - この衝突はremap対象とするNodeの指定では回避できない（Nodeの指定はnode名のみで指定可能な為）
    - 同じnode名が存在しないexecutableでは、Node名前空間のremapは可能。（下記例６）

| 例 | remap対象 | remap内容 | 置換前Node完全修飾名[^1] | 置換後Node完全修飾名 |
| ---- | ---- | ---- | ---- | ---- |
| 1 | node名x | node名をz | /nsA/xと/nsB/x | /nsA/zと/nsB/z |
| 2 | 指定なし | node名をz | /xと/y | 衝突（/z） |
| 3 | node名x | node名をz | /xと/y | /zと/y |
| 4 | 指定なし | node名をz | /nsA/xと/nsB/y | /nsA/zと/nsB/z |
| 5 | 指定なし | node名前空間を/nsC | /nsA/xと/nsB/x | 衝突（/nsC/x） |
| 6 | 指定なし | node名前空間を/nsC | /nsA/xと/nsB/y | /nsC/xと/nsC/y |

[^1]: node完全修飾名＝fully qualified name(node名前空間とnode名を結合した名前)