---
title: "ROS2を深く理解する：ノード編３　remap"
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

本記事では、ROS2のノードを扱ううえで非常に重要なremapについて解説します。remapはノード名／ノード名前空間／トピック名／サービス名をノード起動時に書き換える処理です。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

## 目標

本記事の目標は、remapで行われている置換処理の詳細を理解することで、完全修飾ノード名／完全修飾トピック名／完全修飾サービス名が衝突しないようなremapルールを記述するにはどのようにすれば良いかを理解することです。

大規模なロボットシステムを設計する場合には、remapを使用して名称が衝突しないよう配慮が大事ですので正しく理解しましょう。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（`rclcpp::Node`）とライフサイクルを持つノード（`rclcpp_lifecycle::LifecycleNode`）の２種類がありますが、remapの扱いに関しては完全に同じ実装であり違いはありません。

# 公式ドキュメント

remapは結構ドキュメントが充実しています（が深く理解するには足りません）

コンセプト
http://design.ros2.org/articles/static_remapping.html

コマンドラインからのremapルールの指定
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html?highlight=remapping

launchファイルからのremapルールの指定
https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

でもいまいち理解しにくいのが
- remapルールの指定はexecutableに対して行われる
- でもexecutable上で作成・実行されるノードは１つとは限らない。複数のノードが実行される場合もある
- remapルールの指定は、どのノードに対して適用されるの？？？

というあたり。（開発初期で1executable=1ノードで作成している間はこのあたり気にならないかも・・・）

ちなみに下記には1executableで複数ノードを起動する場合が記載されているが記述がcomponent形式でノードを作成する場合に特化されていて、腹落ちしない。

https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html

ドキュメント見て良くわからないのでソースを見て理解をした結果を本記事にまとめています

# 解説

ノード名とノード名前空間について理解が曖昧な方はまず下記を読んでください。

https://zenn.dev/uedake/articles/ros2_node2_name

## なぜremapが必要なのか？

ノード名とノード名前空間の結合で決まる完全修飾ノード名はノードを一意に識別する為の文字列であり、ノードを多数起動するときには一意になるように配慮する必要があります。

ノードを起動するには必ず「executableの実行」が起点となります（正確には実行されたexecutableの中でexecutorとノードが作成され、executor上でノードが実行されます）。このexecutableは実行プログラムでありノードをconstructする時の引数として渡すノード名とノード名前空間を自由に指定できます。であれば「ノード名とノード名前空間を書き換える」という処理は一見不要にも思えます。（ソースが配布されているならexecutableの実装を書き換えてコンパイルしなおすという方法がとれるという意味）

remapが必要なのは、executableを書き換えずに（再コンパイルせずに）名前の衝突を避ける為です。自分が全部のexecutableを開発しているシステムであれば、このようなremapは不要かもしれませんが、ROS2では通常他人が作成した多数のexecutableと自分が作成したexecutableを組み合わせてシステムを構築します。その場合、他人が作成したexecutableの書き換えはしたくないです。

もちろんexecutableの書き換えをせずにノード名やノード名前空間を可変にする為の方法としては、executableが取る実行時引数（args）受け取った値を使用するという方法も原理的には取可能です。しかし、executable作る各人がそれぞれ別のルールでその処理をしだしたらノードを使う側の人が大変苦労します。そんなことにならないよう、統一した仕組みで「executableから起動されるノード」のノード名とノード名前空間を書き換える為の方法が用意されました。

:::message alert
executableを自作する時「argsでノード名やノード名前空間を受け取る」なんて実装は絶対にしないように
:::

また、remapの仕組みはlaunchファイルの仕組みとも相性よく設計されています。launchファイルによって多数のexecutableを起動することが可能ですが、統一されたremapの仕組みがあることでlaunchファイル中でも「このexecutable中のノードはこの名前で起動して・・・」ということがスッキリと書けるようになっています。

上記では、ノード名について述べていますが、トピック名／サービス名／アクション名も情報の入出力ＩＦの宛先として同様にremapすることでノードが望むように連携させることが重要です。

## remapルールとは？
remapルールとは「何をどんな条件で何に書き換えるのか？」の指定です。
- 何を？
  - 対象として、ノード名／ノード名前空間／トピック名／サービス名の４種類が指定できます
    - アクションは実体としてトピックとサービスから構成され、アクション名から紐づくトピック名/サービス名が生成されるので、トピック名／サービス名のremapができればアクション名のremapもできる
- どんな条件で？
  - ノード名・ノード名前空間名のreampルールでは、置換対象とするノード名を指定することができる（指定しない場合そのexecutable中の全ノードが対象になる）。
  - トピック名・サービス名のreampルールでは、置換対象とするトピック名・サービス名を指定する他、置換対象とするノード名を指定することもできる（指定しない場合そのexecutable中の全ノードが対象になる）。
  - このノード名の指定は完全修飾でない素のノード名でおこなう（ノード名前空間に依存しない）
- 何に？
  - 書き換え後の名前を指定します

そしてremapルールを指定する場所は、local指定とglobal指定の２つがあります。
- local指定
  - `NodeOption`の`arguments`(`rcl_arguments_t`型)内で指定するremapルール
  - executable内での起動するノード１個１個毎に別の指定が可能
- global指定
  - グローバルデフォルトコンテキスト（`NodeOption`の`context()`から得られる）が持つ`global_arguments`(`rcl_arguments_t`型)内で持つremapルール。ここには`ros2 node`コマンドでのコマンドライン引数やlaunchファイルの`Node`アクションの引数として指定したremapルールが格納されます
  - executable内での起動するノード全てに同じ指定となる
    - ただし`NodeOption`の`use_global_arguments`をfalseで起動したノードには適用されないのでノード個別にglobal指定の影響を受けないようにすることは可能

グローバルデフォルトコンテキストについては下記記事を参照ください

https://zenn.dev/uedake/articles/ros2_node5_context


## ノード名・ノード名前空間のremapルールの指定

executableが複数のノードを起動する場合、global指定のremap指定は原則全ノードに及ぶのでremapルールは注意深い指定が必要です。下記に例を示します。

- 1executable=1ノードの場合
  - 置換対象とするノードを指定しなくてもしてもどちらでもよい
- 1executable=複数ノードの場合
  - 同じノード名前空間上に2以上のノードがいる場合（よくあるケース）
    - ノード名のremapは、置換対象とするノードを指定せずに行ってはいけない。名前が衝突する（下記例１）
    - ノード名のremapは、置換対象とするノードを指定すれば衝突を回避しうる（下記例２）
    - ノード名前空間のreampは、置換対象とするノードを指定しなくてもしてもどちらでもよい（下記例７・８）
  - 異なるノード名前空間上に異なるノード名の2以上のノードがいる場合
    - ノード名のremapは、置換対象とするノードを指定しなくてもしてもどちらでもよい（下記例３・４）
    - ノード名前空間のremapは、置換対象とするノードを指定しなくてもしてもどちらでもよい（下記例９・１０）
  - 異なるノード名前空間上に同じノード名の2以上のノードがいる場合
    - ノード名のremapは、置換対象とするノードを指定しなくてもしてもどちらでもよい（下記例５・６）
    - ノード名前空間のremapは、置換対象とするノードを指定しないと名前衝突する（下記例１１）
    - ノード名前空間のremapは、置換対象とするノードを指定すれば衝突を回避しうる（下記例１２）


| 例 | remap条件 | remap内容 | 置換前完全修飾ノード名[^1] | 置換後完全修飾ノード名 |
| ---- | ---- | ---- | ---- | ---- |
| 1 | 指定なし | ノード名をz | /nsA/xと/nsA/y | 衝突（/nsA/z） |
| 2 | ノード名x | ノード名をz | /nsA/xと/nsA/y | /nsA/zと/nsA/y |
| 3 | 指定なし | ノード名をz | /nsA/xと/nsB/y | /nsA/zと/nsB/z |
| 4 | ノード名x | ノード名をz | /nsA/xと/nsB/y | /nsA/zと/nsB/y |
| 5 | 指定なし | ノード名をz | /nsA/xと/nsB/x | /nsA/zと/nsB/z |
| 6 | ノード名x | ノード名をz | /nsA/xと/nsB/x | /nsA/zと/nsB/z |
| 7 | 指定なし | ノード名前空間を/nsC | /nsA/xと/nsA/y | /nsC/xと/nsC/y |
| 8 | ノード名x | ノード名前空間を/nsC | /nsA/xと/nsA/y | /nsC/xと/nsA/y |
| 9 | 指定なし | ノード名前空間を/nsC | /nsA/xと/nsB/y | /nsC/xと/nsC/y |
| 10 | ノード名x | ノード名前空間を/nsC | /nsA/xと/nsB/y | /nsC/xと/nsB/y |
| 11 | 指定なし | ノード名前空間を/nsC | /nsA/xと/nsB/x | 衝突（/nsC/x） |
| 12 | ノード名x | ノード名前空間を/nsC | /nsA/xと/nsB/x | /nsC/xと/nsB/x |

[^1]: 完全修飾ノード名＝fully qualified name(ノード名前空間とノード名を結合した名前。システム全体で一意である必要がある。)

## トピック名・サービス名のremapルールの指定
TBD

# （参考）ソースの確認

上記の解説内容について実際にソースコードを追って確認していきます。

## Nodeの実装を確認する

remapルールは、`Node`のconstructorの引数`options`で渡されてきます。渡す側の処理（launchファイルの仕組み等）は別記事にします。渡された`options`から`options.context()`及び`options.get_rcl_node_options()`で設定が取り出されて`NodeBase`のconstructorへ渡されます。


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

## NodeBaseの実装を確認する

`NodeBase`のconstructorでrclノードが作成され`NodeBase`のメンバ変数`node_handle_`に参照が設定されます。このrclノードがremapルールを含んでいます。

`NodeBase`のconstructor処理は、別記事で解説していますので省略します。

https://zenn.dev/uedake/articles/ros2_node1_basic

- remapルールは、下記でアクセス可能となっています
  - rclノードオプションを取り出す
    - `NodeBase`のメソッド`get_rcl_node_handle()`より`get_rcl_node_handle()->impl->options`でrclノードオプション（`rcl_node_options_t`構造体）が得られる。
  - rclノードオプションからremapルールを取り出す
    - `arguments.impl->remap_rules`でremapルール（`rcl_remap_t`型へのポインタ）が得られる

rclノードの作成では、`rcl_node_init()`が使用されますが、この関数の中でremap処理の為の関数が呼び出されます。remap処理を行っている関数は、`rcl_remap_node_name()`及び`rcl_remap_node_namespace()`です。

## remapルールの型を確認する

remap処理を行っている関数を見る前に、remapルールを表す`rcl_remap_t`型の定義を見てみましょう。

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

- 置換タイプ（remap対象がノード名なのか？ノード名前空間なのか？トピック名なのか？サービス名なのか？）
- 対象とするノード名（NULLの場合すべてのノードが対象となる）
- replacement指定文字列
- ※ノード名・ノード名前空間のremapにおいてmatchはNULL

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

## remap処理を確認する

remap処理を行う関数である
- `rcl_remap_node_name()`
- `rcl_remap_node_namespace()`
- `rcl_resolve_name()`
はどれも`rcl_remap_name()`を呼び出します。

`rcl_remap_name()`で行われている処理は、
- 最初にlocalな`rcl_arguments_t`構造体で指定される`remap`から適用可能なremapルールを探す
- 次にglobalな`rcl_arguments_t`構造体で指定される`remap`から適用可能なremapルールを探す（localのほうでremapルールが見つかった場合はskipされる）

remapルール（`rcl_remap_impl_s`構造体）視点でみるとルールが適用可能な対象かの判定は、
- ノード名・ノード名前空間のremapの場合
  - remapルールの`node_name`で絞り込まれる。`node_name`がNULLでない場合、`node_name`と一致するノード名をもつノードのみが対象になる。（このノード名は完全修飾ノード名ではない素のノード名）
- トピック名・サービス名のremapの場合
  - remapは完全修飾名に対して行われる
  - まずremapルールの`node_name`で絞り込まれる。`node_name`がNULLでない場合、`node_name`と一致するノード名をもつノードが生成するPublisher/Subscription/ServiceServer/ServiceClientのみが対象になる。（このノード名は完全修飾ノード名ではない素のノード名）
  - 次にreampルールの`match`で絞り込まれる。`match`（指定のトピック名もしくはサービス名）を完全修飾名にした名前と、オリジナルのトピック名・サービス名を完全修飾名にした名前が一致する場合のみ対象となる

適用可能なremapルールが複数ある場合、最初に見つかったルール１つのみが適用されます。

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

