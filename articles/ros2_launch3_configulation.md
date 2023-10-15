---
title: "ROS2を深く理解する：launchファイル編３　launch引数とLaunchConfiguration"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-10-14 16:01"
---

# 解説対象
本記事では、ROS2のlaunch機能を理解する上で避けて通れないlaunch引数（launch argument）とlaunch configulationの仕組みを解説します。初学者にはlaunch引数（launch argument）とlaunch configulationって何が違うの？という点が混乱しやすいですので、解説していきます

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 前提知識
- launchファイルにおけるsubstitutionの仕組みの理解が必要です

下記の記事を参考にしてください

https://zenn.dev/uedake/articles/ros2_launch2_substitution

- launch引数（launch argument）は、launchファイルを`ros2 launch`コマンドで実行したり他のlaunchファイルから読み込んだり（=`IncludeLaunchDescription`アクションを実行）する際に、外部から値を与える為の仕組みです
- launch configulationとは、launchファイル内に定義するアクションから共有使用できる記憶領域です
  - launchファイルをpython形式でなくxml形式やyaml形式で記載する場合でも、launch configulationを用いることで値を受け渡しできるようになります
  - launchファイルをpython形式で記載する場合、通常のpythonの変数とlaunch configulationの違いは下記の通りです

## pythonの変数とlaunch configulationの違い

pythonの変数もlaunch configulationもどちらも変数としての性質（launchファイル実行の度に値が変わる可能性）を有しますが、下記の違いがあります

- pythonの変数
  - （substituionクラス以外の）python変数は、その値がアクション読み込み時点で確定します
- launch configulation
  - launch configulationは、その値がアクション読み込み時点で確定せず、アクション実行時点で確定します

# 公式ドキュメント

情報がとても少ない

- [Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
  - 使用例としては参考になるが、概念はよくわからない

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

- launch引数（launch argument）は`DeclareLaunchArgument`アクションをlaunchファイル中で定義することで使用できるようになります。まずはこの`DeclareLaunchArgument`アクションから見ていきましょう。

## `DeclareLaunchArgument`アクションを理解する

- `DeclareLaunchArgument`アクションの`execute()`を確認すると、その実態は「指定のキー名（`name`）でlaunch configulationが存在することを強制する」という処理であることがわかります
  - `DeclareLaunchArgument`アクションを定義するときにデフォルト値（`default_value`）を与えなかった場合、`DeclareLaunchArgument`アクション実行時にlaunch configulationに指定のキー名が存在しない場合には例外が発生するようになります
  - `DeclareLaunchArgument`アクションを定義するときにデフォルト値（`default_value`）を与えた場合、`DeclareLaunchArgument`アクション実行時にlaunch configulationに指定のキー名が存在しない場合には、そのデフォルト値がlaunch configulationに設定されます
- なお、launch configulationの値は、`LaunchContext`クラスのメンバ変数`launch_configurations`（keyもvalueもstr型のdict型）に格納されていることもわかります

どうやらlaunch引数を使用したいだけなら`DeclareLaunchArgument`アクションを使用せずとも任意のkey名の引数を暗黙的に使用できるようです。※ただし、そのような暗黙的な引数の利用はlaunchファイルの使用方法が不明瞭になり混乱のもとですので、基本的には必ず引数は`DeclareLaunchArgument`アクションを使用して宣言する方針がよいです

[declare_launch_argument.py](https://github.com/ros2/launch/blob/humble/launch/launch/actions/declare_launch_argument.py)

```py
class DeclareLaunchArgument(Action):
    # 略
    def __init__(
        self,
        name: Text,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
        description: Optional[Text] = None,
        choices: Iterable[Text] = None,
        **kwargs
    ) -> None:
        """Create a DeclareLaunchArgument action."""
        super().__init__(**kwargs)
        self.__name = name
        self.__logger = launch.logging.get_logger(__name__)
        if default_value is None:
            self.__default_value = default_value
        else:
            self.__default_value = normalize_to_list_of_substitutions(default_value)
        if choices is not None:
            if len(choices) == 0:
                self.__logger.error(
                    'Provided choices arg is empty. Use None to ignore the choice list.')
                raise RuntimeError(
                    'Provided choices arg is empty. Use None to ignore the choice list.')

            # Check if a non substitution default value is provided and is a valid choice
            if default_value is not None and not isinstance(default_value, (Substitution, list)):
                if default_value not in choices:
                    self.__logger.error(
                        'Provided default_value "{}" is not in provided choices "{}".'.format(
                            default_value, choices)
                    )
                    raise RuntimeError(
                        'Provided default_value "{}" is not in provided choices "{}".'.format(
                            default_value, choices))

        if description is None:
            if choices is None:
                self.__description = 'no description given'
            else:
                self.__description = 'One of: ' + str(choices)
        else:
            self.__description = description
            if choices is not None:
                if not self.__description.endswith('.'):
                    self.__description += '.'
                self.__description += ' Valid choices are: ' + str(choices)

        self.__choices = choices

        # This is used later to determine if this launch argument will be
        # conditionally visited.
        # Its value will be read and set at different times and so the value
        # may change depending at different times based on the context.
        self._conditionally_included = False

    # 略

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if self.name not in context.launch_configurations:
            if self.default_value is None:
                # Argument not already set and no default value given, error.
                self.__logger.error(
                    'Required launch argument "{}" (description: "{}") was not provided'
                    .format(self.name, self.description)
                )
                raise RuntimeError(
                    'Required launch argument "{}" was not provided.'.format(self.name))
            context.launch_configurations[self.name] = \
                perform_substitutions(context, self.default_value)

        if self.__choices is not None:
            value = context.launch_configurations[self.name]
            if value not in self.__choices:
                error_msg = ('Argument "{}" provided value "{}" is not valid. Valid options '
                             'are: {}'.format(self.name, value, self.__choices))
                self.__logger.error(error_msg)
                raise RuntimeError(error_msg)
```

## `LaunchConfiguration`を理解する

- launch引数の読み出しには、substitutionの１種である`LaunchConfiguration`を使用します。`perform()`の実装を見てみるとわかりますが、やっていることは非常に単純で指定のkey名（`variable_name`）でlaunch configulationの値を読み出すだけです。
  - key名のlaunch configulationがない場合、`LaunchConfiguration`の引数として`default`を与えて入ればその値が、与えていなければ例外が発生します

[launch_configuration.py](https://github.com/ros2/launch/blob/humble/launch/launch/substitutions/launch_configuration.py)

```py:launch_configuration.py
class LaunchConfiguration(Substitution):
    """Substitution that can access launch configuration variables."""

    def __init__(
        self,
        variable_name: SomeSubstitutionsType,
        *,
        default: Optional[Union[Any, Iterable[Any]]] = None
    ) -> None:
        """Create a LaunchConfiguration substitution."""
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions
        self.__variable_name = normalize_to_list_of_substitutions(variable_name)
        if default is None:
            self.__default = default
        else:
            # convert any items in default that are not a Substitution or str to a str
            str_normalized_default = []  # type: List[Union[Text, Substitution]]
            definitely_iterable_default = ((),)  # type: Iterable[Any]
            if isinstance(default, collections.abc.Iterable):
                definitely_iterable_default = default
            else:
                definitely_iterable_default = (default,)
            for item in definitely_iterable_default:
                if isinstance(item, (str, Substitution)):
                    str_normalized_default.append(item)
                else:
                    str_normalized_default.append(str(item))
            # use normalize_to_list_of_substitutions to convert str to TextSubstitution's too
            self.__default = \
                normalize_to_list_of_substitutions(
                    str_normalized_default)  # type: List[Substitution]

    # 略

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution by retrieving the launch configuration, as a string.

        If the launch configuration is not found and a default has been set,
        the default will be returned, as a string.
        """
        from ..utilities import perform_substitutions
        expanded_variable_name = perform_substitutions(context, self.__variable_name)
        if expanded_variable_name not in context.launch_configurations:
            if self.__default is None:
                raise SubstitutionFailure(
                    "launch configuration '{}' does not exist".format(expanded_variable_name))
            else:
                return perform_substitutions(context, self.__default)
        return context.launch_configurations[expanded_variable_name]
```

## launch configulationを理解する

- launch configulationの実体である`LaunchContext`クラスのプロパティ`launch_configurations`を見てみます。
  - keyもvalueもstr型のdict型であることがわかります
  - `launch_configurations`はスタックによるスコープ管理の仕組みを備えており、`_push_launch_configurations()`により新しいスコープを開始し`_pop_launch_configurations()`により現在のスコープを抜けることができるようになっています。スコープ開始後に`launch_configurations`に変化を与えても、スコープを抜けたときにスコープ開始時`launch_configurations`の状態に戻すということが可能になっています

:::alert
launch configulationは、明示的にスコープを切らない限りglobalな記憶領域として振る舞います。つまり、launchファイルから別のlaunchファイルを呼び出した場合でも、全てのlaunchファイルでlaunch configulationは共通の領域が使用されます。これは、キー名が意図せず衝突した場合に不具合の原因になります
:::


[launch_context.py](https://github.com/ros2/launch/blob/humble/launch/launch/launch_context.py)

```py:launch_context.py
class LaunchContext:
    """Runtime context used by various launch entities when being visited or executed."""

    def __init__(
        self,
        *,
        argv: Optional[Iterable[Text]] = None,
        noninteractive: bool = False
    ) -> None:
        # 略
        self.__launch_configurations_stack = []  # type: List[Dict[Text, Text]]
        self.__launch_configurations = {}  # type: Dict[Text, Text]
        # 略

    # 略

    def _push_launch_configurations(self):
        self.__launch_configurations_stack.append(self.__launch_configurations.copy())

    def _pop_launch_configurations(self):
        if not self.__launch_configurations_stack:
            raise RuntimeError('launch_configurations stack unexpectedly empty')
        self.__launch_configurations = self.__launch_configurations_stack.pop()

    @property
    def launch_configurations(self) -> Dict[Text, Text]:
        """Getter for launch_configurations dictionary."""
        return self.__launch_configurations
```

## `GroupAction`（launch configulationのスコープ制御）を理解する
- 「launch configurationsのスコープを切る」方法として`GroupAction`アクションが用意されています。
- `GroupAction`アクションのソースを見てみましょう。下記を順に実行することでスコープの分離を実現していることがわかります。（正確には、環境変数のスコープの分離も同時に実現していますが記載省略しています）
  1. `PushLaunchConfigurations`アクションの実行
      - 現在のlaunch configurationsを退避しておく（後で戻せるように）
  2. 新スコープのlaunch configurationsを初期化
      - `GroupAction`アクションの引数`forwarding`が`true`の時
        - `GroupAction`アクション実行時点で存在していたlaunch configurationsは、1実行時点ですべてコピーされて新スコープ内で使える状態で始まります。
        - `GroupAction`アクションの引数launch_configurationsを指定していた場合は、その値がlaunch configurationsに上書きされます
      - `GroupAction`アクションの引数`forwarding`が`false`の時
        - `ResetLaunchConfigurations`アクションを実行し、新スコープのlaunch configurationsが初期化されます。
        - 初期値は原則空ですが、`GroupAction`アクションの引数launch_configurationsを指定していた場合は、その値が初期値になります
  4. 引数で指定したアクションのリストを実行
  5. `PopLaunchConfigurations`アクションの実行
      - 1で退避していおいたlaunch configurationsに戻します
      - 2~4の中でlaunch configurationsに加えた影響は外部に及びません
      - 注：なお、`GroupAction`アクションの引数`scoped`を`false`にすると上記のようなスコープを切る挙動をOFFにできますが、用途が思いつきません

[group_action.py](https://github.com/ros2/launch/blob/humble/launch/launch/actions/group_action.py)

```py:group_action.py
class GroupAction(Action):
    # 略

    def __init__(
        self,
        actions: Iterable[Action],
        *,
        scoped: bool = True,
        forwarding: bool = True,
        launch_configurations: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **left_over_kwargs
    ) -> None:
        """Create a GroupAction."""
        super().__init__(**left_over_kwargs)
        self.__actions = actions
        self.__scoped = scoped
        self.__forwarding = forwarding
        if launch_configurations is not None:
            self.__launch_configurations = launch_configurations
        else:
            self.__launch_configurations = {}
        self.__actions_to_return: Optional[List] = None

    # 略

    def get_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Return subentities."""
        if self.__actions_to_return is None:
            self.__actions_to_return = list(self.__actions)
            configuration_sets = [
                SetLaunchConfiguration(k, v) for k, v in self.__launch_configurations.items()
            ]
            if self.__scoped:
                if self.__forwarding:
                    self.__actions_to_return = [
                        PushLaunchConfigurations(),
                        PushEnvironment(),
                        *configuration_sets,
                        *self.__actions_to_return,
                        PopEnvironment(),
                        PopLaunchConfigurations()
                    ]
                else:
                    self.__actions_to_return = [
                        PushLaunchConfigurations(),
                        PushEnvironment(),
                        ResetEnvironment(),
                        ResetLaunchConfigurations(self.__launch_configurations),
                        *self.__actions_to_return,
                        PopEnvironment(),
                        PopLaunchConfigurations()
                    ]
            else:
                self.__actions_to_return = [
                    *configuration_sets,
                    *self.__actions_to_return
                ]
        return self.__actions_to_return

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action."""
        return self.get_sub_entities()
```

- `GroupAction`アクションは、`ResetLaunchConfigurations`アクション・`PushLaunchConfigurations`アクション及び`PopLaunchConfigurations`アクションを用いることで「launch configurationsのスコープを切る」という動作を実現してくれています。これらのアクションを個別に呼ぶことで自前でスコープを切ることも可能ですが、launchファイルの可読性を高めるためには基本`GroupAction`アクションを使用すべきです

# まとめ
- launch configurationsとは、launchファイル内の各種アクションから読み書きできる記憶領域です
- launch引数で与えた値がlaunch configurationsに書き込まれる他、任意の値をアクション実行時に書き込むことが可能です
- launch configurationsを読むにはsubstitutionの１つである`LaunchConfiguration`クラスを用いる必要があります
- キー名の衝突に注意が必要です。特に`IncludeLaunchDescription`アクションを使用して外部のlaunchファイルを読み込むようなlaunchファイルの場合、呼び出し元のlaunchファイルと呼び出し先のlaunchファイル間でlaunch configurationsは（明示的にスコープを切らない限り）全て共有されます。
  - つまり、呼び出し先で使用するlaunch configurationsのキー名を把握せずにincludeしてしまうと、呼び出し先で使用するlaunch configurationsの値を意図せず渡してしまう（誤った値で・・・）ことや、呼び出し元で使用しているlaunch configurationsの値が呼び出し先で意図せず書き換えられてしまうことが起こりえます
  - そのような事態を避ける為の方法として、`GroupAction`アクションで`IncludeLaunchDescription`アクションを包んで呼び出すことで「launch configurationsのスコープを切る」ことができます
- launch configurationsが使用されるのは下記の時です（使用箇所を網羅的に検索して調べてみた結果）
- 値を読み出す時
  - substitution
    - `LaunchConfiguration`を用いることで`launch_configurations`中の指定キーの値を読み出せる
- 値により分岐する時
  - condition(アクション起動条件)
    - `LaunchConfigurationEquals`もしくは`LaunchConfigurationNotEquals`を用いることで`launch_configurations`中の指定キーの値が指定値と等しいかでアクションの起動可否を分岐できる
  - action
    - `ExecuteLocal`アクション中では、`launch_configurations`中のキー名`emulate_tty`の値を読み出すことでtty（仮想端末）をエミュレートするか分岐するようになっている
- 値を書き込む時
  - action
    - `SetLaunchConfiguration`アクションを用いることで`launch_configurations`中に指定のキー名で指定の値を書き込める
      - `ros2 launch`で与えたlaunch引数や`IncludeLaunchDescription`で与えたlaunch引数もこのアクションの実行によって処理される
    - `AnonName`アクションを用いることで`launch_configurations`中にキー名（＝'anon'+指定名）で指定名を匿名化した文字列を書き込める
- 値を宣言する時
  - action
    - `DeclareLaunchArgument`アクションを用いることで`launch_configurations`中に指定のキー名で値が存在する（or指定の選択肢のいずれかの値である）ことの強制（ない場合は例外発生させる）もしくは指定のキー名で値が存在しない場合のみ指定キー名に指定値を設定するという処理ができる
- キーを削除する時
  - action
    - `UnsetLaunchConfiguration`アクション
- リセットする時
  - action
    - `ResetLaunchConfigurations`アクションを用いることで`launch_configurations`を空（もしくは指定の辞書）にリセットできる
- スコープを分離する時
  - action
    - `PushLaunchConfigurations`アクション及び`PopLaunchConfigurations`アクションを用いることで、`launch_configurations`に対する変更の影響範囲を制限できる
      - `PushLaunchConfigurations`アクションによって新しいスコープを開始できる。以後`launch_configurations`への書き込みや変更を行っても`PopLaunchConfigurations`アクションを実行したら変更前の状態（＝`PushLaunchConfigurations`アクションを実行したタイミング）の`launch_configurations`に戻る
