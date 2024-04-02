---
title: "ROS2を深く理解する：launchファイル編４　Nodeアクション"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-10-22 01:24"
---

# 解説対象
本記事では、ROS2のlaunch機能が提供するアクションの中で最も重要な`Node`アクションを解説します。
`LifecycleNode`アクションについては下記記事を参照ください。

https://zenn.dev/uedake/articles/ros2_launch5_lifecycle_node

１つのexecutableから複数のnodeを起動する場合は別の方法が存在します。下記記事を参照ください。

https://zenn.dev/uedake/articles/ros2_launch6_composable_node

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 公式ドキュメント
- [Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
  - 最初によむべきところ。内容はとてもあっさり、

# 解説

launchシステムは、launchファイル中にやりたい処理（＝アクション）をやりたい順序で記載していきますが、アクションの中で最も基本的なものはROSノードを動かす為のexecutableを起動するアクション（`Node`アクション/`LifecycleNode`アクション）です。

## `Node`アクションとは何か？

`Node`アクションを使用することでROSノード（`Node`や`LifecycleNode`）を生成するexecutableを起動できます

- 仕組み上は、ROSノードを起動しないexecutableの実行にも`Node`アクションを使用することはできますが、そのような使用は意味がないです（`Node`も`LifecycleNode`も起動しないexecutableを実行する場合は`ExecuteProcess`アクションを直接使うべき）
- `LifecycleNode`を起動するのに必ずしも`LifecycleNode`アクションを使う必要はなく`Node`アクションで十分ば場合も多い

`Node`アクションの基本の使い方は、コンストラクタで引数`package`と`executable_name`を指定し、executableの実行を指示することです

- そのほかROSノードの起動オプションとして下記が可能です
  - ノード名を指定する
    - 引数`name`に値を指定すると、コマンドライン引数`--ros-args --remap __node:={name}`を設定できる
    - 複数のROSノードを起動するexecutableに対してノード名を指定するとexecutableが起動する全てのROSノードのノード名が変わるの要注意
  - namespace名を指定する
    - 引数`namespace`に値を指定すると、コマンドライン引数`--ros-args --remap __ns:={namespace}`（"/"で始まる文字列を指定した場合）もしくは`--ros-args --remap __ns:={base_namespace}/{namespace}`（"/"で始まらない文字列を指定した場合）を設定できる
    - 引数`namespace`に値を指定しない場合、コマンドライン引数`--ros-args --remap __ns:={base_namespace}`を設定できる
    - `{base_namespace}`は`Node`アクションの前に`PushROSNamespace`アクションを実行することで設定できる
  - ノードパラメータを与える
    - 引数`parameters`に値を指定するor`Node`アクションの前に`SetParameter`アクション・`SetParametersFromFile`アクションを実行することで、コマンドライン引数`--ros-args --param {param_name}:={param_value}`もしくは`--ros-args --params-file {param_file_path}`を設定できる
  - remapルールを与える
    - 引数`remappings`に値を指定するor`Node`アクションの前に`SetRemap`アクションを実行することで、コマンドライン引数`--ros-args --remap {src}:={dst}`を設定できる
  - その他任意のコマンドラインROS引数（`ros2 run`コマンドで`--ros-args`と記載した後に指定できる引数）を与える
    - 引数`ros_arguments`に値を指定することで、コマンドライン引数`--ros-args {ros_arguments}`を設定できる
    - remapルールやノードパラメータの指定等ができるが、生でROS引数を指定せずとも、前述のようにremapルールやノードパラメータを指定する為の専用の方法が別途用意されているので、事実上明示的に`ros_arguments`を使う用途はない
  - その他任意のコマンドライン引数を与えて起動する（ROSノードを作成しないexecutableでも有効）
    - 引数`arguments`に値を指定することで、コマンドライン引数`{arguments}`を設定できる
    - この値は、executable実行時のエントリポイント（C++であれば通常main関数）の引数に渡される
    - ただし、ROSノードを起動するときにノード動作に影響を与える設定値はノードパラメータを用いて実装することがベストプラクティスであるので、`arguments`で引数を与える方法は使用しないことが望ましい
- `Node`アクションの引数`exec_name`を指定すると、プロセスに名前をつけられる。
  - このプロセス名はlog出力時に使用される。`exec_name`を指定しないとexecutable名がプロセス名になる。同じexecutableを複数起動する場合にはプロセス名をつけることが望ましい（デバッグをしやすくする）

なお、executableは複数のROSノードを起動することが可能です。

- launchファイルで指定したノード名指定・remap指定はそのexecutableから起動されるROSノード全てに影響します。
  - なぜなら、launchシステムの実装では、ノード名指定・remap指定の対象ROSノード名を指定する方法がないからです
  - コマンドライン引数では本来下記のように`{target_node}`（名前空間を含まないノード名）を記載すれば対象ROSノードを限定できます
    - `--ros-args --remap {target_node}:__ns:={namespace}`
    - `--ros-args --remap {target_node}:{src}:={dst}`
  - しかしlaunchシステムでは上記の記法をサポートしていません
- launchファイルで指定したノードパラメータ指定は原則そのexecutableから起動されるROSノード全てに影響します。
  - ただしノードパラメーターをyamlファイルで指定するときは、yamlファイル中の記法で対象ROSノード名を限定してノードパラメータを指定することが可能です

ノードパラメータ・remapの詳細は下記記事も参照ください。

https://zenn.dev/uedake/articles/ros2_node3_remap
https://zenn.dev/uedake/articles/ros2_node4_parameter


# (参考)ソースの確認

- `Node`アクションのソースをみてみます。まずは`__init__()`を見てみましょう。
  - `Node`アクションの`__init__()`では親クラスである`ExecuteProcess`アクションの初期化`super().__init__(cmd=cmd, **kwargs)`を呼んでいます
  - `ExecuteProcess`アクションは任意のコマンドを実行するアクションであり、インスタンス変数`cmd`に格納されているコマンドを実行します。
  - `Node`アクションでは、この`cmd`に「`Node`を起動するexecutable」を指定し各種コマンドライン引数を添えて実行していることがわかります
- `__init__()`中では下記のような`cmd`が生成されることがわかります
    - `{arguments}`,`{ros_arguments}`,`{name}`は`__init__()`の引数で与えた値を指します
    ```
    {path_to_executable} {arguments} --ros-args {ros_arguments} --ros-args -r __node:={name}
    ```

- ソースでは上記の`__node:={name}`のところは凝った作りになっており、substitutionの１つである`LocalSubstitution`を用いています。
  - `LaunchContext`のプロパティ`locals`（型は`Dict[Text, Any]`）の`locals['ros_specific_arguments']['name']`に文字列をセットした上で、`LocalSubstitution`を用いてその値を読み出すという実装になっています（なぜこのような回りくどい実装になっているのか不明です）


[node.py](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py)

```py:node.py
@expose_action('node')
class Node(ExecuteProcess):
    # 略

    def __init__(
        self, *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:

        # 略

        if package is not None:
            cmd = [ExecutableInPackage(package=package, executable=executable)]
        else:
            cmd = [executable]
        cmd += [] if arguments is None else arguments
        cmd += [] if ros_arguments is None else ['--ros-args'] + ros_arguments
        # Reserve space for ros specific arguments.
        # The substitutions will get expanded when the action is executed.
        cmd += ['--ros-args']  # Prepend ros specific arguments with --ros-args flag
        if name is not None:
            cmd += ['-r', LocalSubstitution(
                "ros_specific_arguments['name']", description='node name')]
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            # All elements in the list are paths to files with parameters (or substitutions that
            # evaluate to paths), or dictionaries of parameters (fields can be substitutions).
            normalized_params = normalize_parameters(parameters)
        # Forward 'exec_name' as to ExecuteProcess constructor
        kwargs['name'] = exec_name
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = executable
        self.__node_name = name
        self.__node_namespace = namespace
        self.__parameters = [] if parameters is None else normalized_params
        self.__remappings = [] if remappings is None else list(normalize_remap_rules(remappings))
        self.__ros_arguments = ros_arguments
        self.__arguments = arguments

        self.__expanded_node_name = self.UNSPECIFIED_NODE_NAME
        self.__expanded_node_namespace = self.UNSPECIFIED_NODE_NAMESPACE
        self.__expanded_parameter_arguments = None  # type: Optional[List[Tuple[Text, bool]]]
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_remappings = None  # type: Optional[List[Tuple[Text, Text]]]

        self.__substitutions_performed = False

        self.__logger = launch.logging.get_logger(__name__)

        self.__extensions = get_extensions(self.__logger)

```

    ```
    {path_to_executable} {arguments} --ros-args {ros_arguments} --ros-args -r __node:={name}
    ```

- 次にアクションの実行時の処理である`execute()`の実装を見てみます。下記の流れになっていることがわかります。
  1. `cmd`を拡張する
      - `__init__()`で作成していた`cmd`を下記のように拡張。
      ```
      {path_to_executable} {arguments} --ros-args {ros_arguments} --ros-args -r __node:={name} -r __ns:={namespace} -p {param_name}:={param_value} --params-file {param_file_path} -r {src}:={dst}
      ```
      - `{arguments}`,`{ros_arguments}`,`{name}`部分は`__init__()`の引数で与えた値を指します
      - `{namespace}`部分は(1)LaunchContextの`launch_configurations['ros_namespace']`と(2)`__init__()`の引数で与えた`namespace`、の２つから下記ロジックで生成されます（`prefix_namespace()`の実装を参照すること）
        - (1)がNoneの場合、`{namespace}`部分は(2)
        - (2)がNoneの場合、`{namespace}`部分は(1)
        - (2)が"/"始まりの場合、`{namespace}`部分は(2)
        - それ以外の場合、`{namespace}`部分は(1)/(2)
      - `-p {param_name}:={param_value} --params-file {param_file_path}`部分は記載の都合上１個のみ記載してますが、任意の数の指定が続きます。この指定は、下記２つのリストが順に使用されます。リストの要素は「(name,value)のタプル」もしくは「パス名」
        - LaunchContextの`launch_configurations['global_params']`
        - `__init__()`の引数で与えた`parameters`
      - `-r {src}:={dst}`部分は記載の都合上１個のみ記載してますが、任意の数の指定が続きます。この指定は、下記２つのリストが順に使用されます。リストの要素は「(src,dst)のタプル」
        - LaunchContextの`launch_configurations['ros_remaps']`
        - `__init__()`の引数で与えた`remappings`
  2. LaunchContextにデータを書き込む
      - `context.extend_locals()`メソッドを用いて`LaunchContext`のプロパティ`locals['ros_specific_arguments']`に値を書き込む
  3. 親クラスのexecute()を呼ぶ
      - `ExecuteProcess`アクションの`execute()`を呼ぶことで、`cmd`に指定したコマンドを実行する


```py:node.py
@expose_action('node')
class Node(ExecuteProcess):
    # 略

    def _perform_substitutions(self, context: LaunchContext) -> None:
        # Here to avoid cyclic import
        from ..descriptions import Parameter
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__expanded_node_name)
            self.__expanded_node_name.lstrip('/')
            expanded_node_namespace: Optional[Text] = None
            if self.__node_namespace is not None:
                expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            base_ns = context.launch_configurations.get('ros_namespace', None)
            expanded_node_namespace = make_namespace_absolute(
                prefix_namespace(base_ns, expanded_node_namespace))
            if expanded_node_namespace is not None:
                self.__expanded_node_namespace = expanded_node_namespace
                cmd_extension = ['-r', LocalSubstitution("ros_specific_arguments['ns']")]
                self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
                validate_namespace(self.__expanded_node_namespace)
        except Exception:
            self.__logger.error(
                "Error while expanding or validating node name or namespace for '{}':"
                .format('package={}, executable={}, name={}, namespace={}'.format(
                    self.__package,
                    self.__node_executable,
                    self.__node_name,
                    self.__node_namespace,
                ))
            )
            raise
        self.__final_node_name = prefix_namespace(
            self.__expanded_node_namespace, self.__expanded_node_name)

        # Expand global parameters first,
        # so they can be overridden with specific parameters of this Node
        # The params_container list is expected to contain name-value pairs (tuples)
        # and/or strings representing paths to parameter files.
        params_container = context.launch_configurations.get('global_params', None)

        if any(x is not None for x in (params_container, self.__parameters)):
            self.__expanded_parameter_arguments = []
        if params_container is not None:
            for param in params_container:
                if isinstance(param, tuple):
                    name, value = param
                    cmd_extension = ['-p', f'{name}:={value}']
                    self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
                else:
                    param_file_path = os.path.abspath(param)
                    self.__expanded_parameter_arguments.append((param_file_path, True))
                    cmd_extension = ['--params-file', f'{param_file_path}']
                    assert os.path.isfile(param_file_path)
                    self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])

        # expand parameters too
        if self.__parameters is not None:
            evaluated_parameters = evaluate_parameters(context, self.__parameters)
            for params in evaluated_parameters:
                is_file = False
                if isinstance(params, dict):
                    param_argument = self._create_params_file_from_dict(params)
                    is_file = True
                    assert os.path.isfile(param_argument)
                elif isinstance(params, pathlib.Path):
                    param_argument = str(params)
                    is_file = True
                elif isinstance(params, Parameter):
                    param_argument = self._get_parameter_rule(params, context)
                else:
                    raise RuntimeError('invalid normalized parameters {}'.format(repr(params)))
                if is_file and not os.path.isfile(param_argument):
                    self.__logger.warning(
                        'Parameter file path is not a file: {}'.format(param_argument),
                    )
                    continue
                self.__expanded_parameter_arguments.append((param_argument, is_file))
                cmd_extension = ['--params-file' if is_file else '-p', f'{param_argument}']
                self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
        # expand remappings too
        global_remaps = context.launch_configurations.get('ros_remaps', None)
        if global_remaps or self.__remappings:
            self.__expanded_remappings = []
        if global_remaps:
            self.__expanded_remappings.extend(global_remaps)
        if self.__remappings:
            self.__expanded_remappings.extend([
                (perform_substitutions(context, src), perform_substitutions(context, dst))
                for src, dst in self.__remappings
            ])
        if self.__expanded_remappings:
            cmd_extension = []
            for src, dst in self.__expanded_remappings:
                cmd_extension.extend(['-r', f'{src}:={dst}'])
            self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])    

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
        if self.__node_name is not None:
            ros_specific_arguments['name'] = '__node:={}'.format(self.__expanded_node_name)
        if self.__expanded_node_namespace != '':
            ros_specific_arguments['ns'] = '__ns:={}'.format(self.__expanded_node_namespace)

        # Give extensions a chance to prepare for execution
        for extension in self.__extensions.values():
            cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
                context,
                ros_specific_arguments,
                self
            )
            self.cmd.extend(cmd_extension)

        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})
        ret = super().execute(context)

        if self.is_node_name_fully_specified():
            add_node_name(context, self.node_name)
            node_name_count = get_node_name_count(context, self.node_name)
            if node_name_count > 1:
                execute_process_logger = launch.logging.get_logger(self.name)
                execute_process_logger.warning(
                    'there are now at least {} nodes with the name {} created within this '
                    'launch context'.format(node_name_count, self.node_name)
                )

        return ret

```

