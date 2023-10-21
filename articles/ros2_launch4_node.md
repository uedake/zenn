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
本記事では、ROS2のlaunch機能が提供するアクションの中で最も重要なNodeアクションを解説します。

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 前提知識

- launchの概念
  - launchファイル中に、やりたい処理（＝アクション）をやりたい順序で記載する。アクションの中で最も基本的なものは、nodeを動かす為のexecutableを起動するアクション（Nodeアクション/LifecycleNodeアクション）
  - １つのexecutableから複数のnodeを起動する場合は別の方法が存在します（別記事で紹介）

# 公式ドキュメント
- [Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
  - 最初によむべきところ。内容はとてもあっさり、
- [Launching-composable-nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)
  - componentを用いて１つのexecutableから複数のnodeを起動する方法が解説されている

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

`Node`アクションのソースをみてみます

- `Node`アクションは、`ExecuteProcess`アクションを継承して実装されています。`ExecuteProcess`アクションは任意のコマンドを実行するアクションであり、インスタンス変数`cmd`に格納されているコマンドを実行します。`Node`アクションでは、この`cmd`に各種コマンドライン引数を添えて「`Node`を起動するexecutable」を指定し実行してくれます。
- ここのソースコードはsubstitutionの仕組みを使用した冗長な実装になっており読みにくいですが、下記の流れになっています
- `Node`アクションの実行（=`execute()`メソッド実行時の動作）
  1. コマンドを生成する
      - インスタント変数`cmd`（型は`Iterable[SomeSubstitutionsType]`）にコマンドが書き込まれます
      - この時substitutionの１つである`LocalSubstitution`を用いて、`LaunchContext`のプロパティ`locals`（型は`Dict[Text, Any]`）が保持するデータの`locals['ros_specific_arguments']['name']`及び`locals['ros_specific_arguments']['ns']`から値を取り出すようにしています
  2. LaunchContextにデータを書き込む
      - `LaunchContext`のプロパティ`locals['ros_specific_arguments']['name']`及び`locals['ros_specific_arguments']['ns']`で値を読めるように、`extend_locals()`メソッドを用いて値を書き込んでおきます
  3. 親クラスのexecute()を呼ぶ
      - `ExecuteProcess`アクションの`execute()`を呼ぶことで、`cmd`に指定したコマンドが実行されます
- なぜ直接コマンドを生成せずに上記のような実装になっているのかはわかりません（単に冗長なだけで無駄な実装に見えます）

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

# まとめ

- `Node`アクションの基本の使い方は、コンストラクタで引数`package`と`executable_name`を指定し、executableの実行を指示することです
  - 仕組み上は、Nodeを起動しないexecutableの実行にもNodeアクションを使用することはできますが、そのような使用は意味がないです（NodeもLifecycleNodeも起動しないexecutableを実行する場合は`ExecuteProcess`アクションを直接使うべき）
- executable実行時のコマンドライン引数のコントロールが可能であり、下記３種類に大別できます
  - 任意のexecutable引数を与えて起動する（Nodeを作成しないexecutableでも有効）
    - 引数`arguments`に値を指定する
    - この値は、executable実行時のエントリポイント（C++であれば通常main関数）の引数に渡される
    - ただし、node起動するときにnode動作に影響を与える設定値はnodeパラメータを用いて実装することがベストプラクティスであるので、`arguments`で引数を与える方法は使用しないことが望ましい
  - ROS引数を直接与えて起動する
    - 引数`ros_arguments`に値を指定する
    - ROS引数とは、`ros2 run`コマンドで`--ros-args`と記載した後に指定できる引数のこと
    - remappingやnodeパラメータの指定等ができるが、`Node`アクションには、生でROS引数を指定する代わりに、remappingやnodeパラメータを指定する為の専用の方法が別途用意されているので、事実上明示的に`ros_arguments`を使う用途はない
  - ROS引数を間接的に与えて起動する
    - executable上で実行されるnodeに影響を与える指定は全てROS引数（`ros2 run`コマンドで`--ros-args`と記載した後に指定することでexecutableが受け取る引数）で可能であるが、remappingやnodeパラメータの指定といったよく使う指定の為に、`Node`アクションのコンストラクタ引数として下記が用意されている
      - 引数`parameters`に値を指定することで、nodeパラメータを与えてNodeを起動できる
        - コマンドライン引数`--ros-args --param {}`もしくは`--ros-args --params-file {}`に翻訳される
      - 引数`remappings`に値を指定することで、remapの指定を与えてNodeを起動できる
        - コマンドライン引数`--ros-args --remap {}`に翻訳される
      - 引数`name`に値を指定することで、remapの指定を与えてNodeを起動できる
        - コマンドライン引数`--ros-args --remap __node:={}`に翻訳される
      - 引数`namespace`に値を指定することで、remapの指定を与えてNodeを起動できる
        - コマンドライン引数`--ros-args --remap __ns:={}`に翻訳される
- `Node`アクションの引数`exec_name`を指定すると、プロセスに名前をつけられる。このプロセス名はlog出力時に使用される。`exec_name`を指定しないとexecutable名がプロセス名になる。同じexecutableを複数起動する場合にはプロセス名をつけることが望ましい（デバッグをしやすくする）
