---
title: "ROS2を深く理解する：launchファイル編２　substitution"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: true
published_at: "2023-10-07 18:14"
---

# 解説対象

本記事では、ROS2のlaunch機能を理解する上での難所であるsubstitutionの仕組みを解説します。substitutionとは、「launchファイル記載の各種launchアクションが実行されるタイミングで決定する値」への参照をlaunchファイル中に記述する方法です。何を言っているかよくわからないですよね・・・。筆者も最初は理解があやふやでこの場面でsubstitutionを使う必要があるのか？と悩むことがありましたので本記事で解説します。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

# 前提

- ROS2 humble時の実装に基づいています
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています
  - ただし、xml形式・yaml形式で記載したlaunchファイルは、loaderによってpy形式のlaunchファイル相当に翻訳されて実行される為、本記事で記載している処理はpython形式・xml形式・yaml形式の３形式共通です

# 公式ドキュメント

- [Using-Substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)
  - substitutionの概要が記載されている最初によむべきところ。どんな使い方をするかの例示はされているが、どんな概念なのかは説明が足りない。
- [architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
  - substitutionの概念について少し解説がされている。が、どんな概念なのか理解するには説明が足りていない。

# 解説

## substitutionとは

公式ドキュメントである[Using-Substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)とか[architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)に依ればsubstitutionとは下記の概念です

```
Substitutions can be used in arguments to provide more flexibility when describing reusable launch files. Substitutions are variables that are only evaluated during execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.
```

```
A substitution is something that cannot, or should not, be evaluated until it's time to execute the launch description that they are used in.
```

- 公式ドキュメントによるとsubstitutionとは、
  - launch descriptionを実行する時まで評価されない変数である
  - launchアクションへの引数として使用できる
  - launchアクションへの引数として固定の値を渡す場合と比べると、launchファイルを柔軟にし再利用性を高めることができる
  - substitutionを使うと、例えば、launch設定の値や環境変数の値や任意のpython expressionの評価結果をlaunchアクションの引数として使用できるようになる

例えば環境変数を使いたい場合に、（python形式のlaunchファイルであれば）直接os.environとかで取得してlaunchアクションの引数とすることもできそうですが、環境変数を取得するsubstitutionを使うことと何が違うのか？これだけ読んでもピンときません・・・。

- 説明を補足するとsubstitutionとは、
  - 「launchファイル記載の各種launchアクションの実行フェーズで決定する値」への参照をlaunchファイル中に記述する方法です
  - この意味を理解するには、launchアクションは「読み込みフェーズ」と「実行フェーズ」の２段階のタイミングで処理される仕組みであることの理解が必要です
  - 「読み込みフェーズ」「実行フェーズ」等launchアクションの起動の仕組みについては下記の記事を参照ください。

https://zenn.dev/uedake/articles/ros2_launch1_basic

## pythonの変数を直接使うのと何が違うのか？

普通にpythonの変数を使うのとsubstitutionを使うのでは何が違うのか？という疑問について、解説します。

実用的な例ではないですが、下記の状況を考えてみます。
- 起動したいexecutable名が環境変数（`TARGET_EXECUTABLE_NAME`）にセットされており、環境変数の値によって起動するexecutableを変更したい

作り方として下記の２つの方法を考えます
- 方法例１：`os.environ`を使用して直接環境変数にアクセスする
- 方法例２：`EnvironmentVariable`substitutonを使用して環境変数にアクセスする

コードで書くと下記のような感じ

```py: example1
def generate_launch_description():

    selected_node = Node(
        package='hoge_package',
        executable=os.environ["TARGET_EXECUTABLE_NAME"],
    )
    return LaunchDescription([selected_node])
```

```py: example2
def generate_launch_description():

    selected_node = Node(
        package='hoge_package',
        executable=EnvironmentVariable("TARGET_EXECUTABLE_NAME"),
    )
    return LaunchDescription([selected_node])
```

上記の２つのコードは、「launchファイルを実行したタイミング」から`Node`アクションが実行されるまでの間に`NODE_NAME`が変更されないならまったく同一の処理になり、違いはありません。

違いをわかりやすくするために、上記`Node`アクションの前に別の`Node`アクションを追加した例を考えてみます。

```py: example1
def generate_launch_description():

    os.environ["TARGET_EXECUTABLE_NAME"]="node_default"
    switcher_node = Node(
        package='hoge_package',
        executable='selector',
    ) # this node changes TARGET_EXECUTABLE_NAME = node_hoge or node_fuga

    selected_node = Node(
        package='hoge_package',
        executable=os.environ["TARGET_EXECUTABLE_NAME"], # BAD!!!
    )
    return LaunchDescription([
        switcher_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=switcher_node,
                on_exit=[
                    selected_node
                ]
            )
        ),        
    ])
```

```py: example2
def generate_launch_description():

    os.environ["TARGET_EXECUTABLE_NAME"]="node_default"
    node = Node(
        package='hoge_package',
        executable='selector',
    ) # this node changes TARGET_EXECUTABLE_NAME = node_hoge or node_fuga

    selected_node = Node(
        package='hoge_package',
        executable=EnvironmentVariable("TARGET_EXECUTABLE_NAME"),
    )
    return LaunchDescription([
        switcher_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=switcher_node,
                on_exit=[
                    selected_node
                ]
            )
        ),        
    ])
```

上記例だと違いが明らかにでます。
例１の方は意図通りに動作しません。2つ目の`Node`アクションで`node_default`という名前のexecutableが意図に反して実行されてしまいます。
一方でsubstitutionを使用した例２では、1つ目の`Node`アクションが環境変数の書き込んだ値によって2つ目の`Node`アクションで何のexecutableが実行されるか分岐します。なぜなら、例１の書き方では2つ目の`Node`アクションの引数`executable`の値がlaunchファイルを読み込んだタイミングで確定してしまう（＝launchファイル読み込み時での環境変数の値が使用される）からです。例２のようにsubstitutionを使用した場合は、引数`executable`の値が`Node`アクションを実行するタイミングで決まる（最新の環境変数の値が使用される）ようになります。

## substitutionが使える場所

substitutionはどこでも使用できるわけではなく、substitutionを受け付けれるクラスは限定的です。使用可能なのは主に下記の５つです。
1. 各種launchアクションを定義する引数として使用
2. 各種substitutionを定義する引数として使用
    - substitutionを作成する時の引数としてsubstituitionが使えます。つまり、連鎖的にsubstitutionによる値評価をさせることが可能です
3. ノードパラメータを定義する引数として使用
    - `ParameterValue`クラス・`ParameterFile`クラス・`Parameter`クラスを定義する引数として使用します
    - これらのクラスは、`Node`アクションもしくは`ComposableNodeContainer`アクションを定義するときの引数として使用します
4. `ComposableNode`クラスを定義する引数として使用
    - `ComposableNode`クラスは、`ComposableNodeContainer`アクションを定義する際に使用するクラスです
5. `OnStateTransition`イベントハンドラを定義する引数として使用


substitutionの使いどころとしては、例えば複数のlaunchアクションを順に実行していく場合の条件分岐等です。例えば、先に実行したlaunchアクションの結果（例えば環境変数を変更する動作をする）に応じて後続のlaunchアクションの起動・非起動を分岐したり、ノードを起動するパラメータを変化させたりといったことが可能になります。

## substitutionの種類

[launchレポジトリ](https://github.com/ros2/launch/tree/humble/launch/launch/substitutions)と[launch_rosレポジトリ](https://github.com/ros2/launch_ros/tree/humble/launch_ros/launch_ros/substitutions)で定義されているsubstitutionを列挙すると下記になります

|substitutionクラス名|機能|引数|
|-|-|-|
|`Parameter`|指定の名前をキー名として`launch_configurations['global_params']`内を検索し見つけた値を返す。`launch_configurations['global_params']`にはノードパラメータの初期値が格納されているので、ノードパラメータ名を指定してノードパラメータ値を得ることに相当する|name:ノードパラメータ名|
|`ExecutableInPackage`|指定のパッケージ名と指定のexecutable名からそのパス文字列を得る|executable:executable名, package:パッケージ名|
|`AnonName`|指定の文字列を匿名化した文字列（ランダムに作成された文字列）に置き換える。変換の結果は`launch_configurations['anon'+name]`に格納され他から参照できる|name:文字列|
|`NotSubstitution`|指定された値を否定（NOT）した文字列(`true` or `false`)を得る|value:`1`、`0`、`true`、`false`|
|`AndSubstitution`|指定された値を論理積（AND）した文字列(`true` or `false`)を得る|left:`1`、`0`、`true`、`false`,right:`1`、`0`、`true`、`false`|
|`OrSubstitution`|指定された値を論理和（OR）した文字列(`true` or `false`)を得る|left:`1`、`0`、`true`、`false`,right:`1`、`0`、`true`、`false`|
|`Command`|指定された文字列をコマンドとして実行（subprocess.run()）した結果を得る|command:コマンド|
|`EnvironmentVariable`|指定の名前の環境変数の値を得る|name:環境変数名|
|`FindExecutable`|指定のexecutable名からそのパス文字列を得る。ExecutableInPackageとは異なり、環境変数のPATH以下でexecutableを探索する。|name:executable名|
|`LaunchConfiguration`|指定のキー名でlaunch_configurationsから値を得る|variable_name:LaunchConfigurationのキー名|
|`LocalSubstitution`|指定の文字列を用いてcontext.locals.に格納されている値を得る。`eval('context.locals.' + expression)`で取得するのでexpressionの書き方は内部構造がわかっていないと書けない。|expression:文字列|
|`PathJoinSubstitution`|指定のリスト中の要素をパス区切り文字で連結したパス文字列を得る|substitutions:結合対象要素のリスト|
|`PythonExpression`|指定のpythonのexpression文字列（例:`"math.sin(2*math.pi)"`）を評価(`eval()`)した結果を得る。expression内で使用できるpythonのパッケージは（humble時点では）mathのみ。expressionにはsubstitutionを指定可能なので例えば`["math.sin(",LaunchConfiguration(variable_name="hoge"),"*math.pi)"]`のような指定も可能|expression:pythonのexpression文字列|
|`TextSubstitution`|指定の文字列を得る。固定値しか与えられない為、存在理由が不明。おそらく過去の遺物。|text:文字列　※substitutionは使えずstrのみ|
|`ThisLaunchFile`|このlaunchファイルの絶対パスを得る|なし|
|`ThisLaunchFileDir`|このlaunchファイルの存在するディレクトリの絶対パスを得る|なし|

上記表中に登場するlaunch_configurationsとは、LaunchContextで保持している辞書です。詳しくは下記の記事を参照ください

https://zenn.dev/uedake/articles/ros2_launch3_configulation


# （参考）ソースの確認

全てのsubstitutionは、基底クラスである`Substitution`クラスを継承して作成されています。`Substitution`クラスでは`perform()`メソッドを備えることが宣言されています。この`perform()`メソッドが実際に「launchファイル実行時置換」を行うメソッドです。

[substitution.py](https://github.com/ros2/launch/blob/humble/launch/launch/substitution.py)

```py:substitution.py
class Substitution:
    """Encapsulates a substitution to be performed at runtime."""

    # 略

    # Note: LaunchContext is in a string here to break a circular import.
    def perform(self, context: 'LaunchContext') -> Text:
        """
        Perform the substitution, given the launch context, and return it as a string.

        This should be overridden by the derived classes, and the default
        raises NotImplementedError.

        :raises: NotImplementedError
        """
        raise NotImplementedError('perform() not implemented for Substitution base class.')
```

このsubstitutionを使用する場所は、大きく分けると下記５つです

1. 各種launchアクションを定義する引数として使用
2. 各種substitutionを定義する引数として使用
3. ノードパラメータを定義する引数として使用
4. `ComposableNode`クラスを定義する引数として使用
5. `OnStateTransition`イベントハンドラを定義する引数として使用

このうち主要な用途であり１と２の用途を以下で順にみていきます。

## substitutionをactionの引数として使用する

まずは、substitutionをactionの引数として使用する方法を取り扱います。actionの引数として利用するsubstitutionは、「アクション起動条件」として使用すること、「アクション実行用変数」として使用すること、の２つがありますのでそれぞれ確認します。


### アクション起動条件におけるsubstitutionの使用

- アクション起動条件（＝launchアクションをコンストラクトするときに渡せる引数`condition`で指定）には`Condition`クラス（を継承するクラス）を使用できます。

[condition.py](https://github.com/ros2/launch/blob/humble/launch/launch/condition.py)を見てみると`Condition`はコンストラクタの引数`predicate`（`Callable`型）で条件チェックの為の関数を受け取り、アクション起動条件が満たされているか確認する際（＝`evaluate()`が呼ばれる際）に、その関数を実行してlaunchアクションが起動可能か判定しています

[condition.py](https://github.com/ros2/launch/blob/humble/launch/launch/condition.py)

```py:condition.py
class Condition:
    """
    Encapsulates a condition to be evaluated when launching.

    The given predicate receives a launch context and is evaluated while
    launching, but must return True or False.

    If a predicate is not set when evaluated, False is returned.
    """

    def __init__(self, *, predicate: Optional[Callable[[LaunchContext], bool]] = None) -> None:
        self._predicate = predicate

    # 略

    def evaluate(self, context: LaunchContext) -> bool:
        """Evaluate the condition."""
        if self._predicate is not None:
            return self._predicate(context)
        return False
```

- [conditionsフォルダ](https://github.com/ros2/launch/tree/humble/launch/launch/conditions)を見てみると、アクション起動条件として使用できるのは、`IfCondition`、`UnlessCondition`、`LaunchConfigurationEquals`、`LaunchConfigurationNotEquals`の４つのいずれかであることがわかります
- ここでは試しに`IfCondition`を見てみます。わかかることは・・・
  - アクション起動条件判定用の関数は`_predicate_func()`メソッドであり、実体は`evaluate_condition_expression()`関数をコールしているだけである
  - そのコールの引数として、コンストラクタの引数`predicate_expression`を使用している
  - `predicate_expression`は`SomeSubstitutionsType`型となっており、substitutionが使用できる

[if_condition.py](https://github.com/ros2/launch/blob/humble/launch/launch/conditions/if_condition.py)

```py:if_condition.py
class IfCondition(Condition):
    """
    Encapsulates an if condition to be evaluated when launching.

    This condition takes a string expression that is lexically evaluated as a
    boolean, but the expression may consist of :py:class:`launch.Substitution`
    instances.

    See :py:func:`evaluate_condition_expression` to understand what constitutes
    a valid condition expression.
    """

    def __init__(self, predicate_expression: SomeSubstitutionsType) -> None:
        self.__predicate_expression = normalize_to_list_of_substitutions(predicate_expression)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return evaluate_condition_expression(context, self.__predicate_expression)
```

- 実際に`SomeSubstitutionsType`型の定義も見てみましょう。
- str型もしくはSubstitutionクラス及ぶそれらのIterableが`SomeSubstitutionsType`型と定義されています

[some_substitutions_type.py](https://github.com/ros2/launch/blob/humble/launch/launch/some_substitutions_type.py)

```py:some_substitutions_type.py
SomeSubstitutionsType = Union[
    Text,
    Substitution,
    Iterable[Union[Text, Substitution]],
]
```

- [evaluate_condition_expression_impl.py](https://github.com/ros2/launch/blob/humble/launch/launch/conditions/evaluate_condition_expression_impl.py)の実装もみてみましょう
- `perform_substitutions()`関数でsubstitutionを解決して文字列を得て、文字列が`"true"`もしくは`"1"`であるか？それとも`"false"`もしくは`"0"`であるかを判定することで、アクション起動条件が満たされるかの判断をしています

[evaluate_condition_expression_impl.py](https://github.com/ros2/launch/blob/humble/launch/launch/conditions/evaluate_condition_expression_impl.py)

```py:evaluate_condition_expression_impl.py
def evaluate_condition_expression(context: LaunchContext, expression: List[Substitution]) -> bool:
    """
    Expand an expression and then evaluate it as a condition, returning true or false.

    The expanded expression is stripped and has ``lower()`` called on it before
    being logically evaluated as either true or false.
    A string will be considered True if it matches 'true' or '1'.
    A string will be considered False if it matches 'false' or '0'.
    Any other string content (including empty string) will result in an error.

    :raises: InvalidConditionExpressionError
    """
    expanded_expression = perform_substitutions(context, expression)
    expanded_expression = expanded_expression.strip().lower()
    if expanded_expression in ['true', '1']:
        return True
    if expanded_expression in ['false', '0']:
        return False
    valid_expressions = VALID_TRUE_EXPRESSIONS + VALID_FALSE_EXPRESSIONS
    raise InvalidConditionExpressionError(expanded_expression, expression, valid_expressions)
```

ここまでで、アクション起動条件としてsubstitutionが使用できることがわかりました。具体的には、substitutionの評価結果（=perform()メソッドの戻り値）が文字列`1`、`0`、`true`、`false`（大文字小文字は問わない）のいずれかであるようなsubstitutionはアクション起動条件として使用できることがわかります。

### アクション実行用変数におけるsubstitutionの使用

- substitutionはアクション実行用変数としても使用でき、その様態は個々のlaunchアクションによって異なります。
- ここでは、例として`SetEnvironmentVariable`アクションを取り上げます。
  - `SetEnvironmentVariable`はコンストラクト時に引数としてnameとvalue（どちらも`SomeSubstitutionsType`型）を受け取っています。`SomeSubstitutionsType`型ですので、文字列だけでなくsubstitutionやそのリストも引数として使用できます。
  - launchアクションの実行時（＝`execute()`時）に`perform_substitutions()`メソッドを使用してメンバ変数nameとvalueから値を取り出しています。nameやvalueがsubstitutionであった場合、値はこの実行時に決まります。

[set_environment_variable.py](https://github.com/ros2/launch/blob/humble/launch/launch/actions/set_environment_variable.py)
```py:set_environment_variable.py
class SetEnvironmentVariable(Action):
    """Action that sets an environment variable."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetEnvironmentVariable action."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)
        self.__value = normalize_to_list_of_substitutions(value)

    # 略

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        context.environment[perform_substitutions(context, self.name)] = \
            perform_substitutions(context, self.value)
        return None
```

## substitutionの連鎖的使用（substitutionの引数としての使用）

- substitutionは他のsubstituion作成時の引数としても使えるので、複数のsubstitutionを組み合わせることができ、ある程度複雑な変換を行うことも可能です。
- substitutionの具体例として`PythonExpression`を見てみます。
  - `PythonExpression`ではコンストラクト時に引数としてexpression（`SomeSubstitutionsType`型）を受け取っています。`SomeSubstitutionsType`型ですので、文字列だけでなくsubstitutionやそのリストも引数として使用できます。
  - また、substitutionの解決時（＝`perform()`時）に`perform_substitutions()`メソッドを使用してメンバ変数expressionから値を取り出しています。expressionがsubstitutionであった場合、値はこの実行時に決まります。

[python_expression.py](https://github.com/ros2/launch/blob/humble/launch/launch/substitutions/python_expression.py)
```py:python_expression.py
class PythonExpression(Substitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    It also may contain math symbols and functions.
    """

    def __init__(self, expression: SomeSubstitutionsType) -> None:
        """Create a PythonExpression substitution."""
        super().__init__()

        ensure_argument_type(
            expression,
            (str, Substitution, collections.abc.Iterable),
            'expression',
            'PythonExpression')

        from ..utilities import normalize_to_list_of_substitutions
        self.__expression = normalize_to_list_of_substitutions(expression)

    # 略

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        from ..utilities import perform_substitutions
        return str(eval(perform_substitutions(context, self.expression), {}, math.__dict__))
```
