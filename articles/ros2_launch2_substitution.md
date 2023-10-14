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

本記事では、ROS2のlaunch機能を理解する上での難所であるsubstitutionの仕組みを解説します。substitutionとは、「launchファイル記載のアクションが実行されるタイミングで決定する値」への参照をlaunchファイル中に記述する方法です。何を言っているかよくわからないですよね・・・。筆者も最初は理解があやふやでこの場面でsubstitutionを使う必要があるのか？と悩むことがありましたので本記事で解説します。

本記事は下記の「ROS2を深く理解する」の記事群の一部ですが、この記事単独でも理解できるようになっています。

https://zenn.dev/uedake/articles/ros2_collection

# 前提

- ROS2 humble時の実装に基づいています
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています
  - ただし、xml形式・yaml形式で記載したlaunchファイルは、loaderによってpy形式のlaunchファイル相当に翻訳されて実行される為、本記事で記載している処理はpython形式・xml形式・yaml形式の３形式共通です

# 前提知識

公式ドキュメントである[Using-Substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)とか[architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)に依ればsubstitutionとは下記の概念です

```
Substitutions can be used in arguments to provide more flexibility when describing reusable launch files. Substitutions are variables that are only evaluated during execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.
```

```
A substitution is something that cannot, or should not, be evaluated until it's time to execute the launch description that they are used in.
```

- substitutionとは、launch descriptionを実行する時まで評価されない変数である
- アクションへの引数として使用できる
- アクションへの引数として固定の値を渡す場合と比べると、launchファイルを柔軟にし再利用性を高めることができる
- substitutionを使うと、例えば、launch設定の値や環境変数の値や任意のpython expressionの評価結果をアクションの引数として使用できるようになる

例えば環境変数を使いたい場合に、（python形式のlaunchファイルであれば）直接os.environとかで取得してアクションの引数とすることもできそうですが、環境変数を取得するsubstitutionを使うことと何が違うのか？これだけ読んでもピンときません。どういう意味なのか本記事で解説してきます。

# 公式ドキュメント

- [Using-Substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)
  - substitutionの概要が記載されている最初によむべきところ。どんな使い方をするかの例示はされているが、どんな概念なのかは説明が足りない。
- [architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
  - substitutionの概念について少し解説がされている。が、どんな概念なのか理解するには説明が足りていない。

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

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

1. 各種アクションを定義する引数として使用する
2. 各種substitutionを定義する引数として使用する
    - substitutionを作成する時の引数としてsubstituitionが使えます。つまり、連鎖的にsubstitutioによる値評価をさせることが可能です
3. OnStateTransitionイベントハンドラを定義する引数として使用する
4. ParameterValueクラス・ParameterFileクラス・Parameterクラスを定義する引数として使用する
5. ComposableNodeクラスを定義する引数として使用する

このうち主要な用途であり１と２の用途を以下で順にみていきます。

## substitutionのactionの引数としての使用を理解する

まずは、substitutionをactionの引数として使用する方法を取り扱います。actionの引数として利用するsubstitutionは、「アクション起動条件」として使用すること、「アクション実行用変数」として使用すること、の２つがありますのでそれぞれ解説します。

なおアクションの起動の仕組みについては、下記の記事で解説しているので参考にしてください

https://zenn.dev/uedake/articles/ros2_launch1_basic


### アクション起動条件におけるsubstitutionの使用を理解する 

- アクション起動条件（＝アクションをコンストラクトするときに渡せる引数`condition`で指定）には`Condition`クラス（を継承するクラス）を使用できます。

[condition.py](https://github.com/ros2/launch/blob/humble/launch/launch/condition.py)を見てみると`Condition`はコンストラクタの引数`predicate`（`Callable`型）で条件チェックの為の関数を受け取り、アクション起動条件が満たされているか確認する際（＝`evaluate()`が呼ばれる際）に、その関数を実行してアクションが起動可能か判定しています

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

### アクション実行用変数におけるsubstitutionの使用を理解する 

- substitutionはアクション実行用変数としても使用でき、その様態は個々のアクションによって異なります。
- ここでは、例として`SetEnvironmentVariable`アクションを取り上げます。
  - `SetEnvironmentVariable`はコンストラクト時に引数としてnameとvalue（どちらも`SomeSubstitutionsType`型）を受け取っています。`SomeSubstitutionsType`型ですので、文字列だけでなくsubstitutionやそのリストも引数として使用できます。
  - アクションの実行時（＝`execute()`時）に`perform_substitutions()`メソッドを使用してメンバ変数nameとvalueから値を取り出しています。nameやvalueがsubstitutionであった場合、値はこの実行時に決まります。

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

## substitutionの連鎖的使用（substitutionの引数としての使用）を理解する

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


# まとめ

- substitutionとは、「launchファイル記載のアクションが実行されるタイミングで決定する値」への参照をlaunchファイル中に記述する方法です
  - この意味を理解するには、アクションは「読み込みフェーズ」と「実行フェーズ」の２段階のタイミングで処理される仕組みであることの理解が必要です
  - 読み込みフェーズとは、launchファイル中の`generate_launch_description()`メソッドが実行されるタイミングのことです。この段階ではアクションは処理待ちキューに入るだけで実行されません
  - 実行フェーズとは、実行の順番が来たアクションのvisit()が呼ばれるタイミングのことです。visit()はアクションの起動条件が満たされていれば個々のアクションのexecute()を呼びます。substituitionの値はこの段階で確定します。
- substitutionはどこでも使用できるわけではなく、substitutionを受け付けれるクラスは限定的です。使用可能なのは、各種アクションを定義する引数として使用、各種substitutionを定義する引数としての使用、OnStateTransitionイベントハンドラを定義する引数としての使用、nodeに渡すnodeパラメータを定義する引数としての使用、ComposableNodeを定義する引数としての使用、の５つです。
  - substitutionの引数としてsubstitutionを使用できることから、substitutionは連鎖的に適用可能な作りになっています。
- substitutionの使いどころとしては、例えば複数のアクションを順に実行していく場合の条件分岐等です。例えば、先に実行したアクションの結果（例えば環境変数を変更する動作をする）に応じて後続のアクションの起動・非起動を分岐したり、Nodeを起動するパラメータを変化させたりといったことが可能になります。

- [launchレポジトリ](https://github.com/ros2/launch/tree/humble/launch/launch/substitutions)と[launch_rosレポジトリ](https://github.com/ros2/launch_ros/tree/humble/launch_ros/launch_ros/substitutions)で定義されているsubstitutionを列挙すると下記になります
  - なお、launch_configurationsとは、LaunchContextで保持しているkeyもvakueも文字列である辞書です。詳しくは別記事を参照ください

|substitutionクラス名|機能|引数|
|-|-|-|
|Parameter|指定のnodeパラメータ名からnodeパラメータの値を得る|name:nodeパラメータ名|
|ExecutableInPackage|指定のパッケージ名と指定のexecutable名からそのパス文字列を得る|executable:executable名, package:パッケージ名|
|AnonName|指定の文字列を匿名化した文字列（ランダムに作成された文字列）に置き換える。変換の結果はlaunch_configurations['anon'+name]に格納され他から参照できる|name:文字列|
|NotSubstitution|指定された値を否定（NOT）した文字列(`true` or `false`)を得る|value:`1`、`0`、`true`、`false`|
|AndSubstitution|指定された値を論理積（AND）した文字列(`true` or `false`)を得る|left:`1`、`0`、`true`、`false`,right:`1`、`0`、`true`、`false`|
|OrSubstitution|指定された値を論理和（OR）した文字列(`true` or `false`)を得る|left:`1`、`0`、`true`、`false`,right:`1`、`0`、`true`、`false`|
|Command|指定された文字列をコマンドとして実行（subprocess.run()）した結果を得る|command:コマンド|
|EnvironmentVariable|指定の名前の環境変数の値を得る|name:環境変数名|
|FindExecutable|指定のexecutable名からそのパス文字列を得る。ExecutableInPackageとは異なり、環境変数のPATH以下でexecutableを探索する。|name:executable名|
|LaunchConfiguration|指定のキー名でlaunch_configurationsから値を得る|variable_name:LaunchConfigurationのキー名|
|LocalSubstitution|指定の文字列を用いてcontext.locals.に格納されている値を得る。`eval('context.locals.' + expression)`で取得するのでexpressionの書き方は内部構造がわかっていないと書けない。|expression:文字列|
|PathJoinSubstitution|指定のリスト中の要素をパス区切り文字で連結したパス文字列を得る|substitutions:結合対象要素のリスト|
|PythonExpression|指定のpythonのexpression文字列（例:`"math.sin(2*math.pi)"`）を評価(`eval()`)した結果を得る。expression内で使用できるpythonのパッケージは（humble時点では）mathのみ。expressionにはsubstitutionを指定可能なので例えば`["math.sin(",LaunchConfiguration(variable_name="hoge"),"*math.pi)"]`のような指定も可能|expression:pythonのexpression文字列|
|TextSubstitution|指定の文字列を得る。固定値しか与えられない為、存在理由が不明。おそらく過去の遺物。|text:文字列　※substitutionは使えずstrのみ|
|ThisLaunchFile|このlaunchファイルの絶対パスを得る|なし|
|ThisLaunchFileDir|このlaunchファイルの存在するディレクトリの絶対パスを得る|なし|

## 参考：launchファイルで記載したアクションの処理の流れ

### 読み込みフェーズ

1. launchファイルの`generate_launch_description()`メソッドの実行
    - これは、上位の`IncludeLaunchDescription`アクションが実行されることで始まります。`IncludeLaunchDescription`アクションの実行は、`ros2 launch`コマンドの実行もしくは他のlaunchファイル中で定義されている`IncludeLaunchDescription`アクションが実行されることで発生します
    - `generate_launch_description()`メソッドの中で定義しているアクションについて、アクションへの引数として（substitutionでない）通常の変数を使用していた場合、このタイミングで値は確定してしまいます
2. アクションが処理待ちキューに追加される
    - `generate_launch_description()`の戻り値（`LaunchDescription`クラス）に設定されている全てのアクションが実行待ちになります
      - このアクションは、`IncludeLaunchDescription`アクションから見た「サブアクション」にあたります。サブアクションという用語は、何かのアクションの結果生成されるアクションという意味です
    - アクションは「アクションを再帰的に実行する仕組み」の中で順次実行されるまで処理を待ちます。大切なのは、アクションは決して並列実行されることなく、シーケンシャルに１つ１つ実行されるということです

### 実行フェーズ
1. アクション起動条件チェック
    - 「アクション起動条件」の評価結果によって、アクションを起動するか抑制するか分岐します
    - アクション起動条件(`IfCondition`、`UnlessCondition`、`LaunchConfigurationEquals`、`LaunchConfigurationNotEquals`のいずれか)は、アクションをコンストラクトするときに渡せる引数`condition`に設定できます
    - アクション起動条件を作成する際に条件として指定できるのは「文字列」か「substitution」のどちらかです
      - 文字列はstr型で`1`、`0`、`true`、`false`（大文字小文字は問わない）のいずれかである必要があります
        - ただし、文字列を設定して条件分岐する方法は、積極的に使用する意味は一切ありません
          - python形式のlaunchファイルにおいては、pythonとしての通常のif分で条件分岐をしてアクションの追加・非追加を判定しても同じことができます
          - xml形式・yaml形式のlaunchファイルにおいては、文字列を固定値で与えることになる（アクションの実行・非実行はlaunchファイルを記載したタイミングで確定する）が、アクションを記述するしないで同等のことができる為
    - 意味のある使い方としては、アクション起動条件はsubstitutionを使用して設定することになります
    - substitutionの内容がアクション起動条件チェックを実施するタイミングで評価され、その値に応じてアクションを実行すべきか分岐します。なお、この評価結果は、文字列化したときに、`1`、`0`、`true`、`false`（大文字小文字は問わない）のいずれかである必要があります
2. アクションの実行
    - 各アクション毎の`execute()`メソッドによってアクションが実行されます
    - アクション実行において、アクションをコンストラクトした時の引数を使用するかはアクション毎に異なりますが、そのような引数があるアクションでは、substitutionを引数にいれておくことで、`execute()`メソッドが実行されたタイミングでsubstitutionが評価され値が決まります
