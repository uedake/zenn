---
title: "ROS2を深く理解する：launchファイル編２　substitution"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

# 前提
- ROS2 humble時の実装に基づいています。
- launchファイルの記述は、python形式・xml形式・yaml形式の３形式のどれでも可能ですが、本記事はpython形式について解説しています。
  - ※launchファイルは特段の理由ない限りpython形式で書くべきです。シンプルな構成であればどの形式でも記述可能ですが、複雑なことをする場合xml形式・yaml形式では行き詰まります。最初は良くてもプロジェクトの進展によって後から複雑なことをしたくなるのが常ですので、launchファイルは最初からpython形式で書き始めることを推奨します。

# 前提知識

# 公式ドキュメント

- TBD

# ソースの確認

結論だけ知りたい人は飛ばして「まとめ」へ

## substituionを理解する
次にlaucnhファイル中で使用できるsubstitutionについて解説します。

substitutionとは、「launchファイル実行時に決定する値」をlaunchファイル中に記述する方法です。何を言っているかよくわからないですよね・・・。



最初に`SomeSubstitutionsType`型が何かを見てみます。

「launchファイル実行時置換」

- `SomeSubstitutionsType`型は下記の通り定義されています
  - `Text`型
  - `Substitution`型
  - `Text`型か`Substitution`型からなるIterable(list等)


[some_substitutions_type.py](https://github.com/ros2/launch/blob/humble/launch/launch/some_substitutions_type.py)
```py:some_substitutions_type.py
SomeSubstitutionsType = Union[
    Text,
    Substitution,
    Iterable[Union[Text, Substitution]],
]
```

`Substitution`型とは、launch機能がサポートする各種substitutionの基底クラスであり、`perform()`メソッドを備えることが宣言されています。この`perform()`メソッドが実際に「launchファイル実行時置換」を行うメソッドです。


[python_expression.py](https://github.com/ros2/launch/blob/humble/launch/launch/substitutions/python_expression.py)
```py:python_expression.py
    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        from ..utilities import perform_substitutions
        return str(eval(perform_substitutions(context, self.expression), {}, math.__dict__))
```



