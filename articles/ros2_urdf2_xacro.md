---
title: "ROS2を深く理解する：urdf編２　xacro"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

## 目標

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいています。
- ノードには、ライフサイクルを持たないノード（`rclcpp::Node`）とライフサイクルを持つノード（`rclcpp_lifecycle::LifecycleNode`）の２種類がありますが、ノード名とノード名前空間の扱いに関しては完全に同じ実装であり違いはありません。

# 公式ドキュメント

# 解説

# （参考）ソースの確認

## xacroマクロのパラメータの解釈

xacroマクロのを呼び出すときに指定するパラメータはstr型で入力されます。xacroファイルをパースしてマクロ定義の展開を行っているコードは下記のxacro/__init__.pyで定義されていますが、そのファイル中の`eval_text()`がxacroマクロ呼び出しにおけるパラメータの値の解釈をしている箇所になります。

- xacroマクロ引数の値は単なるstr型として入力されてきていることがわかります
  - マクロを呼ぶ側が`hoge="1"`を入力した場合、これはstr型の`"1"`として受け取られる

[xacro/__init__.py](https://github.com/ros/xacro/blob/53f71c2f667bfdc2008e5ea2583cc01501b13b82/xacro/__init__.py#L686C1-L716C42)

```py:xacro/__init__.py
def eval_text(text, symbols):
    def handle_expr(s):
        try:
            return safe_eval(eval_text(s, symbols), symbols)
        except Exception as e:
            # re-raise as XacroException to add more context
            raise XacroException(exc=e,
                                 suffix=os.linesep + "when evaluating expression '%s'" % s)

    def handle_extension(s):
        return eval_extension("$(%s)" % eval_text(s, symbols))

    results = []
    lex = QuickLexer(LEXER)
    lex.lex(text)
    while lex.peek():
        id = lex.peek()[0]
        if id == lex.EXPR:
            results.append(handle_expr(lex.next()[1][2:-1]))
        elif id == lex.EXTENSION:
            results.append(handle_extension(lex.next()[1][2:-1]))
        elif id == lex.TEXT:
            results.append(lex.next()[1])
        elif id == lex.DOLLAR_DOLLAR_BRACE:
            results.append(lex.next()[1][1:])
    # return single element as is, i.e. typed
    if len(results) == 1:
        return results[0]
    # otherwise join elements to a string
    else:
        return ''.join(map(str, results))

LEXER = QuickLexer(DOLLAR_DOLLAR_BRACE=r"^\$\$+(\{|\()",  # multiple $ in a row, followed by { or (
                   EXPR=r"^\$\{[^\}]*\}",        # stuff starting with ${
                   EXTENSION=r"^\$\([^\)]*\)",   # stuff starting with $(
                   TEXT=r"[^$]+|\$[^{($]+|\$$")  # any text w/o $  or  $ following any chars except {($  or  single $

class QuickLexer(object):
    def __init__(self, *args, **kwargs):
        if args:
            # copy attributes + variables from other instance
            other = args[0]
            self.__dict__.update(other.__dict__)
        else:
            self.res = []
            for k, v in kwargs.items():
                self.__setattr__(k, len(self.res))
                self.res.append(re.compile(v))
        self.str = ""
        self.top = None

    def lex(self, str):
        self.str = str
        self.top = None
        self.next()

    def peek(self):
        return self.top

    def next(self):
        result = self.top
        self.top = None
        if not self.str:  # empty string
            return result
        for i in range(len(self.res)):
            m = self.res[i].match(self.str)
            if m:
                self.top = (i, m.group(0))
                self.str = self.str[m.end():]
                return result
        raise XacroException('invalid expression: ' + self.str)        
```

- なお受け取ったパラメータをマクロ内で使用する時、例えば`${hoge + 1}`等の形式でxacroマクロ引数を使用する時には、`hoge`部分が置換される時に`eval()`によって値が解釈されます（`"1"`という文字列はintの`1`と解釈される）。そして最終的に`${hoge + 1}`の結果はint型の`2`となる（これを受け取る側では文字列として解釈し、str型の`"2"`となる）

