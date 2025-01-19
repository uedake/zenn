---
title: "透視投影変換"
emoji: "📘"
type: "tech"
topics:
  - "math"
  - "perspective transformation"
  - "projective transformation"
published: false
---

# 解説対象

本記事では、透視投影（perspective projection）変換について解説します。透視投影変換は単に射影変換（projective transformation）とも呼ばれることがありますが、単に射影だと正射影/直交射影（orthographic projection）と用語が混乱するので本記事では透視投影変換と呼びます。

透視投影変換は、3次元上の点をピンホールカメラモデルによって2次元上に投影した点を求める変換です。

## 定義

透視投影変換の中でも特にピンホールカメラの中心（ピンホール）が原点に位置し、光軸がz軸向きで、投影面がz=1の平面である射影変換を標準的な透視投影変換として考える。

3次元上の点$\bm{v}=\begin{bmatrix} x \\ y \\ z   \end{bmatrix}$(ただし$z>0$)をz=1の平面上の点$\begin{bmatrix} x' \\ y' \\ 1 \end{bmatrix}$に下記関数$\bm{f}_{pp}: \R^3 \rightarrow \R^3$で写す変換を（z=1平面への）透視投影変換と呼ぶ。

$$
\bm{f}_{pp}(\bm{v})
= \begin{bmatrix}
    x / z \\
    y / z \\
    1
  \end{bmatrix}
\tag{1}
$$ 

もしくは、z座標を無視し、2次元空間へ射影する関数$\bm{F}_{pp}: \R^3 \rightarrow \R^2$を透視投影変換と呼ぶ。

$$
\bm{F}_{pp}(\bm{v})
= \begin{bmatrix}
    x / z \\
    y / z
  \end{bmatrix}
\tag{1'}
$$ 

## 透視投影変換の光線平行化変換と正射影変換への分解

透視投影変換$\bm{f}_{pp}$は、下記２つの変換を順に行った変換として分解できる。

- 光線平行化変換$\bm{f}_{rp}: \R^3 \rightarrow \R^3$
- 正射影変換$\bm{f}_{op}: \R^3 \rightarrow \R^3$

光線平行化変換とは、原点を通る直線（ピンホールカメラの中心を通る光線）群を平行な直線群に移す変換であり、（特にz軸に平行で距離を保存する）光線平行化変換$\bm{f}_{rp}$は下記で定義される。

$$
\bm{f}_{rp}(\bm{v})
= \begin{bmatrix}
    x / z \\
    y / z \\
    |\bm{v}|
  \end{bmatrix}
\tag{2}
$$ 

また、（z軸方向への）正射影変換$\bm{f}_{op}$は、下記で定義される。

$$
\bm{f}_{op}(\bm{v})
= P_3 \bm{v}
\tag{3}
$$ 
$$
P_3 = 
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    0 & 0 & 0
\end{bmatrix}
$$ 

この$\bm{f}_{rp}$と$\bm{f}_{op}$を使えば透視投影変換$\bm{f}_{pp}$は下記で求まる。

$$
\begin{split}
\bm{f}_{pp}(\bm{v})
& = \bm{f}_{op}(\bm{f}_{rp}(\bm{v}))
+ \begin{bmatrix} 
    0 \\
    0 \\
    1
  \end{bmatrix} \\
& = P_3\bm{f}_{rp}(\bm{v})
+ \begin{bmatrix} 
    0 \\
    0 \\
    1
  \end{bmatrix}
\end{split}
\tag{4}
$$ 

つまり、透視投影変換は、光線平行化変換した後に正射影変換することと（z軸方向への平行移動分を除き）等しい。

透視投影変換$\bm{f}_{pp}$は、ヤコビ行列が正則でなく扱いづらいが、光線平行化変換$\bm{f}_{rp}$はヤコビ行列が正則で扱いやすいという特徴がある。

なお、透視投影変換を2次元への射影$\bm{F}_{pp}$として記述する場合は、正射影変換を2次元への射影$\bm{F}_{op}$として下記で定義する。

$$
\bm{F}_{op}(\bm{v})
= P_2 \bm{v}
\tag{3'}
$$ 
$$
P_2 = 
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0
\end{bmatrix}
$$ 

この$\bm{f}_{rp}$と$\bm{F}_{op}$を使えば透視投影変換$\bm{F}_{pp}$は下記で求まる。

$$
\begin{split}
\bm{F}_{pp}(\bm{v})
& = \bm{F}_{op}(\bm{f}_{rp}(\bm{v})) \\
& = P_2\bm{f}_{rp}(\bm{v})
\end{split}
\tag{4'}
$$ 


## ヤコビ行列を用いた近似

1. 光線平行化変換を（正則な）アフィン変換で近似できること
2. 透視投影変換を（正則でない）アフィン変換で近似できること

を順に示す。

※アフィン変換とは線形変換と平行移動の合成で実現できる変換

### 光線平行化変換をアフィン変換で近似

光線平行化変換$\bm{f}_{rp}$のヤコビ行列を$J_{\bm{v}}  \in M_{3,3}$と書くと

$$
J_{\bm{v}}
= \frac{\partial \bm{f}_{rp}}{\partial \bm{v}}
= \begin{bmatrix}
    1/z & 0 & -x/z^2 \\
    0 & 1/z & -y/z^2 \\
    x/|\bm{v}| & y/|\bm{v}| & z/|\bm{v}|
  \end{bmatrix}
\tag{5}
$$

ヤコビ行列を用いると、ある点$\bm{v_0} = \begin{bmatrix} x_0 \\ y_0 \\ z_0 \end{bmatrix}$(ただし$z_0\neq0$)周りの局所領域$\bm{v}=\bm{v_0}+\delta \bm{v}$では、

$$
\begin{split}
\bm{f}_{rp}(\bm{v})
& = \bm{f}_{rp}(\bm{v_0}) + J_{\bm{v_0}}(\bm{v}-\bm{v_0}) \\
& = \begin{bmatrix}
    x_0 / z_0 \\ y_0 / z_0 \\ |\bm{v_0}|
  \end{bmatrix}
  +
  \begin{bmatrix}
    1/z_0 & 0 & -x_0/{z_0}^2 \\
    0 & 1/z_0 & -y_0/{z_0}^2 \\
    x_0/|\bm{v_0}| & y_0/|\bm{v_0}| & z_0/|\bm{v_0}|
  \end{bmatrix}
  (\bm{v}-\bm{v_0}) \\
& = \begin{bmatrix}
    x_0 / z_0 \\ y_0 / z_0 \\ 0
  \end{bmatrix}
  +
  \begin{bmatrix}
    1/z_0 & 0 & -x_0/{z_0}^2 \\
    0 & 1/z_0 & -y_0/{z_0}^2 \\
    x_0/|\bm{v_0}| & y_0/|\bm{v_0}| & z_0/|\bm{v_0}|
  \end{bmatrix}
  \bm{v}
\end{split}
\tag{6}
$$ 

と近似できる。

これは、ある点$\bm{v_0}$周りの局所領域$\bm{v}=\bm{v_0}+\delta \bm{v}$では、光線平行化変換を正則なアフィン変換で近似できることを意味する。

正則であることは、

$$
  |J_{\bm{v}}| = \begin{vmatrix}
    1/z & 0 & -x/{z}^2 \\
    0 & 1/z & -y/{z}^2 \\
    x/|\bm{v}| & y/|\bm{v}| & z/|\bm{v}|
  \end{vmatrix}
  = \frac{x^2+y^2+z^2}{z^3|\bm{v}|}
  = \frac{|\bm{v}|}{z^3}
  \neq 0
\tag{7}
$$ 

よりわかる

なお、$J_{\bm{v}}$の逆行列は下記である

$$
J_{\bm{v}}^{-1}
= \begin{bmatrix}
    z(1-x^2/|\bm{v}|^2) & -xyz/|\bm{v}|^2 & x/|\bm{v}| \\
    -xyz/|\bm{v}|^2 & z(1-y^2/|\bm{v}|^2) & y/|\bm{v}| \\
    -xz^2/|\bm{v}|^2 & -yz^2/|\bm{v}|^2 & z/|\bm{v}|
  \end{bmatrix}
$$


### 透視投影変換をアフィン変換で近似

(6)式の近似を(4)式に代入すると

$$
\begin{split}
\bm{f}_{pp}(\bm{v})
& = P_3\bm{f}_{rp}(\bm{v})
+ \begin{bmatrix} 
    0 \\
    0 \\
    1
  \end{bmatrix} \\
& = P_3
  \begin{bmatrix}
    1/z_0 & 0 & -x_0/{z_0}^2 \\
    0 & 1/z_0 & -y_0/{z_0}^2 \\
    x_0/|\bm{v_0}| & y_0/|\bm{v_0}| & z_0/|\bm{v_0}|
  \end{bmatrix}
  \bm{v}
+ P_3
  \begin{bmatrix}
    x_0 / z_0 \\ y_0 / z_0 \\ 0
  \end{bmatrix}
+ \begin{bmatrix} 
    0 \\
    0 \\
    1
  \end{bmatrix} \\
& =
  \begin{bmatrix}
    1/z_0 & 0 & -x_0/{z_0}^2 \\
    0 & 1/z_0 & -y_0/{z_0}^2 \\
    0 & 0 & 0
  \end{bmatrix}
  \bm{v}
+ \begin{bmatrix}
    x_0 / z_0 \\ y_0 / z_0 \\ 1
  \end{bmatrix}
\end{split}
\tag{8}
$$ 

これは、ある点$\bm{v_0}$周りの局所領域$\bm{v}=\bm{v_0}+\delta \bm{v}$では、透視投影変換を（正則でない）アフィン変換で近似できることを意味する。

## 楕円体に対する透視投影変換

3次元の楕円体を透視投影変換した場合の2次元平面上の図形は厳密には楕円にはならない。しかし、上記近似のもとでは、3次元空間上の楕円体は2次元空間の楕円に射影される。

そのことを下記の順で示す
1. 楕円体を光線平行化変換すると（近似のもとで）楕円体となること
2. 楕円体を正射影変換すると楕円となること

上記２つを示せれば、楕円体を透視投影変換すると（近似のもとで）楕円となることを示したことになる。

### 楕円体に対する光線平行化変換

中心を$\bm{v_0}$とする3次元空間上の任意の楕円体の表面を構成する点$\bm{v_{cylinder}} \in \R^3$は、楕円体の形を決める（正則な）対称行列$M \in M_{3,3}$を用いて下記式であらわせる。

$$
(\bm{v_{cylinder}}-\bm{v_0})^TM(\bm{v_{cylinder}}-\bm{v_0})
= 1
\tag{9}
$$

3次元の楕円体を光線平行化変換$\bm{f}_{rp}$で変換した点$\bm{v_{cylinder}}'=\bm{f}_{rp}(\bm{v_{cylinder}})$が(6)式の近似のもとで成す形は
- 中心$\bm{v_0}'=\bm{f}_{rp}(\bm{v_0})$
- 形を決める対称行列$M'={{J_{\bm{v_0}}}^{-1}}^{T}M{{J_{\bm{v_0}}}}^{-1}$

の楕円体となるが、そのことを確認してみる。

#### 証明

下記式(10)が成り立つことを確認すればよい

$$
(\bm{f}_{rp}(\bm{v_{cylinder})}-\bm{f}_{rp}(\bm{v_0}))^T{{J_{\bm{v_0}}}^{-1}}^{T}M{{J_{\bm{v_0}}}}^{-1}(\bm{f}_{rp}(\bm{v_{cylinder})}-\bm{f}_{rp}(\bm{v_0})) = 1
\tag{10}
$$

式(5)より$\bm{v_0}$の近傍に限れば

$$
\bm{f}_{rp}(\bm{v_{cylinder}}) - \bm{f}_{rp}(\bm{v_0})
= J_{\bm{v_0}}(\bm{v_{cylinder}}-\bm{v_0})
\tag{11}
$$

が成り立つ。(10)式の左辺を(11)式と(9)式を用いて変形すると、

$$
\begin{split}
& (\bm{f}_{rp}(\bm{v_{cylinder})}-\bm{f}_{rp}(\bm{v_0}))^T{{J_{\bm{v_0}}}^{-1}}^{T}M{{J_{\bm{v_0}}}}^{-1}(\bm{f}_{rp}(\bm{v_{cylinder})}-\bm{f}_{rp}(\bm{v_0})) \\
& = (\bm{v_{cylinder}}-\bm{v_0})^T{J_{\bm{v_0}}}^{T}{{J_{\bm{v_0}}}^{-1}}^{T}M{J_{\bm{v_0}}}^{-1}{J_{\bm{v_0}}}(\bm{v_{cylinder}}-\bm{v_0}) \\
& = (\bm{v_{cylinder}}-\bm{v_0})^TM(\bm{v_{cylinder}}-\bm{v_0}) \\
& = 1 
\end{split}
\tag{12}
$$

となり確かに(10)式が成り立っている。

### 楕円体に対する正射影変換

中心が$\bm{v}_0'$で形を決める対称行列が$M'$の楕円体（３次元図形）の表面を構成する点$\bm{v}'$

$$
(\bm{v}'-\bm{v}_0')^TM'(\bm{v}'-\bm{v}_0') = 1
\tag{13}
$$


をz方向の正射影$f_o$で変換したz=0平面上の点$\bm{v}''$

$$
\bm{v}'' = \bm{f}_{op}(\bm{v}') = P_3 \bm{v}'
\tag{14}
$$

の包絡線上の点$\bm{v}''_{envelope}$が楕円となっているかを確認する。これが確認できれば式(7)で決まる楕円体を式(3)で変換した図形（すなわち任意の楕円体を透視投影変換した図形）が楕円であることを示せる。

$\bm{v}''_{envelope}$がxy平面上の楕円であることを示すには、あるランクが２である行列$M'' \in M_{3,3}$（3行目の要素が全て0かつ3列目の要素が全て0とする）とベクトル$\bm{v}_0''=P_3\bm{v}_0'$を用いて、下記式が成り立つことを示せばよい

$$
(\bm{v}''_{envelope}-\bm{v}_0'')^TM''(\bm{v}''_{envelope}-\bm{v}_0'')
= 1
\tag{15}
$$

なお$M''$は、3行目の要素が全て0かつ3列目の要素が全て0の行列であるので下記が成り立たなければいけない。

$$
M'' 
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
= 0
\tag{16}
$$
$$
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'' 
= 0
\tag{17}
$$
$$
M'' 
P_3
= M''
\tag{18}
$$
$$
P_3
M'' 
= M''
\tag{19}
$$

また、楕円体（３次元図形）の表面を構成する点$\bm{v}'$の法線ベクトルは$M'(\bm{v}'-\bm{v}_0')$で与えられる。このことは

$$
\bm{g}(\bm{v}')=(\bm{v}'-\bm{v}_0')^TM'(\bm{v}'-\bm{v}_0')
\tag{20}
$$

の全微分を0とおいた下記式から直ちにわかる

$$
d\bm{g}(\bm{v}')
=\frac{\partial \bm{g}(\bm{v}')}{\partial \bm{v}'} \cdot d\bm{v}'
=2M'(\bm{v}'-\bm{v}_0') \cdot d\bm{v}'
=0
\tag{21}
$$

正射影されると包絡線上の点$\bm{v}''_{envelope}$になるような$\bm{v}'=\bm{v}'_{envelope}$は、法線ベクトルのz成分が0である点の集合であるので下記を満たす。

$$
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}

M'(\bm{v}'_{envelope}-\bm{v}_0')
=0
\tag{22}
$$

#### 証明
以下(15)式が成り立つことを示していく。まず$M''$をある行列$N \in M_{3,3}$を用いて

$$
M'' = M' - N
\tag{23}
$$

と置いてみる。

(23)式に(16)式・(17)式を当てはめると下記が成り立つ必要がある。

$$
M'
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
=
N
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
\tag{24}
$$

$$
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'
=
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
N
\tag{25}
$$

ここで(24)式・(25)式が成り立つ$N$は下記とかけることがわかる（実際に左右から$\begin{bmatrix}0 & 0 & 1\end{bmatrix}$もしくは$\begin{bmatrix}0 & 0 & 1\end{bmatrix}^T$をかけてみることで容易に確かめられる）

$$
N
=
\frac{
M'
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'
}{
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
}
\tag{26}
$$

さらに、この$N$は(22)式より下記を満たすことがわかる。

$$
N(\bm{v}'_{envelope}-\bm{v}_0')=0
\tag{27}
$$


ここで(15)式の左辺を(14)(18)(19)(23)(27)式を使用して置き換え,最後に(13)式の$\bm{v}'$に$\bm{v}'_{envelope}$を代入した式を代入してみれば

$$
\begin{split}
(\bm{p}_{envelope}-\bm{v}_0'')^TM''(\bm{p}_{envelope}-\bm{v}_0'')
= & (\bm{v}'_{envelope}-\bm{v}_0')^TP_3M''P_3(\bm{v}'_{envelope}-\bm{v}_0') \\
= & (\bm{v}'_{envelope}-\bm{v}_0')^TM''(\bm{v}'_{envelope}-\bm{v}_0') \\
= & (\bm{v}'_{envelope}-\bm{v}_0')^T(M'-N)(\bm{v}'_{envelope}-\bm{v}_0') \\
= & (\bm{v}'_{envelope}-\bm{v}_0')^TM'(\bm{v}'_{envelope}-\bm{v}_0') \\
= & 1
\end{split}
\tag{27}
$$

となり確かに(15)式が満たされることが示せた。

すなわち、中心が$\bm{v}_0'$で形を決める対称行列が$M'$の楕円体は、(z方向への)正射影によって下記の楕円となることがわかった。

- 中心$\bm{v}_0''=P_3\bm{v}_0'$
- 形を決める対称行列$M''
= M' -
\frac{
M'
\begin{bmatrix} 
  0 & 0 & 0\\
  0 & 0 & 0 \\
  0 & 0 & 1
\end{bmatrix}
M'
}{
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
}
$

### 楕円体に対する透視投影変換

透視投影変換は、光線平行化変換と正射影変換の合成であるので、順に適用すればよい。

下記の任意の楕円体は
- 中心$\bm{v_0}$
- 形を決める対称行列$M$

まず光線平行化変換$\bm{f}_{rp}$により下記の楕円体に（近似的に）変換される
- 中心$\bm{v_0}'=\bm{f}_{rp}(\bm{v_0})$
- 形を決める対称行列$M'={{J_{\bm{v_0}}}^{-1}}^{T}M{{J_{\bm{v_0}}}}^{-1}$

さらに正射影変換$\bm{f}_{op}$により下記の楕円に変換される
- 中心$\bm{v}_0''=P_3\bm{v}_0'=P_3\bm{f}_{rp}(\bm{v_0})$
- 形を決める対称行列$M''
= M' -
\frac{
M'
\begin{bmatrix} 
  0 & 0 & 0\\
  0 & 0 & 0 \\
  0 & 0 & 1
\end{bmatrix}
M'
}{
\begin{bmatrix} 
  0 & 0 & 1
\end{bmatrix}
M'
\begin{bmatrix} 
  0 \\
  0 \\
  1
\end{bmatrix}
}
= {{J_{\bm{v_0}}}^{-1}}^{T}
(M - \frac{
M\bm{v_0}\bm{v_0}^TM
}{
\bm{v_0}^{T}M\bm{v_0}
})
{{J_{\bm{v_0}}}}^{-1}
$

## 3次元ガウス関数で決まる密度値の透視投影変換

3次元空間上に、楕円体の不透明度を持つ粒子（ガウス粒子）を考える。この粒子は中心を観測した場合には不透明度が1（完全に不透明）であり、中心からずれた位置を観測した場合、中心から離れれるほどに不透明度が下がり（透明度があがり）後ろにあるモノからの光が見えるとする。この不透明度を下記の通りモデル化して定義する。

### ガウス粒子の定義

ガウス粒子は3次元空間上に広がっておりその位置に応じた密度$D_{3d}: \R^3 \rightarrow \R$を有し、その密度は粒子の中心$v_0 \in R^3$で最大となり、中心から$v \in R^3$離れた位置では3次元ガウス関数（分散共分散行列を$Σ \in M_{3,3}$とする）に従って減衰するものとする。すなわち、

$$
D_{3d}(\bm{v})
= G_3(\bm{v};\bm{v}_0,Σ)
= \frac{1}{\sqrt{(2\pi)^3|Σ|}}
\Large
𝑒^{-\frac{1}{2}(\bm{v}-\bm{v}_0)^TΣ^{-1}(\bm{v}-\bm{v}_0)}
\tag{28}
$$

この密度とは背景にあるモノからの光を通しにくさを表し、不透明度を計算するために定義する概念である。

粒子を原点に穴があるピンホールカメラで観測した時の不透明度を考える準備の為、ピクセル座標値の斉次座標$\hat{\bm{p}}=(p_x,p_y,1)$とカメラ座標$\bm{v_c} \in \R^3$の対応付けをカメラ内部パラメーラと呼ぶ行列$K \in M_{3,3}$を用いて下記で定義する。なお、カメラ座標とは画素を通る光が$z=1$の平面と交わる点を表す座標である。

$$
\hat{\bm{p}} = K \bm{v_c} \\
\tag{29}
$$
$$
K  = \begin{bmatrix}
    f_x & 0 & p_{x0}\\
    0 & f_y & p_{y0}\\
    0 & 0 & 1
    \end{bmatrix}
$$
$$
K^{-1}  = \begin{bmatrix}
    1/f_x & 0 & -p_{x0}/f_x\\
    0 & 1/f_y & -p_{y0}/f_y\\
    0 & 0 & 1
    \end{bmatrix}
$$

上記における$f_x,f_y$はピクセル間距離（画素ピッチ）単位での焦点距離であり、焦点距離$f$、x方向画素ピッチ$PitchX$、y方向画素ピッチ$PitchY$から下記で定まる値である。

$$
f_x = f/PitchX \\
f_y = f/PitchY
$$

また、原点及び空間上の点$\bm{v}\in \R^3$を通る光線を$Ray(\bm{v})$と呼ぶこととする

以上で準備が整ったので、カメラ座標$\bm{v_c}$で観測される粒子の不透明度$α(\bm{v_c})$を下記で定義する。

$$
α(\bm{v}_c) = C\int_{Ray(\bm{v}_c)} D_{3d}(\bm{v}) ds
\tag{30}
$$

ここで$ds$は線素であり$ds=|d\bm{v}|$である。$C$は、不透明度の値の最大値を1にするための定数であり、下記で定義する。

$$
C = \frac{1}{\int_{Ray(\bm{v}_0)} D_{3d}(\bm{v}) ds}
\tag{31}
$$

上記(31)式の積分を光線平行化変換$\bm{f}_{rp}$による変換したパラメータ$\bm{v}'=\bm{f}_{rp}(\bm{v})$で書き直すと

$$
\begin{split}
α(\bm{v}_c)
= & C\int_{0}^{\infty}{D_{3d}(f_{rp}^{-1}(\bm{v}'))}|\frac{\partial v}{\partial z'}|dz' \\
= & C\int_{0}^{\infty}{D_{3d}(f_{rp}^{-1}(\bm{v}'))}dz'
\end{split}
\tag{31}
$$

とかける。(31)式は積分範囲$(0,\infty)$を$(-\infty,\infty)$で近似し、さらに近似$J_{\bm{v_0}}^{-1}(\bm{v}'- \bm{v}_0') = \bm{v}-\bm{v_0}$
を用い$dz'$の積分を計算すると下記に書き換えられる。

$$
\begin{split}
α(\bm{v}_c)
= & \frac{C}{\sqrt{(2\pi)^3|Σ|}}
    \int_{-\infty}^{\infty}
    \Large
    𝑒^{-\frac{1}{2}(\bm{v}'-\bm{v}'_0)^T(J_{\bm{v_0}}ΣJ_{\bm{v}_0}^T)^{-1}(\bm{v}'-\bm{v}'_0)}
    \normalsize
    dz' \\
= & C\sqrt{\frac{|J_{\bm{v}_0}ΣJ_{\bm{v_0}}^T|}{|Σ|}}
  \int_{-\infty}^{\infty} G_3(\bm{v}';\bm{v}'_0,J_{\bm{v}_0}ΣJ_{\bm{v_0}}^T)
    dz' \\
= &
  C||J_{\bm{v}_0}||
  G_2(K \bm{v_c};P_2\bm{v}'_0,P_2J_{\bm{v_0}}ΣJ_{\bm{v_0}}^TP_2^T) \\
\end{split}
$$
なお、定数Cは
$$
C = \frac{1}{
  ||J_{\bm{v}_0}||
  G_2(K \bm{v_0};P_2\bm{v}'_0,P_2J_{\bm{v_0}}ΣJ_{\bm{v_0}}^TP_2^T)}
  = \frac{1}{||J_{\bm{v}_0}||}\\
$$
であるから、最終的に下記となる
$$
α(\bm{v}_c)
= G_2(K \bm{v_c};P_2\bm{v}'_0,P_2J_{\bm{v_0}}ΣJ_{\bm{v_0}}^TP_2^T) \\
$$
