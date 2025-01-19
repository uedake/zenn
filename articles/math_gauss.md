---
title: "正規分布"
emoji: "📘"
type: "tech"
topics:
  - "math"
published: false
published_at: "2023-09-03 22:13"
---

# 解説対象

本記事では、多変量正規分布の周辺分布について解説します。多変量正規分布の周辺分布は、多変量正規分布が定義された空間上で正射影したときに現れる分布です。

## 定義

中心が$\bm{μ} \in \R^n$で分散共分散行列が$Σ \in M_{n,n}$である$n$次元の多変量正規分布$G_n:\R^n \rightarrow \R$を、n次元ベクトル$\bm{v} \in \R^n$に対して下記で定義する。

$$
G_n(\bm{v};\bm{μ},Σ) =
\frac{1}{\sqrt{(2\pi)^n|Σ|}}
\Large
e^{-\frac{(\bm{v}-μ)^{T}Σ^{-1}(\bm{v}-μ)}{2}}
\tag{1}
$$

## 別定義

分散共分散行列が$Σ$の逆行列を$Λ=Σ^{-1}$と書き精度行列と呼ぶ。精度行列で正規分布を記載した場合は下記となる。

$$
G_n(\bm{v};\bm{μ},Λ) =
\sqrt{\frac{|Λ|}{(2\pi)^n}}
\Large
e^{-\frac{(\bm{v}-μ)^{T}Λ(\bm{v}-μ)}{2}}
\tag{2}
$$

## 部分行列

後述の正規分布の周辺分布の計算（積分）の為に$Σ$と$Λ$の部分行列を定義しておく。

分散共分散行列$Σ$を最終列・最終行部分とそれ以外に分ける。$Σ_{n-1} \in M_{n-1,n-1}$、$\bm{s}_{n-1} \in \R^{n-1}$、$σ_{n}^{2} \in \R$で記述すると下記となる

$$
Σ= \begin{bmatrix}
   Σ_{n-1} & \bm{s}_{n-1} \\
   \bm{s}_{n-1}^{T} & σ_{n}^{2}
   \end{bmatrix}
\tag{3}
$$

同様に精度行列$Λ$を最終列・最終行部分とそれ以外に分ける。$Λ_{n-1} \in M_{n-1,n-1}$、$\bm{a}_{n-1} \in \R^{n-1}$、$α_{n,n} \in \R$で記述すると下記となる

$$
Λ= \begin{bmatrix}
   Λ_{n-1} & \bm{a}_{n-1} \\
   \bm{a}_{n-1}^{T} & α_{n,n}
   \end{bmatrix}
\tag{4}
$$

上記の定義においては$ΣΛ=E_n$(※$E_n$はランク$n$の単位行列)の関係より下記が成り立つ

$$
Σ_{n-1}Λ_{n-1}+\bm{s}_{n-1}\bm{a}_{n-1}^{T}=E_{n-1}
$$
$$
Σ_{n-1}\bm{a}_{n-1}+\bm{s}_{n-1}α_{n,n}=\bm{0}
$$
$$
Λ_{n-1}\bm{s}_{n-1}+\bm{a}_{n-1}σ^2_{n}=\bm{0}
$$
$$
\bm{s}_{n-1}^{T}\bm{a}_{n-1}+σ^2_{n}α_{n,n}=1
$$

また上記の式を整理すると下記が求まる。

$$
Σ_{n-1}^{-1}=Λ_{n-1}-\frac{\bm{a}_{n-1}\bm{a}_{n-1}^{T}}{α_{n,n}}
\tag{5}
$$

さらに下記の逆行列と余因子行列の関係より、$Σ$の余因子行列を$\hat{Σ}$とすると

$$
Λ　= \frac{1}{|Σ|}\hat{Σ}
$$

上式における最終列・最終行の要素をみると下記が成り立つ

$$
α_{n,n} = \frac{|Σ_{n-1}|}{|Σ|}
\tag{6}
$$

## 多変量正規分布の周辺分布

$n$次元(n>=2)の多変量正規分布$G_n$をある１つの次元で積分した周辺分布は、$n-1$次元の多変量正規分布$G_{n-1}$となる。特に最終次元で積分した場合を考え最終次元を分けて

$$
\bm{v}
= \begin{bmatrix}
  \bm{v}_{n-1} \\
  v_n
  \end{bmatrix}
$$

$$
\bm{μ}
= \begin{bmatrix}
  \bm{μ}_{n-1} \\
  μ_n
  \end{bmatrix}
$$

と記載すれば、下記が成り立つ。

$$
G_{n-1}(\bm{v}_{n-1};\bm{μ}_{n-1},Σ_{n-1}) =
\int_{-\infty}^{\infty} G_{n}(\bm{v};\bm{μ},Σ)dv_n
\tag{7}
$$

### 証明

定義(1)式より

$$
\int_{-\infty}^{\infty} G_{n}(\bm{v})dv_n
= \frac{1}{\sqrt{(2\pi)^n|Σ|}}
 \int_{-\infty}^{\infty}
\Large
e^{-\frac{(\bm{v}-μ)^{T}Σ^{-1}(\bm{v}-μ)}{2}}
\normalsize
dv_n
$$

$\bm{v}=\bm{v}'+\bm{μ}$と置くと、

$$
\bm{v}'
= \begin{bmatrix}
  \bm{v}_{n-1}' \\
  v_n'
  \end{bmatrix}
= \begin{bmatrix}
  \bm{v}_{n-1} - \bm{μ}_{n-1}\\
  v_n - μ_n
  \end{bmatrix}
$$

$$
\int_{-\infty}^{\infty} G_{n}(\bm{v})dv_n
= \frac{1}{\sqrt{(2\pi)^n|Σ|}}
 \int_{-\infty}^{\infty}
\Large
e^{-\frac{\bm{v}'^{T}Σ^{-1}\bm{v}'}{2}}
\normalsize
dv'_n
\tag{8}
$$

ここで指数関数の肩の部分の２次形式部分を抜き出して$F$と書くと、

$$
F = \bm{v}'^{T}Σ^{-1}\bm{v}'= \bm{v}'^{T}Λ\bm{v}'
$$

上式を部分行列を用いて積和に分解し、$v_n'$について平方完成し、(5)式を適用すると

$$
\begin{split}
F 
& = \bm{v}_{n-1}'^{T}Λ_{n-1}\bm{v}_{n-1}'
  + 2\bm{v}_{n-1}'^{T}\bm{a}_{n-1}v_n'
  + α_{n,n}v_n'^2 \\
& = α_{n,n}(v_n'+\frac{\bm{v}_{n-1}'^{T}\bm{a}_{n-1}}{α_{n,n}})^2
  + \bm{v}_{n-1}'^{T}Λ_{n-1}\bm{v}_{n-1}'
  - \frac{(\bm{v}_{n-1}'^{T}\bm{a}_{n-1})^2}{α_{n,n}} \\
& = α_{n,n}(v_n'+\frac{\bm{v}_{n-1}'^{T}\bm{a}_{n-1}}{α_{n,n}})^2
  + \bm{v}_{n-1}'^{T}(Λ_{n-1}-\frac{\bm{a}_{n-1}\bm{a}_{n-1}^{T}}{α_{n,n}})\bm{v}_{n-1}' \\
& = α_{n,n}(v_n'+\frac{\bm{v}_{n-1}'^{T}\bm{a}_{n-1}}{α_{n,n}})^2
  + \bm{v}_{n-1}'^{T}Σ_{n-1}^{-1}\bm{v}_{n-1}'
\end{split} 
$$

ここで公式

$$
\int_{-\infty}^{\infty} e^{-ax^2}dx=\sqrt{\frac{\pi}{a}}
$$

を用いると、Fの第１項の積分は

$$
\int_{-\infty}^{\infty}
\Large
e^{-\frac{α_{n,n}(v_n'+\frac{\bm{v}_{n-1}'^{T}\bm{a}_{n-1}}{α_{n,n}})^2}{2}}
\normalsize
dv_n'
= \sqrt{\frac{2\pi}{α_{n,n}}}
$$

であるから、(8)式は積分部分が計算でき、さらに(6)式を適用し、最後に$\bm{v}_{n-1}'$を$\bm{v}_{n-1}$に戻すと

$$
\begin{split}
\int_{-\infty}^{\infty} G_{n}(\bm{v})dv_n
= & \frac{1}{\sqrt{(2\pi)^n|Σ|}}
\sqrt{\frac{2\pi}{α_{n,n}}}
\Large
e^{-\frac{\bm{v}_{n-1}'^{T}Σ_{n-1}^{-1}\bm{v}_{n-1}'}{2}} \\
= & \frac{1}{\sqrt{(2\pi)^{n-1}|Σ_{n-1}|}}
\Large
e^{-\frac{\bm{v}_{n-1}'^{T}Σ_{n-1}^{-1}\bm{v}_{n-1}'}{2}} \\
= & \frac{1}{\sqrt{(2\pi)^{n-1}|Σ_{n-1}|}}
\Large
e^{-\frac{(\bm{v}_{n-1}-\bm{μ}_{n-1})^{T}Σ_{n-1}^{-1}(\bm{v}_{n-1}-\bm{μ}_{n-1})}{2}} \\
= & G_{n-1}(\bm{v}_{n-1};\bm{μ}_{n-1},Σ_{n-1})
\end{split}
$$

となり(7)式が証明できた。