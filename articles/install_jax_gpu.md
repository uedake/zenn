---
title: "Jaxの環境構築（GPU版／windows）方法"
emoji: "📘"
type: "tech"
topics:
  - "jax"
published: false
---

# 解説対象

本記事では、jaxの環境構築方法として、nvidia-GPU版のwindows上（WSL2上）での環境構築方法を解説します。

## サポートしている環境の確認

cpu版のインストールに比べるとGPU版は少し難しい様子。しかもwindowsへの対応は2025/1/19現在は公式サイト上で「experimental」となっている。うまく動くか試してみた。
- https://jax.readthedocs.io/en/latest/installation.html


![Supported platforms](https://github.com/uedake/zenn/blob/main/articles/jax_img/jax_supported.jpg?raw=true)

## インストール環境
下記の環境を前提とする。

- WSL2 の Ubuntu22.04イメージ上に入れたdockerを利用
- NvidiaのGPU

## インストールの事前確認

1. WSL2環境の確認

下記コマンドを実行。
``` pwsh
PS C:\> wsl -l -v
```

下記のように、Ubuntuがdefaultになっていることを確認する。
※今回はdocker-desktopを使用しない
```
  NAME                   STATE           VERSION
* Ubuntu                 Running         2
  docker-desktop         Running         2
  docker-desktop-data    Running         2
```

2. docker環境の確認

下記コマンドを実行してdockerコマンドが通ることを確認する。
``` pwsh
PS C:\> wsl docker -v 
```

下記のようにDocker versionが表示されたらOK。
```
Docker version 26.0.0, build 2ae903e
```

3. nvidiaドライバ環境の確認


```pwsh
PS C:\> wsl nvidia-smi
```

下記のようにマシンのGPU（例：NVIDIA GeForce RTX 4070）が表示されればOK。
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 550.54.10              Driver Version: 551.61         CUDA Version: 12.4     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 4070 ...    On  |   00000000:01:00.0 Off |                  N/A |
|  0%   36C    P8              8W /  285W |     646MiB /  16376MiB |      2%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
```

## インストール（dockerイメージの用意）

下記コマンドで、nvidiaが配布しているdockerイメージをpullしてくる。

```pwsh
PS C:\> docker pull nvcr.io/nvidia/jax:24.10-py3
```

下記コマンドでイメージがダウンロードできているか確認する。

```pwsh
PS C:\> wsl docker images
```

下記のようにイメージが表示されればOK。
```
REPOSITORY           TAG              IMAGE ID       CREATED        SIZE
nvcr.io/nvidia/jax   24.10-py3        7daba2a04243   3 months ago   12.5GB
```

## 動作確認

[nvidiaのページ](https://docs.nvidia.com/deeplearning/frameworks/jax-release-notes/running.html)に従って動作確認する。

下記コマンドを実行しコンテナを起動する
（`--shm-size=4g`のオプションは共有メモリとして4GBを割当てる指定。何GB割り当てるかは処理によるので各自適切な値を指定すること。）
```pwsh
PS C:\> wsl docker run --gpus all -it --rm --shm-size=4g nvcr.io/nvidia/jax:24.10-py3
```

下記が表示される場合あるが、無視。
```
WARNING: CUDA Minor Version Compatibility mode ENABLED.
```

プロンプトがでたら適当なエディタ（下記例はnano）を起動して性能を計測する為の下記スクリプトを保存する。

```bash
root@f22f03ea8071:/# nano eval.py
```

``` py : eval.py
from functools import partial

import numpy as np
import jax.numpy as jnp
import jax
import timeit

@partial(jax.jit, static_argnums=(0,), device=jax.devices("cpu")[0])
def jax_cpu_mod(size):
    x = jnp.arange(size, dtype=jnp.int32)
    mat = x[None, :] * x[:, None]
    return mat % 256

@partial(jax.jit, static_argnums=(0,), device=jax.devices("gpu")[0])
def jax_gpu_mod(size):
    x = jnp.arange(size, dtype=jnp.int32)
    mat = x[None, :] * x[:, None]
    return mat % 256

def numpy_mod(size):
    x = np.arange(size, dtype=np.int32)
    mat = x[None, :] * x[:, None]
    return mat % 256

def calc_time(cmd,loop,title):
    t = timeit.timeit(cmd, globals=globals(), number=loop)
    print(title, f"{t:.3f}")
    return t

for i in range(4):
    size = 10**(i+1)
    repeat = 10**(4-i)
    print("size =", size, "repeat =", repeat)
    calc_time('numpy_mod(size)',repeat,'-  no jax:')
    calc_time('jax_cpu_mod(size).block_until_ready()',repeat,'- jax cpu:')
    calc_time('jax_gpu_mod(size).block_until_ready()',repeat,'- jax gpu:')    
```

下記コマンドでスクリプトを実行する。
```bash
 python eval.py
```

下記のように結果が表示される。sizeが大きなときにgpuが速いことが確認できればOK。
```
size = 10 repeat = 10000
-  no jax: 0.020
- jax cpu: 0.172
- jax gpu: 0.932
size = 100 repeat = 1000
-  no jax: 0.017
- jax cpu: 0.083
- jax gpu: 0.091
size = 1000 repeat = 100
-  no jax: 0.194
- jax cpu: 0.031
- jax gpu: 0.036
size = 10000 repeat = 10
-  no jax: 1.769
- jax cpu: 0.154
- jax gpu: 0.039
```

参考

- [JAX入門～高速なNumPyとして使いこなすためのチュートリアル～](https://qiita.com/koshian2/items/44a871386576b4f80aff)