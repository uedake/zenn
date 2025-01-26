---
title: "MuJoCoの環境構築（GPU版／windows）方法"
emoji: "📘"
type: "tech"
topics:
  - "MuJoCo"
  - "MJX"
  - "Jax"
published: false
---

# 解説対象

本記事では、MuJoCo及びMuJoCo XLA(MJX)の環境構築方法として、nvidia-GPU版のwindows上（WSL2上）での環境構築方法を解説します。

MuJoCo XLAは、物理シミュレーターMuJoCoをJax上で動くように再実装したバージョンです。
Jax上で動くのでGPUが得意なケースでは高速なシミュレーションができたり、また微分可能な物理シミュレータとして動作するので、深層学習においてシミュレーション結果を望ましい結果と差分をとって学習することも可能になります。

## インストール環境
下記の環境を前提とする。

- WSL2 の Ubuntu22.04イメージ上に入れたdockerを利用
- NvidiaのGPU

## インストールの事前準備

WSL2環境、docker環境、nvidiaドライバ環境が準備できているか確認します。
ノーマルMuJoCoはCPU演算ですが、MuJoCo XLAはGPU演算が可能です。
GPUはJaxを通して利用されるので、下記参照先の通りJaxのGPU版のインストールをしておきます。

[Jaxの環境構築（GPU版／windows）方法](install_jax_gpu.md)

次にOpenGLが使えるか確認します(MuJoCoのビューアーを利用したい場合は必須)。

```sh
glxinfo | grep "OpenGL renderer"
```

下記のようにマシンのGPUの情報が帰ってくればOK。
```
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4070 Ti SUPER)
```

※glxinfoコマンドが見つからない場合は、まず下記コマンドでインストールしてから上記の確認をします
```sh
sudo apt install mesa-utils
```

## インストール（dockerコンテナの作成）

今回は、WSL2のコンテナ（Ubuntu）上へのインストールでなく、「WSL2のコンテナ（Ubuntu）上のdockerイメージ上」にインストールします。dockerイメージ化することで異なるPC上での環境の再現や取り回しが楽になる為。

下記のようなDockerfileを作成します。

```dockerfile
from nvcr.io/nvidia/jax:24.10-py3

RUN apt-get update

# mujocoのGUIを表示するため
RUN apt-get install -y mesa-utils 

# mujocoのGUIにモデルファイルをD&Dする為
RUN apt-get install -y nautilus

RUN pip install mujoco mujoco-mjx
```

まずPowerShell等からwsl上に入ります。
```pwsh
PS C:\> wsl 
```

wslの中で下記を実行します。
```bash
cd
mkdir mujoco
cd mujoco
code Dockerfile    ※注：好きなエディタを起動してDockerfileを作成
docker build -t mujoco .
```
これでDockerイメージが作成されます。

### 解説
`RUN pip install mujoco`の部分でMuJoCoが入ります。

- Web上で検索すると、ビルド済みのバイナリをDLしてくる手順をよく見かけますが、pipでインストールしたときにビルド済みのバイナリも自動的に入るので別途DLする必要はありません。
- [公式](https://mujoco.readthedocs.io/en/latest/python.html)でもそのように案内されている


- ビルド済みバイナリとの差分
  - GUI(ビューアー)のコマンドラインからの起動方法
    - ビルド済みバイナリではビューアの実行ファイル`simulate`があるのでそれを起動するだけ
    - pipインストールした場合は、`python -m mujoco.viewer`で起動する必要がある。
  - サンプルのモデル
    - ビルド済みバイナリでは、サンプルモデルが含まれる。
    - pipインストールした場合は、サンプルモデルが含まれない。

### ビルド済みバイナリを別途いれたい場合

サンプルモデルが欲しい場合はビルド済みバイナリを別途DLしてくる。

- [ビルド済みバイナリ](https://github.com/google-deepmind/mujoco/releases)

例えば下記コマンドを実行してダウンロードする。
- 使いたいバージョンを指定すること
- ファイルのDL場所はコンテナからアクセスできるところならどこでもよい。

```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xvzf mujoco-3.2.7-linux-x86_64.tar.gz
rm mujoco-3.2.7-linux-x86_64.tar.gz
```

なお、ビルド済みバイナリ付属のサンプルモデル以外に、ロボットのサンプルモデルは下記が充実している。
- [ロボットのモデル](https://github.com/google-deepmind/mujoco_menagerie)

## MuJoCoの動作確認

まずPowerShell等から下記コマンドでMuJoCoのコンテナ内に入ります。
```pwsh
PS C:\> wsl docker run --rm -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all --shm-size=4gb mujoco bash
```

コンテナ内で下記を実行します。
```bash
python -m mujoco.viewer
```

これで、下記のようなGUIが表示されたらOK。

![Viewer](https://github.com/uedake/zenn/blob/main/articles/mujoco_img/viewer.jpg?raw=true)

さらに、GUI上でモデルを開く為に、ファイルマネージャー（nautilus）もコンテナ上で起動すると便利です。下記のようにコンテナ内でviewer及びファイルマネージャーをバックグランド起動するとよいです。
```bash
python -m mujoco.viewer &
nautilus &
```

windows上にviewerのwindowとファイルマネージャー（コンテナ内のファイルが見える）が両方開きます。ファイルマネージャー上からサンプルモデル（ビルド済みバイナリのmodelフォルダ以下にある）をviewerにD&Dすると開けます。

![FileManager](https://github.com/uedake/zenn/blob/main/articles/mujoco_img/viewer_with_filemanager.jpg?raw=true)

Linuxの画面間でのD＆DもスムーズにできるWSL2すごい！

### GUIが開かない場合
```
ERROR: could not create window
```
とエラーがでる場合は、OpenGLが有効になっていない可能性が高い。この場合は、

```sh
apt list --installed | grep mesa
```
をコンテナ内で実行してlibglx-mesa等がインストールされているか確認。
インストールされていない場合は、下記コマンドでインストールする。
（libglx-mesa等も一緒にインストールされる）
```sh
sudo apt install mesa-utils
```

### モデルを動かしてみる

[ロボットのモデル](https://github.com/google-deepmind/mujoco_menagerie)
をcloneしてきて、試しにロボットハンド（robotiq 2f85）のscene.xmlを開いてみると下記の表示になる。

![hand open](https://github.com/uedake/zenn/blob/main/articles/mujoco_img/hand_open.jpg?raw=true)

画面右下のcontrolの中にあるfingers_actuatorの数値をあげるとハンドが閉じる。うまくハンドを閉じれると上からパン食い競争のようにぶらさがっているキューブをつかめる。

![hand open](https://github.com/uedake/zenn/blob/main/articles/mujoco_img/hand_close.jpg?raw=true)


## MuJoCo XLAの動作確認

- 記載予定