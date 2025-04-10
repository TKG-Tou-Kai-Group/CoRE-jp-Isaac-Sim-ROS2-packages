# CoREjp-Isaac-Sim-ROS2-packages
- サンプルロボット操縦デモ
  ![ros2_control_demo](figs/core_sim_multi_robot_test.gif)

このリポジトリは，NvidiaのIsaac SimでROS 2で作ったロボットを開発するためのパッケージをまとめたものです．
重要なパッケージは"isaac_ros2_utils"内に格納された"topic_based_ros2_control"と"isaac_ros2_scripts"です．
"topic_based_ros2_control" は hardware_interface クラスを提供し，ros2_control からのコマンドを "isaac_ros2_scripts" に送ります．
"isaac_ros2_scripts "にはIsaac Simを起動し制御するためのpythonスクリプトがあります．

このリポジトリでできること
- CoREjp2024の1部リーグ決勝トーナメント用ステージの生成
- オムニホイールロボットの生成
- ROS 2のteleop_twist_keyboadパッケージでの操縦
- ROSトピックとして、ロボットに取り付けられたカメラ映像を取得

## 必要なもの
1. Docker
1. Isaac SimのDockerイメージ (このリポジトリでは nvcr.io/nvidia/isaac-sim:4.1.0 を使ってテストしています)

## 使い方
1. Dockerをインストールして、Isaac SimのDockerイメージをプルする

   このURLを参考にイメージをプルしてください https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_container.html

1. このリポジトリをクローンする
   ```bash
   git clone https://github.com/TKG-Tou-Kai-Group/CoRE-jp-Isaac-Sim-ROS2-packages.git
   ```

1. サブモジュール（オムニホイールコントローラ）をセットアップする
   ```bash
   cd CoRE-jp-Isaac-Sim-ROS2-packages/docker
   git submodule update --init --recursive
   ```

1. Dockerイメージをビルドする
   ```bash
   ./build_docker_image.sh
   ```

1. Dockerコンテナを立ち上げる
   ```bash
   ./launch_docker.sh
   ```

1. ROS 2のソースコードをビルドする
   ```bash
   colcon build && source install/setup.sh
   ```

4. パッケージを起動する
   - シミュレータおよびステージの立ち上げ
   
   立ち上げたコンテナ内で以下のコマンドを実行してください

   立ち上がるまでに数分かかるので、気長に待ってください
   ```bash
   ros2 launch sample_robot_sim bring_up_core_stage.launch.py
   ```
   - ロボットの生成

   別のターミナルから下記のコマンドを実行してください
   ```bash
   docker exec -it isaac-sim /bin/bash
   ros2 launch sample_robot_sim sample_robot_1_spawn_for_core.launch.py
   ```
   ディスクが一通り生成し終わったら、シミュレータ左の三角ボタンでシミュレーションを開始してください
   次のロボットを生成するときも、一度シミュレータ左の三角ボタンを押してシミュレーションを開始して、一息おいて四角ボタンで停止させてください。
   一度シミュレーションを動かしておかないと、シミュレーションが落ちる場合があります。
   sample_robot_1_spawn_for_core.launch.pyの数字部分を変更することで、別のロボットを生成できます。番号は1から8まで用意しています。

   - 操作インターフェースの立ち上げ

   tools/robot_1_control.htmlをブラウザで開いてください。
   こちらもrobot_1からrobot_8まであり、各ロボットに対応しています。

   補足1：control.html内のlocalhostの記述を所望のIPアドレスに変更することで、同一LANの別のPCから操作することができます。

   補足2：このhtmlでは、表示用画像をサブスクライブし、ジョイコンの状態をパブリッシュしています。適宜トピック名を変更することで別の操作画面やロボットの操作に活用できます。

   - 操作方法
   
   PSコントローラだと、
   
   - 左スティック：ロボットの前後左右移動
   - 右スティック左右：ロボットの回転
   - ○ボタン：ディスク射出（単発）

## シミュレータを外部ネットワークの参加者と共有

本シミュレータはROS 2の通信をrosbridge_suiteによって特定のポートを用いてPub/Subすることでブラウザ経由でロボットの操作ができるというものです。そのため、指定したポートを外部ネットワークの参加者と共有することにより、同じシミュレーション環境を共有することができます。

かんたんにポートを共有するツールとして、[Secure Share Net](https://gsht.io/)というツールがあります。こちらのツールを利用することで、外部ネットワークの参加者と気軽にシミュレーションすることが可能です。以下に、Secure Share Netを使用する手順を示します。（本シミュレータは愚直に画像を伝送しているので通信帯域を多く使います。支援特典により帯域を多く割り当ててもらえるようになるので、活用してみてください。）

1. 現在シミュレータを起動している同じPCでSecure Share Netを起動します。
1. Secure Share Netの管理画面から、「ローカルポート番号 (例: 25565)」の入力欄に「9090」を入力してください。（rosbridge_suiteのデフォルトポート番号です。必要に応じて変更してください。）
1. 設定項目「プロトコル」を「TCP/UDP両方」に設定します。
1. 設定などを変更・確認後、「公開する」を押します。
1. 表示された「○○.ssnetwork.io:○○○○○」が他のユーザーが参加できる「サーバーアドレス」です。このアドレスを共有したい参加者に教えてください。参加者は、toolsのhtmlのサーバーアドレス、ポート番号を教えてもらったサーバーアドレスに変更してhtmlを開いてください。
