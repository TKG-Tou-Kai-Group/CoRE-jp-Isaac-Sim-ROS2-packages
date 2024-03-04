# CoREjp-Isaac-Sim-ROS2-packages
- サンプルロボット操縦デモ
  ![ros2_control_demo](figs/CoREjp_robot_test3.gif)

このリポジトリは，NvidiaのIsaac SimでROS 2で作ったロボットを開発するためのパッケージをまとめたものです．
重要なパッケージは"isaac_ros2_utils"内に格納された"isaac_ros2_control"と"isaac_ros2_scripts"です．
"isaac_ros2_control" は hardware_interface クラスを提供し，ros2_control からのコマンドを "isaac_ros2_scripts" に送ります．
"isaac_ros2_scripts "にはIsaac Simを起動し制御するためのpythonスクリプトがあります．

このリポジトリでできること
- CoREjp2024の1部リーグ決勝トーナメント用ステージの生成
- オムニホイールロボットの生成
- ROS 2のteleop_twist_keyboadパッケージでの操縦
- ROSトピックとして、ロボットに取り付けられたカメラ映像を取得

## 必要なもの
1. Docker
1. Isaac SimのDockerイメージ (このリポジトリでは nvcr.io/nvidia/isaac-sim:2023.1.1 を使ってテストしています)

## 使い方
1. Dockerをインストールして、Isaac SimのDockerイメージをプルする

   このURLを参考にイメージをプルしてください https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_container.html

1. このリポジトリをクローンする
   ```bash
   git clone https://github.com/hijimasa/isaac-ros2-control-sample.git
   ```

1. サブモジュール（オムニホイールコントローラ）をセットアップする
   ```bash
   cd isaac-ros2-control-sample/docker
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
   colcon build && source install/setup.bash
   ```

4. パッケージを起動する
   - シミュレータおよびステージの立ち上げ
   
   立ち上げたコンテナ内で以下のコマンドを実行してください

   立ち上がるまでに数分かかるので、気長に待ってください
   ```bash
   ros2 launch sample_robot_sim bring_up_core_stage.launch.py
   ```
   - ロボットの生成

   シミュレータが完全に立ち上がってから（ステージが表示されてから）、シミュレータ左の四角ボタンでシミュレーションを停止させて、別のターミナルから下記のコマンドを実行してください

   シミュレーションを停止させない場合には、ディスクの生成中にセグメンテーションフォルトでプログラムが落ちる場合があります
   ```bash
   docker exec -it isaac-sim /bin/bash
   ros2 launch sample_robot_sim bring_up_core_stage.launch.py
   ```
   ディスクが一通り生成し終わったら、シミュレータ左の三角ボタンでシミュレーションを開始してください

   - 操作方法
   
   基本はteleop_twist_joyのps3-holonomic.config.yamlを使用しています。
   
   PSコントローラだと、
   
   - Rボタン: イネーブルボタン（押さないとロボットが移動しない）
   - 左スティック：ロボットの前後左右移動
   - 右スティック左右：ロボットの回転
   - 丸ボタン：ディスク射出（単発）
