# ito_lab_F1Tenth - 自律走行ロボットプロジェクト 初心者向け

このリポジトリは、F1Tenthスケールロボットを用いた自律走行プロジェクトのROS (Robot Operating System) コードを管理しています。主に、RPLIDARセンサーを用いた障害物回避と壁追従によるナビゲーション機能に焦点を当てています。

## プロジェクトの目的

* RPLIDAR A1M8センサーからのデータ処理
* `laser_filters`パッケージを用いたLidarデータのフィルタリング（自己認識フィルターを含む）
* `FreeSpaceNavigator`による障害物回避と壁追従の自律走行アルゴリズムの実装
* `pigpio`ライブラリを介したRaspberry Pi GPIOによるモーターおよびステアリングのPWM制御
* `robot_state_publisher`によるロボットモデルのROSへの公開
* Rvizを用いたロボットの状態とセンサーデータの可視化

## 動作環境

* **ROS バージョン:** Noetic (Ubuntu 20.04 LTS)
* **ハードウェア:** F1Tenthスケールロボット、Raspberry Pi (pigpioを使用するため)、RPLIDAR A1M8
* **OS:** Ubuntu 20.04 LTS (Raspberry Pi OS もしくはPC上のUbuntu)

## セットアップ方法

### 1. ROSのインストール

ROS Noeticがインストールされていない場合は、公式ドキュメントに従ってインストールしてください。
[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 2. Catkinワークスペースの作成

bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

### 3. このリポジトリのクローン
catkin_ws/src ディレクトリ内で、このリポジトリをクローンします。
git clone [https://github.com/p22046/ito_lab_F1Tenth.git](https://github.com/p22046/ito_lab_F1Tenth.git)

### 4. 依存パッケージのインストール
このプロジェクトは、以下のROSパッケージに依存しています。

Bash

sudo apt-get update
sudo apt-get install ros-noetic-laser-filters \
                     ros-noetic-robot-state-publisher \
                     ros-noetic-joint-state-publisher \
                     ros-noetic-rviz \
                     ros-noetic-robot-self-filter \
                     ros-noetic-xacro # URDF/XACROを扱うため

### 5.aspberry Pi 固有の依存関係
もしRaspberry Pi上で実行する場合、pigpioライブラリが必要です。

Bash

sudo apt-get install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
5. Catkinワークスペースのビルド
Bash

cd ~/catkin_ws
catkin_make
source devel/setup.bash
注意: source devel/setup.bash は新しいターミナルを開くたびに実行するか、.bashrc に追加してください。

実行方法
1. RPLIDARのシリアルポート設定
RPLIDARが接続されているUSBポートのパーミッションを設定します。通常、/dev/ttyUSB0 です。

Bash

sudo chmod 666 /dev/ttyUSB0
注意: これは一時的な設定です。永続化するにはudevルールを設定する必要があります。

2. メインLaunchファイルの実行
すべてのノードを起動するには、以下のLaunchファイルを実行します。

Bash

roslaunch ito_lab_F1Tenth auto_drive.launch
このコマンドを実行すると、以下のノードが起動します。

robot_state_publisher: ロボットのURDFモデルを公開

joint_state_publisher: ジョイントの状態を公開（通常は固定ジョイントのみ）

rplidar_node: RPLIDARセンサーからのLidarデータ (/scan)

laser_filter: Lidarデータをフィルタリング (/scan_filtered)

free_space_navigator: 自律走行アルゴリズム (/cmd_velをパブリッシュ)

pwm_controller: モーターとステアリングのPWM制御

rviz: 視覚化ツール

3. Rvizでの確認
auto_drive.launchが起動するとRvizも自動的に立ち上がります。以下のトピックやディスプレイを確認してください。

RobotModel: ロボットの3Dモデルが表示されているか

LaserScan: /scan_filteredトピックを選択し、Lidarデータが表示されているか

Marker: debug_markersトピックを選択し、FreeSpaceNavigatorのデバッグマーカーが表示されているか

TF: ロボットの各フレーム間の座標変換が表示されているか


### UbuntuでのLaunchファイルの実行方法

Launchファイルを実行する手順は以下の通りです。

1.  **新しいターミナルを開く** (または既存のターミナルでROS環境をセットアップする)。

2.  **ROS環境をセットアップする。**
    Catkinワークスペースをビルドした後、このコマンドを実行する必要があります。新しいターミナルを開くたびに実行するか、`.bashrc`ファイルに追記して自動的に実行されるように設定します。

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

3.  **RPLIDARのシリアルポートのパーミッションを設定する。**
    これは、RPLIDARが接続されているUSBポートへのアクセス権限を与えるためのものです。ロボットを再起動したり、USBを抜き差ししたりすると、再度実行が必要になる場合があります。

    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```
    * もし`/dev/ttyUSB0`以外のポートに接続されている場合は、そのポート名に置き換えてください（例: `/dev/ttyUSB1`）。
    * 永続的に設定するには、udevルールを作成する必要がありますが、まずはこの一時的な方法で動作確認するのが一般的です。

4.  **Launchファイルを実行する。**
    あなたの`auto_drive.launch`ファイルは、`ito_lab_F1Tenth`リポジトリ（Gitでクローンしたフォルダ）の中にある`wall_follow/launch`ディレクトリに配置されているはずです。

    ```bash
    roslaunch wall_follow auto_drive.launch
    ```
    * `ito_lab_F1Tenth`リポジトリをクローンした際に、その中に`wall_follow`パッケージが含まれている場合、`roslaunch ito_lab_F1Tenth auto_drive.launch` のように、リポジトリ名をパッケージ名として使うこともできます。しかし、ROSの一般的な慣習としては、`wall_follow`がROSパッケージとして認識されているのであれば、`roslaunch wall_follow auto_drive.launch` の方がより直接的です。




                     
