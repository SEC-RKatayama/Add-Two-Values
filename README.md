# Add two values

# 使い方
## 準備
まず、sshでgitにアクセスできる状態に準備する。

## 本リポジトリをクローン
```
git clone git@github.com:SEC-RKatayama/Add-Two-Values.git
```

## ビルド
クローンしたディレクトリで
```
colcon build --symlink-install --packages-select add_two_values
```

## 実行
ビルドしたターミナルとは別の新規ターミナルを2つ開く。
場所はクローンしたディレクトリ。
一方でノードを起動（うまくいっていればTab補完が効く）。
```
source install/setup.bash
```
```
ros2 launch add_two_values add_two_values.launch.py
```
うまくいっていれば、そうとわかる。

もう一つのターミナルで、足す数を入力する。
足す数はval1とval2と名付けてある。
```
source install/setup.bash
```
```
ros2 topic pub --once /val1 std_msgs/msg/Float64 "{data: <数値>}"
```
```
ros2 topic pub --once /val2 std_msgs/msg/Float64 "{data: <数値>}"
```
入力したら足し算の結果が1つ目のターミナルに表示される。
