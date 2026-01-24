# Landing Target 3D

MAVLink LANDING_TARGET (ID 149) と ATTITUDE (ID 30) メッセージを使用した精密着陸状態の3D可視化ツール。

## セットアップ

```bash
cd /home/rtos/github/WebTools/LandingTarget3D
npm install
```

## 実行

### デフォルト設定（127.0.0.1:14570）

```bash
npm start
```

### カスタム設定

```bash
node server.js <MAVLink IP> <MAVLink Port> <HTTP Port>
```

例:
```bash
node server.js 127.0.0.1 14570 8080
```

## ブラウザでアクセス

```
http://localhost:8080
```

## Mission Planner設定

1. Mission Plannerを起動
2. **Ctrl+F** → **MAVLink** → **UDP Client** を選択
3. **Remote Host**: このPCのIPアドレス
4. **Remote Port**: 14561（server.jsが受信するポート）

または、Mission Plannerで:
1. **Config/Tuning** → **Planner**
2. UDP出力を有効化

## 機能

- 着陸ターゲット（ArUcoスタイルマーカー）の固定表示
- 機体の相対位置表示（NED座標系から変換）
- 機体姿勢（Roll/Pitch/Yaw）の反映
- ステータス表示（距離、位置、角度、有効性）
- 軌跡表示
- OrbitControlsによるカメラ操作

## 操作

- **左ドラッグ**: 回転
- **右ドラッグ**: パン
- **スクロール**: ズーム
