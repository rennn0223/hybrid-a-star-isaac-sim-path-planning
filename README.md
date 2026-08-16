# Hybrid A* Isaac Sim 路徑規劃

這是一套適用於 NVIDIA Isaac Sim 5.1 的 Ackermann 車輛路徑規劃與視覺化腳本。程式會在已烘焙的 NavMesh 上執行僅前進的 Hybrid A* 搜尋、在 USD 場景中繪製規劃路徑、將路徑發布為具持久性的 ROS 2 `nav_msgs/Path`，並可透過運動學方式讓車輛沿著路徑行駛。

## 功能特色

- 使用 Ackermann 自行車模型進行僅前進的 Hybrid A* 搜尋
- 限制轉向角度與轉向角變化速度
- 透過 NavMesh 進行碰撞檢查，並支援可調整的車輛安全距離
- 將道路邊界距離納入成本，使路徑盡量遠離道路邊緣
- 支援先經過選用的中繼點，再前往最終目的地
- 使用 USD `BasisCurves` 顯示規劃路徑
- 以 transient-local 持久性發布 ROS 2 `nav_msgs/Path`
- 採用類 Pure Pursuit 的運動學路徑跟隨
- NavMesh 尚未準備完成時會自動重試

## 系統需求

- NVIDIA Isaac Sim 5.1
- 已烘焙 NavMesh 的 USD 場景
- Isaac Sim 導航擴充套件（`omni.anim.navigation.core`）
- Isaac Sim Core Python API
- NumPy
- 選用：ROS 2 與 Isaac Sim ROS 2 Bridge

本腳本設計為在 Isaac Sim 的 Script Editor 中執行，不能直接當作獨立 Python 程式執行。

## 場景設定

預設設定會尋找以下 USD prim：

| 用途 | Prim 路徑或名稱 |
| --- | --- |
| 車輛 | `/root/white_vehicle_v2` |
| 目的地 | `/root/_731/destination` |
| 選用中繼點 | 名稱唯一的 `move_ball` prim |
| 路徑視覺化 | `/root/Debug_Navigation_0816/PlannedAckermann` |

執行腳本前：

1. 在 Isaac Sim 中開啟地圖場景。
2. 為可行駛區域烘焙 NavMesh。
3. 確認車輛、目的地與選用的中繼點 prim 均存在。
4. 修改 `0816.py` 頂部附近的設定常數，使其符合你的場景與車輛。

## 使用方式

1. 在 Isaac Sim 中開啟 **Window > Script Editor**。
2. 在編輯器中開啟或貼上 `0816.py`。
3. 等待場景與 NavMesh 準備完成後執行腳本。
4. 在 Script Editor 輸出中查看 `[INFO]`、`[WARN]` 與 `[ERROR]` 診斷訊息。

規劃成功後，腳本會：

1. 將車輛與目標位置投影至 NavMesh。
2. 使用 Hybrid A* 規劃每一段路徑。
3. 驗證曲率、NavMesh 佔用狀態與道路邊界安全距離。
4. 在場景中繪製橘色路徑曲線。
5. 啟用 ROS 2 時發布完整路徑。
6. 啟用自動跟隨時讓車輛沿著路徑移動。

再次執行腳本時，程式會先安全關閉先前的應用程式實例，再啟動新的實例。

## 參數設定

主要設定集中於 `0816.py` 頂部附近：

- **場景：** 車輛、目的地、中繼點與除錯曲線的 prim 路徑
- **車輛：** 軸距、輪距、最大轉向角與安全邊距
- **規劃器：** 步長、網格解析度、朝向分箱數、轉向層級、搜尋邊界與最大展開節點數
- **安全距離：** NavMesh 道路邊界的最低與期望安全距離
- **跟隨器：** 速度、前視距離、到達容許距離與時間軸行為
- **ROS 2：** 節點名稱、topic 與 frame ID

內建的預設車輛參數如下：

| 參數 | 數值 |
| --- | ---: |
| 軸距 | 1.28139 m |
| 輪距 | 0.94942 m |
| 最大轉向角 | 30° |
| NavMesh agent 半徑 | 0.50 m |
| 安全邊距 | 0.05 m |

## ROS 2 輸出

當 `ENABLE_ROS2 = True` 時，路徑會發布至：

- Topic：`/sim/planned_path/full`
- 訊息類型：`nav_msgs/msg/Path`
- Frame：`map`
- QoS：reliable、keep last 1、transient local

發布前，程式會將位置從 USD 場景單位換算為公尺。

## 目前限制

- 規劃器僅支援向前行駛，不會產生倒車路徑。
- 運動學跟隨器會直接更新車輛 transform，並非物理式控制器。
- 場景 prim 路徑與車輛幾何尺寸為專案特定設定，套用至其他場景時必須調整。
- 規劃結果仰賴正確烘焙且連通的 NavMesh。

## 專案檔案

- `0816.py` — 路徑規劃、驗證、視覺化、ROS 2 發布與路徑跟隨
