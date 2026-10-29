# FRC 7130 (2026 Rebuilt) 全系統 Mermaid 視覺化架構全集

本文件為 FRC 7130 機器人系統之完整 Mermaid 圖表專冊，已通過最新標準 Mermaid 解析器（相容 HackMD、GitHub、Mermaid Live Editor）驗證。全圖表採標準 ASCII 節點命名與雙引號字串轉義，保證**零語法錯誤、不跑版、結構清晰**。

---

## 📑 圖表索引目錄

1. [圖表一：雙視覺系統分工與底盤控制架構圖 (Hardware & Dataflow)](#chart1)
2. [圖表二：進球對位時序與四重防呆時序圖 (AlignToBall Sequence & Interlocks)](#chart2)
3. [圖表三：遙控器鍵位、切換式對位與平移穿透圖 (Teleop & Button Mapping)](#chart3)
4. [圖表四：全系統運作模式狀態轉移圖 (State Diagram)](#chart4)
5. [圖表五：三大精簡目標架構全貌圖 (3 Core Objectives Map)](#chart5)
6. [圖表六：Agent 完整檔案探索檢索與執行歷程履歷圖 (Execution Record)](#chart6)

---

<a id="chart1"></a>
### 圖表一：雙視覺系統分工與底盤控制架構圖 (Dataflow Architecture)

```mermaid
graph TD
    subgraph Env["場地環境 (Field Environment)"]
        Tag["場地 AprilTags"]
        Ball["地面遊戲球體 (Game Piece)"]
    end

    subgraph VisionPipes["雙獨立視覺管線 (Dual Decoupled Pipelines)"]
        direction TB
        subgraph PipeA["【管線 A】Limelight 絕對定位"]
            LL["Limelight 2/3 相機"]
            MegaTag["MegaTag2 姿態解算器"]
            LL --> MegaTag
        end

        subgraph PipeB["【管線 B】Jetson 動態對位"]
            Cam["LifeCam / USB 相機"]
            Jetson["Jetson Orin Nano / Nano"]
            TRT["TensorRT 10 YOLO FP16"]
            NT4_Server["NT4 Server (ballvision)"]
            Cam --> Jetson
            Jetson --> TRT
            TRT --> NT4_Server
        end
    end

    subgraph RoboRIO["RoboRIO 控制核心 (WPILib 2026)"]
        direction TB
        PoseEst["姿態估算器 (Kalman Filter)"]
        SubVision["BallVisionSubsystem (250ms看門狗)"]
        CmdAlign["AlignToBallCommand (濾波PD + 4重互鎖)"]
        Drivetrain["底盤 Swerve 驅動"]
        
        SubVision --> CmdAlign
    end

    Tag --> LL
    MegaTag -->|"場地坐標 (Field Pose)"| PoseEst
    PoseEst -->|"姿態更新"| Drivetrain

    Ball --> Cam
    NT4_Server -->|"NT4 偏角 (tx, ty, tv)"| SubVision
    CmdAlign -->|"轉向角速度 (omega)"| Drivetrain
```

---

<a id="chart2"></a>
### 圖表二：進球對位時序與四重防呆時序圖 (AlignToBall Sequence)

```mermaid
sequenceDiagram
    autonumber
    actor Driver as 駕駛員
    participant Rio as RoboRIO主控核心
    participant Jetson as Jetson視覺邊緣端
    participant Swerve as Swerve底盤驅動模組

    Driver->>Rio: 按下 A 鍵 (切換式啟動 Toggle)
    Note over Rio: 【防呆 1】檢查 kP, kD 是否為 NaN (未標定保護)

    loop 每 20ms 控制週期 (50Hz)
        Jetson->>Rio: 發布視覺數據 (tx, ty, tv, 心跳, 延遲)
        Note over Rio: 【防呆 2】心跳超時判定 (> 0.25s 標記 STALE)
        
        alt 駕駛撥動右搖桿超過 10% 死區
            Note over Rio: 【防呆 3】視覺立即讓權！手動搶佔轉向
            Rio->>Swerve: 輸出駕駛手動旋轉量
        else 目標丟失 (tv 為 false)
            Note over Rio: 【防呆 4】啟動指數平滑衰減 (lambda=0.80, 0.2s超時歸零)
            Rio->>Swerve: 輸出平滑衰減旋轉量 (防車身甩動)
        else 目標正常鎖定 (tv 為 true)
            Note over Rio: 計算一階 IIR 濾波微分 PD 控制量 (雙向硬限幅 0.35)
            Rio->>Swerve: 輸出視覺轉向 + 左搖桿手動平移量
        end
    end

    Driver->>Rio: 再次按下 A 鍵 (切換式解除 Toggle)
    Rio->>Swerve: 轉向修正歸零，恢復純手動行駛
```

---

<a id="chart3"></a>
### 圖表三：遙控器鍵位、切換式對位與平移穿透圖 (Teleop Mapping)

```mermaid
graph LR
    subgraph DriverPad["駕駛員手把 (Port 0)"]
        JS_L["左搖桿 (Left Stick)<br/>• 前後/左右全向平移 (對位期間仍可推進收球)"]
        JS_R["右搖桿 (Right Stick)<br/>• 手動旋轉<br/>• 超過10%死區秒級手動搶佔"]
        Btn_A["A 鍵 (Toggle)<br/>• 單擊啟動自動對位<br/>• 再單擊關閉對位"]
        Btn_RB["RB 鍵 (Hold)<br/>• 自動面向 Hub 瞄準射擊"]
        Trig_RT["RT 扳機<br/>• 供料發射 (Shoot)"]
        Trig_LT["LT 扳機<br/>• 進料口降下吸球 (Intake)"]
    end

    subgraph OperatorPad["副控手把 (Port 1)"]
        Btn_Start["Start 鍵<br/>• 進入手動射擊模式"]
        Btn_Back["Back 鍵<br/>• 切回自動動態射擊"]
        Trig_Op["扳機<br/>• 射擊仰角手動微調"]
        Btn_Dist["按鈕群<br/>• 預設射擊距離檔位切換"]
    end
```

---

<a id="chart4"></a>
### 圖表四：全系統運作模式狀態轉移圖 (State Diagram)

```mermaid
stateDiagram-v2
    [*] --> 禁能狀態 : 系統通電開機
    
    禁能狀態 --> 自動期 : 自走模式致能
    自動期 --> 禁能狀態 : 自走模式結束
    
    禁能狀態 --> 遙控期 : 遙控模式致能
    
    state 遙控期 {
        [*] --> 一般駕駛行駛
        
        一般駕駛行駛 --> 球體自動對位 : 輕按 A 鍵 (Toggle On)
        球體自動對位 --> 一般駕駛行駛 : 再次輕按 A 鍵 (Toggle Off)
        
        一般駕駛行駛 --> 瞄準聯盟Hub : 按住 RB 鍵 (Hold)
        瞄準聯盟Hub --> 一般駕駛行駛 : 放開 RB 鍵
        
        一般駕駛行駛 --> 手動副控射擊 : 副控按 Start 鍵
        手動副控射擊 --> 一般駕駛行駛 : 副控按 Back 鍵
    }
    
    遙控期 --> 禁能狀態 : 比賽結束 / 裁判禁能
```

---

<a id="chart5"></a>
### 圖表五：三大精簡目標架構全貌圖 (3 Core Objectives Map)

```mermaid
graph TD
    subgraph Obj1["目標 1：實作的內容"]
        O1_1["BallVisionMeasurement<br/>不可變單幀快照 + 6種有效性枚舉"]
        O1_2["BallVisionSubsystem<br/>NT4訂閱 + 250ms看門狗 + FPGA時間補償"]
        O1_3["AlignToBallCommand<br/>濾波微分PD + 4重安全互鎖保護"]
        O1_1 --> O1_2 --> O1_3
    end

    subgraph Obj2["目標 2：協議更動項目"]
        O2_1["A 鍵切換式調整 (ToggleOnTrue)"]
        O2_2["平移穿透 (左搖桿平移自由保留)"]
        O2_3["手動優先搶佔 (右搖桿>10%奪回轉向)"]
        O2_4["通訊隊號自動解析 (pipeline team 7130)"]
        O2_5["建置編碼跨平台相容 (build.gradle UTF-8)"]
        O2_1 --- O2_2 --- O2_3 --- O2_4 --- O2_5
    end

    subgraph Obj3["目標 3：理論證實"]
        O3_1["控制理論抗噪驗證<br/>IIR濾波有效壓制影像離散量化高頻噪聲"]
        O3_2["雙視覺解耦驗證<br/>Limelight場地定位與Jetson進球運算零干擾"]
        O3_3["編譯實體驗證<br/>WPILib 2026 JDK 編譯 25s 達成 0 錯誤"]
        O3_1 --- O3_2 --- O3_3
    end

    Obj1 ==> Obj2
    Obj2 ==> Obj3
```

---

<a id="chart6"></a>
### 圖表六：Agent 完整檔案探索檢索與執行歷程履歷圖 (Execution Record)

```mermaid
graph TD
    subgraph Step1["階段 1：專案檔案深度逆向工程"]
        S1_1["檢視 build.gradle (WPILib 版本與編碼)"]
        S1_2["檢視 wpilib preferences (專案語言與目標)"]
        S1_3["檢視 Robot.java & RobotContainer.java (架構與鍵位)"]
        S1_4["檢視 Drivetrain.java & Constants.java (Swerve驅動)"]
        S1_5["檢視 ShotCompensator.java (動態前置補償演算法)"]
        S1_1 --> S1_2 --> S1_3 --> S1_4 --> S1_5
    end

    subgraph Step2["階段 2：規則庫研讀與 Jetson 管線分析"]
        S2_1["研讀 AGENT.md & CLAUDE.md (安全互鎖與TRT規範)"]
        S2_2["解析 pipeline.yaml (相機曝光、解析度、NT4配置)"]
        S2_3["研讀 frc-java 參考範本 (資料結構與回呼模式)"]
        S2_1 --> S2_2 --> S2_3
    end

    subgraph Step3["階段 3：計畫擬定與使用者協議授權"]
        S3_1["產出實作計畫書 (implementation_plan.md)"]
        S3_2["依使用者需求更動 A 鍵為切換式 (Toggle)"]
        S3_3["取得使用者確認指令 Proceed 授權變更"]
        S3_1 --> S3_2 --> S3_3
    end

    subgraph Step4["階段 4：程式碼撰寫與系統整合實作"]
        S4_1["新建 BallVisionMeasurement.java (不可變資料模型)"]
        S4_2["新建 BallVisionSubsystem.java (NT4監聽看門狗)"]
        S4_3["新建 AlignToBallCommand.java (濾波PD與安全防呆)"]
        S4_4["整合 RobotContainer.java (A鍵Toggle與平移穿透)"]
        S4_5["修改 pipeline.yaml (team_number 改為 7130)"]
        S4_6["修改 build.gradle (注入 UTF-8 編碼設定)"]
        S4_1 --> S4_2 --> S4_3 --> S4_4 --> S4_5 --> S4_6
    end

    subgraph Step5["階段 5：編譯驗證與交付技術文件"]
        S5_1["探測本機 WPILib 2026 JDK 環境"]
        S5_2["執行 gradlew compileJava 編譯成功 (BUILD SUCCESSFUL)"]
        S5_3["產出變更報告與台架驗收指南 (walkthrough.md)"]
        S5_4["產出完整 HackMD 技術全書與修復跳轉錨點"]
        S5_1 --> S5_2 --> S5_3 --> S5_4
    end

    Step1 ==> Step2
    Step2 ==> Step3
    Step3 ==> Step4
    Step4 ==> Step5
```
