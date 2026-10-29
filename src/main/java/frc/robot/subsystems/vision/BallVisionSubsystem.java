package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 訂閱 Jetson 發布的球體視覺資料。
 *
 * <p><b>這個 subsystem 不致動任何東西</b>——它只提供量測。
 * 要不要用、怎麼用，由 Command 決定。這樣影子模式與輔助介入共用同一份程式，
 * 切換時不必改架構。
 *
 * <p><b>每個週期只讀一次</b>：{@link #periodic()} 讀取並快取成一份不可變快照，
 * 同一迴圈內所有消費者看到同一份資料。直接在 Command 裡各自呼叫 NT4
 * 會讓同一週期內的 tx 與 distance 來自不同幀。
 */
public class BallVisionSubsystem extends SubsystemBase {

    /** 與 Jetson 的 config/pipeline.yaml 的 networktables.table 必須一致。 */
    private static final String TABLE = "ballvision";

    /**
     * 心跳在這段時間內沒有遞增就視為失效。
     * Jetson 以 30 fps 發布（33 ms），取 5 個週期的餘裕。
     * ⚠️ 此值應依實測的端到端延遲調整，見 docs/acceptance-ledger。
     */
    private static final double HEARTBEAT_TIMEOUT_SECONDS = 0.25;

    /**
     * 資料超過這個年齡就不用於控制。
     * 理論端到端延遲 58–78 ms，取 150 ms 作為上限。
     */
    private static final double MAX_AGE_SECONDS = 0.15;

    /**
     * 機器人端的信心度門檻。
     * ⚠️ 這是<b>第二道</b>門檻——Jetson 端已依 PR 曲線過濾過一次。
     * 這裡只在需要更保守時才調高；填 0 表示完全信任 Jetson 的判斷。
     * 不得憑感覺填 0.75 之類的數字。
     */
    private static final double MIN_SCORE = 0.0;

    private final BooleanSubscriber tvSub;
    private final DoubleSubscriber txSub;
    private final DoubleSubscriber tySub;
    private final DoubleSubscriber taSub;
    private final DoubleSubscriber distanceSub;
    private final DoubleSubscriber scoreSub;
    private final DoubleSubscriber totalLatencySub;
    private final DoubleSubscriber inferLatencySub;
    private final DoubleSubscriber uSub;
    private final DoubleSubscriber vSub;
    private final DoubleSubscriber boxWSub;
    private final DoubleSubscriber boxHSub;
    private final IntegerSubscriber candidatesSub;
    private final IntegerSubscriber heartbeatSub;
    private final IntegerSubscriber frameSub;
    private final StringSubscriber modeSub;
    private final BooleanSubscriber calibratedSub;

    private final NetworkTableInstance instance;

    private long lastHeartbeat = -1L;
    private double lastHeartbeatChangeSeconds = 0.0;
    private BallVisionMeasurement latest =
            BallVisionMeasurement.unavailable(BallVisionMeasurement.Validity.DISCONNECTED);

    public BallVisionSubsystem() {
        this(NetworkTableInstance.getDefault());
    }

    /** 可注入 instance，方便單元測試對接本機 NT4 伺服器。 */
    public BallVisionSubsystem(NetworkTableInstance instance) {
        this.instance = instance;
        NetworkTable table = instance.getTable(TABLE);

        tvSub = table.getBooleanTopic("tv").subscribe(false);
        txSub = table.getDoubleTopic("tx").subscribe(0.0);
        tySub = table.getDoubleTopic("ty").subscribe(0.0);
        taSub = table.getDoubleTopic("ta").subscribe(0.0);
        distanceSub = table.getDoubleTopic("distance").subscribe(Double.NaN);
        scoreSub = table.getDoubleTopic("score").subscribe(0.0);
        totalLatencySub = table.getDoubleTopic("latency_total_ms").subscribe(0.0);
        inferLatencySub = table.getDoubleTopic("latency_infer_ms").subscribe(0.0);
        uSub = table.getDoubleTopic("u").subscribe(Double.NaN);
        vSub = table.getDoubleTopic("v").subscribe(Double.NaN);
        boxWSub = table.getDoubleTopic("box_w").subscribe(0.0);
        boxHSub = table.getDoubleTopic("box_h").subscribe(0.0);
        candidatesSub = table.getIntegerTopic("n_candidates").subscribe(0);
        heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(0);
        frameSub = table.getIntegerTopic("frame").subscribe(0);
        modeSub = table.getStringTopic("mode").subscribe("unknown");
        calibratedSub = table.getBooleanTopic("calibrated").subscribe(false);
    }

    @Override
    public void periodic() {
        latest = read();
        publishTelemetry();
    }

    /** 本週期的量測快照。永不回傳 null。 */
    public BallVisionMeasurement get() {
        return latest;
    }

    /** 便利方法：這一幀能否用於控制。 */
    public boolean hasUsableTarget() {
        return latest.validity().usable();
    }

    private BallVisionMeasurement read() {
        double now = NetworkTablesJNI.now() / 1e6;

        if (!instance.isConnected()) {
            return BallVisionMeasurement.unavailable(
                    BallVisionMeasurement.Validity.DISCONNECTED);
        }

        // 心跳停止遞增代表 Jetson 端程式掛了或相機斷線。
        // 不能只看 isConnected()——NT4 連線還在但發布端已死的情況很常見。
        long heartbeat = heartbeatSub.get();
        if (heartbeat != lastHeartbeat) {
            lastHeartbeat = heartbeat;
            lastHeartbeatChangeSeconds = now;
        } else if (now - lastHeartbeatChangeSeconds > HEARTBEAT_TIMEOUT_SECONDS) {
            return BallVisionMeasurement.unavailable(
                    BallVisionMeasurement.Validity.STALE);
        }

        // getAtomic() 連帶取得「值寫入 NT 伺服器的時間戳」。
        // 在 FRC 中 roboRIO 就是 NT 伺服器，ntcore 的時基即 FPGA 時間，
        // 所以這個時間戳可以直接與 Timer.getFPGATimestamp() 比較。
        TimestampedDouble txStamped = txSub.getAtomic();
        double totalLatencyMillis = totalLatencySub.get();

        // 曝光時刻 = 值抵達伺服器的時間 − Jetson 內部處理花的時間。
        // Jetson 回報的是「時間長度」而非絕對時間，所以不需要時鐘對齊——
        // 兩端時域不同的問題在契約設計上就被繞開了。
        double captureSeconds = txStamped.timestamp / 1e6 - totalLatencyMillis / 1000.0;

        boolean hasTarget = tvSub.get();
        double score = scoreSub.get();
        boolean calibrated = calibratedSub.get();

        BallVisionMeasurement.Validity validity;
        if (!hasTarget) {
            validity = BallVisionMeasurement.Validity.NO_TARGET;
        } else if (score < MIN_SCORE) {
            validity = BallVisionMeasurement.Validity.LOW_CONFIDENCE;
        } else if (now - captureSeconds > MAX_AGE_SECONDS) {
            validity = BallVisionMeasurement.Validity.TOO_OLD;
        } else {
            validity = BallVisionMeasurement.Validity.OK;
        }

        return new BallVisionMeasurement(
                hasTarget,
                txStamped.value,
                tySub.get(),
                taSub.get(),
                distanceSub.get(),
                score,
                (int) candidatesSub.get(),
                captureSeconds,
                totalLatencyMillis,
                inferLatencySub.get(),
                uSub.get(),
                vSub.get(),
                boxWSub.get(),
                boxHSub.get(),
                frameSub.get(),
                modeSub.get(),
                calibrated,
                validity);
    }

    private void publishTelemetry() {
        BallVisionMeasurement m = latest;
        SmartDashboard.putString("BallVision/validity", m.validity().name());
        SmartDashboard.putString("BallVision/mode", m.mode());
        SmartDashboard.putBoolean("BallVision/calibrated", m.calibrated());
        SmartDashboard.putNumber("BallVision/tx", m.txDegrees());
        SmartDashboard.putNumber("BallVision/score", m.score());
        SmartDashboard.putNumber("BallVision/candidates", m.candidateCount());
        SmartDashboard.putNumber("BallVision/latencyMs", m.totalLatencyMillis());
        SmartDashboard.putNumber("BallVision/ageMs",
                m.ageSeconds(NetworkTablesJNI.now() / 1e6) * 1000.0);
        // 距離可能是 NaN；SmartDashboard 會顯示為 NaN，這是刻意的——
        // 顯示 0 會讓駕駛以為球就在腳邊。
        SmartDashboard.putNumber("BallVision/distanceM", m.distanceMeters());
    }
}
