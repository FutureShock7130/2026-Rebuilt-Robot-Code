package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * 以視覺偏角修正車頭朝向。帶濾波微分項的 PD，含四道安全互鎖。
 *
 * <p>這支 Command 與 Jetson 端的 {@code src/control.py} 是同一套控制律，
 * 刻意分別實作在兩邊：Jetson 端用於影子模式記錄 {@code u_would_be}，
 * 可離線調參；roboRIO 端才真的輸出。兩邊的參數必須一致。
 *
 * <p><b>參數未量測時這支 Command 不會輸出任何東西。</b>
 * 常數預設為 {@link Double#NaN}，{@link #initialize()} 會偵測並拒絕。
 * 這是依專案規則：未經動力學計算的控制常數不得硬編碼。
 */
public class AlignToBallCommand extends Command {

    /**
     * 控制常數。全部預設 NaN = 未量測。
     *
     * <p>填入前必須完成的推算與實測，見 docs/acceptance-ledger：
     * <ul>
     *   <li>{@code kP} = uMax / 飽和角。取 intake 導流板極限 15°、uMax 0.30 → 0.0200
     *   <li>{@code kD} = τ · kP。航向迴路 τ ≈ 0.1 s → 0.0020
     *   <li>{@code derivativeFilterBeta} 由實測 tx 標準差回推。
     *       <b>注意群延遲</b>：β=0.8 要付 80 ms，是整條視覺管線的 8 倍。
     *       β=0.6 給 2× 衰減、30 ms 延遲，是較平衡的起點。
     *       若實測 tx 標準差已在 0.1° 量級，可能根本不需要重濾波。
     *   <li>{@code uMax} 由底盤傳動比、輪距與 intake 開口角推算。<b>硬上限 0.35</b>
     *   <li>{@code joystickDeadband} 由駕駛操作手感實測
     *   <li>{@code lostTimeoutSeconds} 須 ≥ 通過近場盲區的時間。
     *       0.3 m 盲區、1.5 m/s → 0.200 s
     *   <li>{@code lostDecayLambda} 在 lostTimeout 內衰減到 10% → 0.794
     * </ul>
     */
    public static final class Config {
        public double kP = Double.NaN;
        public double kD = Double.NaN;
        public double derivativeFilterBeta = Double.NaN;
        public double uMax = Double.NaN;
        public double joystickDeadband = Double.NaN;
        public double lostTimeoutSeconds = Double.NaN;
        public double lostDecayLambda = Double.NaN;

        /** 安全上限。任何 uMax 都不得超過，掃描參數時也一樣。 */
        public static final double U_MAX_SAFETY_CEILING = 0.35;

        public String firstProblem() {
            if (Double.isNaN(kP)) return "kP";
            if (Double.isNaN(kD)) return "kD";
            if (Double.isNaN(derivativeFilterBeta)) return "derivativeFilterBeta";
            if (Double.isNaN(uMax)) return "uMax";
            if (Double.isNaN(joystickDeadband)) return "joystickDeadband";
            if (Double.isNaN(lostTimeoutSeconds)) return "lostTimeoutSeconds";
            if (Double.isNaN(lostDecayLambda)) return "lostDecayLambda";
            if (uMax > U_MAX_SAFETY_CEILING) {
                return "uMax=" + uMax + " 超過安全上限 " + U_MAX_SAFETY_CEILING;
            }
            return null;
        }
    }

    private final BallVisionSubsystem vision;
    private final Config config;
    private final DoubleConsumer steerOutput;
    private final BooleanSupplier assistHeld;
    private final DoubleSupplier joystickYaw;

    private boolean disabled = false;
    private double previousTx = Double.NaN;
    private double filteredDerivative = 0.0;
    private double lastOutput = 0.0;
    private double lostSinceSeconds = Double.NaN;
    private double lastLoopSeconds = Double.NaN;

    /**
     * @param steerOutput 送出轉向修正量（−1..1）。
     *     <b>正負號必須在台車上實測確認</b>——tx 是「目標在右為正」，
     *     但底盤的正轉方向依慣例是逆時針，所以這裡通常要取負。弄反會往反方向追球。
     * @param assistHeld 死人開關：駕駛按住輔助鍵才作用
     * @param joystickYaw 駕駛手動轉向輸入，用於判定搶佔
     * @param driveRequirement 底盤 subsystem，宣告為需求以避免與手動駕駛指令衝突
     */
    public AlignToBallCommand(
            BallVisionSubsystem vision,
            Config config,
            DoubleConsumer steerOutput,
            BooleanSupplier assistHeld,
            DoubleSupplier joystickYaw,
            Subsystem driveRequirement) {
        this.vision = vision;
        this.config = config;
        this.steerOutput = steerOutput;
        this.assistHeld = assistHeld;
        this.joystickYaw = joystickYaw;
        addRequirements(vision, driveRequirement);
    }

    @Override
    public void initialize() {
        String problem = config.firstProblem();
        disabled = problem != null;
        if (disabled) {
            // 不拋例外——比賽中讓機器人掉線比不作用更糟。
            // 改為停用並在儀表板明示原因。
            SmartDashboard.putString("BallVision/alignBlocked",
                    "控制參數未量測：" + problem);
            System.err.println("[AlignToBall] 已停用，控制參數未量測：" + problem);
        }
        previousTx = Double.NaN;
        filteredDerivative = 0.0;
        lastOutput = 0.0;
        lostSinceSeconds = Double.NaN;
        lastLoopSeconds = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (disabled) {
            steerOutput.accept(0.0);
            return;
        }

        double now = Timer.getFPGATimestamp();
        double dt = Double.isNaN(lastLoopSeconds) ? 0.0 : now - lastLoopSeconds;
        lastLoopSeconds = now;

        BallVisionMeasurement m = vision.get();
        String blocked = null;
        double applied = 0.0;

        if (!m.validity().usable()) {
            // 目標丟失：先平滑衰減，逾時後硬性歸零。
            // 衰減是漸近的，單靠自己到不了 0——硬切斷是必要條件而非備援。
            if (Double.isNaN(lostSinceSeconds)) {
                lostSinceSeconds = now;
            }
            if (now - lostSinceSeconds >= config.lostTimeoutSeconds) {
                lastOutput = 0.0;
                blocked = "lost_timeout";
            } else {
                lastOutput *= config.lostDecayLambda;
                blocked = "lost_decaying";
            }
            applied = lastOutput;
            previousTx = Double.NaN;   // 重新出現時不要用舊值算微分
        } else {
            lostSinceSeconds = Double.NaN;
            double tx = m.txDegrees();

            double rawDerivative = 0.0;
            if (!Double.isNaN(previousTx) && dt > 0.0) {
                rawDerivative = (tx - previousTx) / dt;
            }
            double beta = config.derivativeFilterBeta;
            filteredDerivative = beta * filteredDerivative + (1.0 - beta) * rawDerivative;
            previousTx = tx;

            double raw = config.kP * tx + config.kD * filteredDerivative;
            lastOutput = MathUtil.clamp(raw, -config.uMax, config.uMax);
            applied = lastOutput;
        }

        // 互鎖：駕駛搶佔優先於死人開關——駕駛正在閃避時，按不按輔助鍵都該讓位
        if (Math.abs(joystickYaw.getAsDouble()) > config.joystickDeadband) {
            applied = 0.0;
            blocked = "manual_override";
        } else if (!assistHeld.getAsBoolean()) {
            applied = 0.0;
            blocked = "deadman_released";
        }

        SmartDashboard.putString("BallVision/alignBlocked",
                blocked == null ? "" : blocked);
        SmartDashboard.putNumber("BallVision/uApplied", applied);
        steerOutput.accept(applied);
    }

    @Override
    public void end(boolean interrupted) {
        steerOutput.accept(0.0);
        lastOutput = 0.0;
    }

    /** 不自行結束；由觸發條件（whileTrue / toggleOnTrue）決定存續。 */
    @Override
    public boolean isFinished() {
        return false;
    }
}
