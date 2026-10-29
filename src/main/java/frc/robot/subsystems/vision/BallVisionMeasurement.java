package frc.robot.subsystems.vision;

/**
 * Jetson 單一幀的視覺量測結果。
 *
 * <p>這是不可變快照：同一個控制週期內所有消費者看到的是同一份資料，
 * 不會因為 NT4 在迴圈中途更新而出現前後不一致（tearing）。
 *
 * <p><b>座標與單位約定</b>
 * <ul>
 *   <li>{@code txDegrees} 水平偏角，<b>目標在右為正</b>。繞 Z 軸的右手定律是逆時針為正，
 *       所以送進底盤的轉向指令通常要取負號——這個正負號必須在台車上實測確認，
 *       弄反會讓機器人往反方向追球。
 *   <li>{@code tyDegrees} 垂直偏角，目標在上為正。
 *   <li>{@code taPercent} 外接框面積佔畫面百分比，0–100。
 *   <li>{@code distanceMeters} <b>水平地面距離</b>，不是斜距。
 *       未標定時為 {@code NaN}——不是 0，因為 0 是個看起來合理的距離。
 *   <li>{@code captureTimestampSeconds} 影格<b>曝光時刻</b>的 FPGA 時間，
 *       已扣除 Jetson 內部處理延遲與網路傳輸。可直接餵給 pose estimator 的
 *       {@code addVisionMeasurement}。
 * </ul>
 */
public record BallVisionMeasurement(
        boolean hasTarget,
        double txDegrees,
        double tyDegrees,
        double taPercent,
        double distanceMeters,
        double score,
        int candidateCount,
        double captureTimestampSeconds,
        double totalLatencyMillis,
        double inferenceLatencyMillis,
        double pixelU,
        double pixelV,
        double boxWidthPixels,
        double boxHeightPixels,
        long frame,
        String mode,
        boolean calibrated,
        Validity validity) {

    /** 這一幀能不能用，以及不能用的原因。 */
    public enum Validity {
        /** 可用。 */
        OK,
        /** NT4 未連線到 Jetson。 */
        DISCONNECTED,
        /** 心跳停止遞增——Jetson 程式掛了或相機斷線。 */
        STALE,
        /** Jetson 回報本幀沒有目標。 */
        NO_TARGET,
        /** 信心度低於機器人端設定的門檻。 */
        LOW_CONFIDENCE,
        /** 資料過舊，超過可接受的延遲上限。 */
        TOO_OLD;

        public boolean usable() {
            return this == OK;
        }
    }

    /** 距離是否可用於控制。未標定時為 NaN，任何算術都會傳染，不會靜默出錯。 */
    public boolean hasDistance() {
        return calibrated && !Double.isNaN(distanceMeters);
    }

    /** 這一幀從曝光到現在經過多久（秒）。 */
    public double ageSeconds(double nowFpgaSeconds) {
        return nowFpgaSeconds - captureTimestampSeconds;
    }

    /** NT4 斷線或尚未收到任何資料時的佔位值。 */
    public static BallVisionMeasurement unavailable(Validity why) {
        return new BallVisionMeasurement(
                false, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0,
                0.0, 0.0, 0.0, Double.NaN, Double.NaN, 0.0, 0.0,
                0L, "unknown", false, why);
    }
}
