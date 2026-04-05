package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDCenter extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(9);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(22);

    private final LEDPattern defaultPattern = LEDPattern.solid(Color.kLightBlue);
    private final LEDPattern manualModePattern = LEDPattern.solid(Color.kFirstRed);
    private final LEDPattern testModePattern = LEDPattern.solid(Color.kYellow);
    private final LEDPattern readyPattern = LEDPattern.solid(Color.kGreen);
    private final LEDPattern shootingPattern = readyPattern.blink(Seconds.of(0.25));
    private final LEDPattern intakingPattern = defaultPattern.blink(Seconds.of(0.25));
    
    private XboxController driverController;

    public LEDCenter() {
        led.setLength(buffer.getLength());
        led.start();
    }

    public void initialize(XboxController driverController) {
        this.driverController = driverController;
    }

    public void setDefault() {
        defaultPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setManual() {
        manualModePattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setTest() {
        testModePattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setAiming() {
        readyPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setOff() {
        LEDPattern.kOff.applyTo(buffer);
        led.setData(buffer);
    }

    public void setShooting() {
        shootingPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setIntaking() {
        intakingPattern.applyTo(buffer);
        led.setData(buffer);
    }

    private int counter = 0;
    public void setLarsonScanner() {
        counter++;
        int step = counter % 42;
        int eyePosition = step < 22 ? step : 42 - step;
        for (int i = 0; i < 22; i++) {
            int distance = Math.abs(eyePosition - i);
            double fadeFactor = Math.max(0.0, 1.0 - (distance * 0.3)); 
            buffer.setRGB(i, (int)(173 * fadeFactor), (int)(216 * fadeFactor), (int)(230 * fadeFactor));
        }
        led.setData(buffer);
    }

    private boolean isTestMode = false, isManualMode = false;

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setLarsonScanner();
        } else if (driverController.getRightBumperButton()) {
            if (driverController.getRightTriggerAxis() > 0.5) {
                setShooting();
            } else {
                setAiming();
            }
        } else if (driverController.getLeftTriggerAxis() > 0.5) {
            setIntaking();
        } else {
            if (isTestMode) {
                setTest();
            } else if (isManualMode) {
                setManual();
            } else {
                setDefault();
            }
        }
    }
}
