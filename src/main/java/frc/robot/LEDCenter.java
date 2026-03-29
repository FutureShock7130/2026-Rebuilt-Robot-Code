package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LEDCenter {
    private static final LEDCenter instance = new LEDCenter();

    private final AddressableLED led = new AddressableLED(9);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(22);

    private final LEDPattern defaultPattern = LEDPattern.solid(Color.kLightBlue);
    private final LEDPattern readyPattern = LEDPattern.solid(Color.kGreen);
    private final LEDPattern shootingPattern = readyPattern.blink(Seconds.of(0.25));
    private final LEDPattern intakingPattern = defaultPattern.blink(Seconds.of(0.25));

    private LEDCenter() {
        led.setLength(buffer.getLength());
        led.start();
    }

    public static LEDCenter getInstance() {
        return instance;
    }

    public void setDefault() {
        defaultPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public Command setDefaultState() {
        return Commands.runOnce(this::setDefault);
    }

    public void setAiming() {
        readyPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public Command setAimingState() {
        return Commands.runOnce(this::setAiming);
    }

    public void setOff() {
        LEDPattern.kOff.applyTo(buffer);
        led.setData(buffer);
    }

    public Command setOffState() {
        return Commands.runOnce(this::setOff);
    }

    public void setShooting() {
        shootingPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public Command setShootingState() {
        return Commands.runOnce(this::setShooting);
    }

    public void setIntaking() {
        intakingPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public Command setIntakingState() {
        return Commands.runOnce(this::setIntaking);
    }

    private int counter = 0;
    public void larsonScanner() {
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
}
