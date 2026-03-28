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
    private final LEDPattern shootingPattern = readyPattern.blink(Seconds.of(0.125));
    private final LEDPattern intakingPattern = defaultPattern.blink(Seconds.of(0.125));
    private LEDPattern patternToApply = LEDPattern.kOff;

    private LEDCenter() {
        led.setLength(buffer.getLength());
    }

    public static LEDCenter getInstance() {
        return instance;
    }

    public void setDefault() {
        patternToApply = defaultPattern;
    }

    public Command setDefaultState() {
        return Commands.runOnce(this::setDefault);
    }

    public void setAiming() {
        patternToApply = readyPattern;
    }

    public Command setAimingState() {
        return Commands.runOnce(this::setAiming);
    }

    public void setOff() {
        patternToApply = LEDPattern.kOff;
    }

    public Command setOffState() {
        return Commands.runOnce(this::setOff);
    }

    public void setShooting() {
        patternToApply = shootingPattern;
    }

    public Command setShootingState() {
        return Commands.runOnce(this::setShooting);
    }

    public void setIntaking() {
        patternToApply = shootingPattern;
    }

    public Command setIntakingState() {
        return Commands.runOnce(this::setIntaking);
    }
    
    public void update() {
        patternToApply.applyTo(buffer);
        led.setData(buffer);
    }
}
