package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;

public class ClimberConstants {
    public static final CANBus climberBus = new CANBus("GTX7130");
    public static final int leftMotorID = 20;
    public static final int rightMotorID = 21;

    public static final double climberKP = 0.1;
    public static final double climberKI = 0.0; 
    public static final double climberKD = 0.0; 
}