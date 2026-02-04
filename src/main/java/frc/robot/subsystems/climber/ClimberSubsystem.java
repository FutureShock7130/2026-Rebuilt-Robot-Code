package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimberSubsystem {
    private TalonFX leftMotor = new TalonFX(ClimberConstants.leftMotorID,"GTX7130");
    private TalonFX rightMotor = new TalonFX(ClimberConstants.rightMotorID,"GTX7130");

    private PIDController climberPID = new PIDController(ClimberConstants.climberKP, ClimberConstants.climberKI, ClimberConstants.climberKD);

    public ClimberSubsystem() {
   
    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfigs.Inverted =  InvertedValue.Clockwise_Positive;
    leftMotor.setControl(new Follower(ClimberConstants.leftMotorID , MotorAlignmentValue.Aligned));

    // TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // motorConfig.MotorOutput
    //     .withInverted(InvertedValue.CounterClockwise_Positive)
    
    }
    public double getClimberPosition() {
        return (leftMotor.getPosition().getValueAsDouble()+rightMotor.getPosition().getValueAsDouble())/2;
        

    }
    public void resetClimberPosition() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
    public void setClimberTargetPosition(double position) {
        climberPID.setSetpoint(position);
    }
    public void runClimber() {
        leftMotor.setVoltage(climberPID.calculate((leftMotor.getPosition().getValueAsDouble()
        + rightMotor.getPosition().getValueAsDouble())/2));
    }
}
