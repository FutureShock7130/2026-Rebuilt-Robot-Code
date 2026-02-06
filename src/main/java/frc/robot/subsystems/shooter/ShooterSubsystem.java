package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    CANBus shooterUpCanBus = new CANBus("GTX7130");
    TalonFX shooterUp = new TalonFX(ShooterConstants.shooterUpID,shooterUpCanBus);
    CANBus shooterDownCanBus = new CANBus("GTX7130");
    TalonFX shooterDown = new TalonFX(ShooterConstants.shooterDownID,shooterDownCanBus);
    CANBus shooterAngleCanBus = new CANBus("GTX7130");
    TalonFX shooterAngle = new TalonFX(ShooterConstants.shooterAngleID, shooterAngleCanBus);
    PIDController shooterAnglePID = new PIDController(
        ShooterConstants.anglekP,
        ShooterConstants.anglekI,
        ShooterConstants.anglekD
    );
    double shooterAngleTarget = 0.0;
    MotionMagicVelocityVoltage shooterVelocityControl = new MotionMagicVelocityVoltage(0);
    public ShooterSubsystem(){
        var shooterUpConfig = new TalonFXConfiguration();
        shooterUp.getConfigurator().apply(shooterUpConfig);
        var slot0Config = shooterUpConfig.Slot0;
        slot0Config.kS = ShooterConstants.Slot0kS;
        slot0Config.kV = ShooterConstants.Slot0kV;
        slot0Config.kP = ShooterConstants.Slot0kP;
        var motationConfigUp = shooterUpConfig.MotionMagic;
        motationConfigUp.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationUp;

        var shooterDownConfig = new TalonFXConfiguration();
        shooterDown.getConfigurator().apply(shooterDownConfig);
        var slot1Config = shooterDownConfig.Slot1;
        slot1Config.kS = ShooterConstants.Slot1kS;
        slot1Config.kV = ShooterConstants.Slot1kV;
        slot1Config.kP = ShooterConstants.Slot1kP;
        var motationConfigDown = shooterDownConfig.MotionMagic;
        motationConfigDown.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationDown;
    }
    public double getShooterAnglePos(){
        return (shooterAngle.getPosition().getValueAsDouble());
    }
    public void setShooterTarget(double angle){
        shooterAngleTarget = angle;
    }
    public void setShooterAngle(){
        double setShooterAngle = (shooterAnglePID.calculate(getShooterAnglePos(), shooterAngleTarget));
        shooterAngle.set(setShooterAngle);
    }
    public void setShooterUpSpeed(double upSpeedRPM){
        // shooterUp.set(speed);
        double upSpeedRPS = upSpeedRPM/60;
        shooterUp.setControl(shooterVelocityControl.withVelocity(upSpeedRPS));
    }
    public void setShooterDownSpeed(double downSpeedRPM){
        double downSpeedRPS = downSpeedRPM/60;
        shooterDown.setControl(shooterVelocityControl.withVelocity(downSpeedRPS));
    }
    public void stop(){
        shooterUp.set(0);
        shooterAngle.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterPos", getShooterAnglePos());
    }
}

