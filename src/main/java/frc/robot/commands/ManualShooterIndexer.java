package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ManualShooterIndexer extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private final XboxController driverController, operatorController;

    private double targetAngle = 0, targetUpSpeed = 0, targetDownSpeed = 0;
    
    public ManualShooterIndexer(
        Shooter shooter,
        Indexer indexer,
        XboxController driverController,
        XboxController operatorController
    ) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.driverController = driverController;
        this.operatorController = operatorController;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (operatorController.getRightTriggerAxis() > 0.5) {
            targetAngle += 0.0001;
        }
        if (operatorController.getLeftTriggerAxis() > 0.5) {
            targetAngle -= 0.0001;
        }

        targetAngle = MathUtil.clamp(targetAngle, ShooterConstants.kAngleMin, ShooterConstants.kAngleMax);

        targetUpSpeed = operatorController.getRightY() * 32;
        targetDownSpeed = operatorController.getLeftY() * 52;

        shooter.setAngle(targetAngle);
        shooter.setSpeed(targetUpSpeed, targetDownSpeed);

        if (driverController.getRightTriggerAxis() > 0.5) {
            indexer.set(1.0, 1.0);
        } else {
            indexer.set(0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setSpeed(0.0, 0.0);
        indexer.set(0.0, 0.0);
    }
}
