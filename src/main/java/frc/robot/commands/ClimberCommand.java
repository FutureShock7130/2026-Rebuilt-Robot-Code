package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommand extends Command{
   private final ClimberSubsystem climberSubsystem;
   private final double climberTarget;

   public ClimberCommand(ClimberSubsystem climberSubsystem, double climberPosition) {
       this.climberSubsystem = climberSubsystem;
       this.climberTarget = climberPosition;
   }
   public void initialize() {
     
        climberSubsystem.setClimberTargetPosition(climberTarget);
   }
    public void execute() {
          climberSubsystem.runClimber();
          SmartDashboard.putNumber("climberMotor", climberSubsystem.getClimberPosition());
          SmartDashboard.putNumber("climberTarget", climberTarget);
    }
    public void end(boolean interrupted) {
          climberSubsystem.resetClimberPosition();
    }
}
