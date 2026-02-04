package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberCommand {
   private final ClimberSubsystem climberSubsystem;
   private final double climberPosition;

   public ClimberCommand(ClimberSubsystem climberSubsystem, double climberPosition) {
       this.climberSubsystem = climberSubsystem;
       this.climberPosition = climberPosition;
   }
   public void initialize() {
        climberSubsystem.resetClimberPosition();
        climberSubsystem.setClimberTargetPosition(climberPosition);
   }
    public void execute() {
          climberSubsystem.runClimber();
          SmartDashboard.putNumber("climberMotor", climberSubsystem.getClimberPosition());
          SmartDashboard.putNumber("climberTarget", climberPosition);
    }
    public void end(boolean interrupted) {
          climberSubsystem.resetClimberPosition();
    }

}
