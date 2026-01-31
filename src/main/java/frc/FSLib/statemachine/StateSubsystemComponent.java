package frc.FSLib.statemachine;

public interface StateSubsystemComponent {
  default void periodic() {}
  default void simulationPeriodic() {}
}
