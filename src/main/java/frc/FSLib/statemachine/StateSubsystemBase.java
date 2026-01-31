package frc.FSLib.statemachine;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StateSubsystemBase<T extends Enum<T>> implements Subsystem, Sendable {
  private final Set<StateSubsystemComponent> m_components;
  private final StateMachine<T> m_machine;

  @SuppressWarnings("this-escape")
  public StateSubsystemBase() {
    m_components = new HashSet<>();
    m_machine = new StateMachine<>(this);

    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.add(this, name);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public final void addComponents(StateSubsystemComponent ...components) {
    for (StateSubsystemComponent component : components) {
      m_components.add(requireNonNullParam(component, "components", "addComponent"));
    }
  }

  public final void createFSM(Map<T, List<T>> adjList, T initialState) {
    m_machine.create(
      adjList,
      requireNonNullParam(initialState, "initialState", "createFSM")
    );
  }

  public final T getCurrentState() {
    return m_machine.getCurrentState();
  }

  public final T getNextState() {
    return m_machine.getNextState();
  }

  public final T getGoalState() {
    return m_machine.getGoalState();
  }
  
  public final void setGoalState(T newGoal) {
    m_machine.setGoalState(newGoal);
  }

  public final Command setGoalStateCommand(T newGoal) {
    return Commands.defer(
      () -> Commands.runOnce(() -> setGoalState(newGoal)).andThen(Commands.waitUntil(() -> m_machine.atGoalState())), Set.of(this)
    );
  }

  protected abstract void transitionAction(T state);
  protected abstract boolean isAtState(T state);
  protected abstract void onEndBehavior(T state);

  @Override
  public final void periodic() {
    for (StateSubsystemComponent component : m_components) {
      component.periodic();
    }
    m_machine.periodic();
  }

  @Override
  public final void simulationPeriodic() {
    for (StateSubsystemComponent component : m_components) {
      component.simulationPeriodic();
    }
  }

  @Override
  public final void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("StateSubsystem");

    builder.addBooleanProperty("AtGoalState", () -> m_machine.atGoalState(), null);
    builder.addStringProperty("CurrentState", () -> getCurrentState().name(), null);
    builder.addStringProperty("NextState", () -> getNextState().name(), null);
    builder.addStringProperty("GoalState", () -> getGoalState().name(), null);
  }
}
