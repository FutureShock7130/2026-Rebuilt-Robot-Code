package frc.FSLib.statemachine;

import java.util.List;
import java.util.Map;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class StateMachine<T extends Enum<T>> {
  private final StateGraph<T> m_graph = new StateGraph<>();
  private final StateCommandFactory<T> m_factory;

  private T m_currentState;
  private T m_nextState;
  private T m_goalState;
  private Stack<T> m_path;

  private Command m_edgeCommand;
  private Command m_stayCommand;

  public StateMachine(StateSubsystemBase<T> subsytem) {
    m_factory = new StateCommandFactory<>(subsytem);
    m_edgeCommand = Commands.none();
    m_stayCommand = Commands.none();
  }

  public void create(Map<T, List<T>> adjList, T initialState) {
    m_graph.create(adjList);

    m_currentState = initialState;
    m_nextState = initialState;
    m_goalState = initialState;
    m_path = new Stack<>();
  }

  public T getCurrentState() {
    return m_currentState;
  }

  public T getNextState() {
    return m_nextState;
  }

  public T getGoalState() {
    return m_goalState;
  }

  public boolean atGoalState() {
    return m_currentState == m_goalState;
  }

  public void setGoalState(T newGoal) {
    if (newGoal == m_goalState) return;
    m_goalState = newGoal;
    
    Stack<T> pathFromCurrent = m_graph.findPath(m_currentState, m_goalState);
    Stack<T> pathFromNext = m_graph.findPath(m_nextState, m_goalState);
    m_path = pathFromCurrent.size() <= pathFromNext.size() ? pathFromCurrent : pathFromNext;

    T newNext = m_path.pop();
    if (newNext != m_nextState) {
      m_edgeCommand.cancel();
      m_edgeCommand = m_factory.build(newNext);
      CommandScheduler.getInstance().schedule(m_edgeCommand);
      m_nextState = newNext;
    }
  }

  public void periodic() {
    if (!m_edgeCommand.isScheduled()) {
      m_currentState = m_nextState;
      if (m_path.isEmpty()) {
        if (!m_stayCommand.isScheduled()) {
          m_stayCommand = m_factory.stay(m_currentState);
          CommandScheduler.getInstance().schedule(m_stayCommand);
        }
      } else {
        if (m_stayCommand.isScheduled()) {
          m_stayCommand.cancel();
        }
        m_nextState = m_path.pop();
        m_edgeCommand = m_factory.build(m_nextState);
        CommandScheduler.getInstance().schedule(m_edgeCommand);
      }
    } 
  }
}
