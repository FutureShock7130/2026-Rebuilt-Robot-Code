// inspired by frc team 6328 Machincal Advantage 2025 season build thread
package frc.FSLib.statemachine;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

import edu.wpi.first.wpilibj.DriverStation;

public class StateGraph<T extends Enum<T>> {
  // Adjacency list representation using a Map
  private final Map<T, List<T>> adjList = new HashMap<>();

  public void create(Map<T, List<T>> adjList) {
    for (Map.Entry<T, List<T>> entry : adjList.entrySet()) {
      this.adjList.put(entry.getKey(), entry.getValue());
    }
  }

  public boolean edgeExsist(T start, T end) {
    if (!adjList.containsKey(start)) {
      return false;
    }

    return adjList.get(start).contains(end);
  }

  // Find path from start vertex to end vertex using BFS
  public Stack<T> findPath(T start, T end) {
    // Check if vertices exist
    if (!adjList.containsKey(start) || !adjList.containsKey(end)) {
      DriverStation.reportError(
        "StateGraph attempts to find a path between "
          + start.name()
          + " and "
          + end.name()
          + ", which does not exsist in the adjaceny list", false);
      return new Stack<>(); // Return empty list if vertices don't exist
    }

    // Map to store the parent of each vertex in the path
    Map<T, T> parent = new HashMap<>();
    // Queue for BFS
    Queue<T> queue = new LinkedList<>();
    // Set to keep track of visited vertices
    Set<T> visited = new HashSet<>();

    // Start BFS
    queue.add(start);
    visited.add(start);
    parent.put(start, null);

    while (!queue.isEmpty()) {
      T current = queue.poll();

      // If we found the end vertex, construct and return the path
      if (current.equals(end)) {
        return constructPath(start, end, parent);
      }

      // Visit all neighbors
      for (T neighbor : adjList.get(current)) {
        if (!visited.contains(neighbor)) {
          visited.add(neighbor);
          queue.add(neighbor);
          parent.put(neighbor, current);
        }
      }
    }

    // No path found
    DriverStation.reportError(
        "Can not find a path between "
          + start.name()
          + " and "
          + end.name(), false);
    return new Stack<>();
  }

  // Helper method to construct the path from parent map
  private Stack<T> constructPath(T start, T end, Map<T, T> parent) {
    Stack<T> path = new Stack<>();
    T current = end;

    while (current != null) {
      path.add(current);
      current = parent.get(current);
    }

    return path;
  }
}
