package frc.robot.helpers;

import java.util.*;

//The path is simply a queue of points to travel to.
//It implements the Queue interface.
public class Path implements Queue<Position> {

  private ArrayList<Position> path;

  public Path() {
    path = new ArrayList<>();
  }

  public Path(ArrayList<Position> pointsInOrder) {
    path = pointsInOrder;
  }

  @Override
  public int size() {
    return path.size();
  }

  @Override
  public boolean isEmpty() {
    return path.isEmpty();
  }

  @Override
  public boolean contains(Object o) {
    return path.contains(o);
  }

  @Override
  public Iterator<Position> iterator() {
    return path.iterator();
  }

  @Override
  public Object[] toArray() {
    return path.toArray();
  }

  @Override
  public <T> T[] toArray(T[] a) {
    return a;
  }

  @Override
  public boolean add(Position position) {
    return path.add(position);
  }

  @Override
  public boolean remove(Object o) {
    return path.remove(o);
  }

  @Override
  public boolean containsAll(Collection<?> c) {
    return path.containsAll(c);
  }

  @Override
  public boolean addAll(Collection<? extends Position> c) {
    return path.addAll(c);
  }

  @Override
  public boolean removeAll(Collection<?> c) {
    return path.removeAll(c);
  }

  @Override
  public boolean retainAll(Collection<?> c) {
    return path.retainAll(c);
  }

  @Override
  public void clear() {
    path.clear();
  }

  @Override
  public boolean offer(Position position) {
    return path.add(position);
  }

  @Override
  public Position remove() {

    if (path.size() == 0) {
      throw new NoSuchElementException("Position Queue is Empty");
    } else {
      Position temp = path.get(0);
      path.remove(0);
      return temp;
    }
  }

  @Override
  public Position poll() {
    if (path.size() == 0) {
      return null;
    } else {
      Position temp = path.get(0);
      path.remove(0);
      return temp;
    }
  }

  @Override
  public Position element() {
    if (path.size() == 0) {
      throw new NoSuchElementException("Position Queue is Empty");
    } else {
      Position temp = path.get(0);
      return temp;
    }
  }

  @Override
  public Position peek() {
    if (path.size() == 0) {
      return null;
    } else {
      Position temp = path.get(0);
      return temp;
    }
  }

  public double getPathEuclideanDistance() {

    double sum = 0;

    Position previousPoint = this.element();

    for (Position p : path) {
      sum = sum + previousPoint.euclideanDistance2D(p);
      previousPoint = p;
    }

    return sum;
  }

  public double getPathManhattanDistance() {

    double sum = 0;

    Position previousPoint = this.element();

    for (Position p : path) {
      sum = sum + previousPoint.manhattanDistance2D(p);
      previousPoint = p;
    }

    return sum;
  }
}
