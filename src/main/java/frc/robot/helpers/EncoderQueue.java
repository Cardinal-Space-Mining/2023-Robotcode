package frc.robot.helpers;

import java.util.*;

public class EncoderQueue implements Queue<Double> {

  private ArrayList<Double> posQueue;

  public EncoderQueue() {
    posQueue = new ArrayList<>();
  }

  public EncoderQueue(ArrayList<Double> posQueue) {
    this.posQueue = posQueue;
  }

  @Override
  public int size() {
    return posQueue.size();
  }

  @Override
  public boolean isEmpty() {
    return posQueue.isEmpty();
  }

  @Override
  public boolean contains(Object o) {
    return posQueue.contains(o);
  }

  @Override
  public Iterator<Double> iterator() {
    return posQueue.iterator();
  }

  @Override
  public Object[] toArray() {
    return posQueue.toArray();
  }

  @Override
  public <T> T[] toArray(T[] a) {
    return posQueue.toArray(a);
  }

  @Override
  public boolean add(Double Double) {
    return posQueue.add(Double);
  }

  @Override
  public boolean remove(Object o) {
    return posQueue.remove(o);
  }

  @Override
  public boolean containsAll(Collection<?> c) {
    return posQueue.containsAll(c);
  }

  @Override
  public boolean addAll(Collection<? extends Double> c) {
    return posQueue.addAll(c);
  }

  @Override
  public boolean removeAll(Collection<?> c) {
    return posQueue.removeAll(c);
  }

  @Override
  public boolean retainAll(Collection<?> c) {
    return posQueue.retainAll(c);
  }

  @Override
  public void clear() {
    posQueue.clear();
  }

  @Override
  public boolean equals(Object o) {
    return posQueue.equals(o);
  }

  @Override
  public int hashCode() {
    return posQueue.hashCode();
  }

  @Override
  public boolean offer(Double Double) {
    return posQueue.add(Double);
  }

  @Override
  public Double remove() {
    if (posQueue.isEmpty()) throw new NoSuchElementException("Queue is empty");

    double temp = posQueue.get(0);
    posQueue.remove(0);

    return temp;
  }

  @Override
  public Double poll() {
    if (posQueue.isEmpty()) return null;

    double temp = posQueue.get(0);
    posQueue.remove(0);

    return temp;
  }

  @Override
  public Double element() {
    if (posQueue.isEmpty()) throw new NoSuchElementException("Queue is empty");

    return posQueue.get(0);
  }

  @Override
  public Double peek() {
    if (posQueue.isEmpty()) return null;

    return posQueue.get(0);
  }
}
