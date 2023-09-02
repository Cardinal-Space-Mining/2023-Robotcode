package frc.robot.helpers;

// import frc.robot.Constants;

public class test {
  public static void main(String[] args) {
    SimpleMap sm = new SimpleMap();
    Position robotPos = new Position(0, 0);
    Position destination = new Position(0, 4);
    // sm.addObjectPoint(new Position(0, 2));
    sm.addObjectPoint(new Position(0, 3));
    sm.addObjectPoint(new Position(1, 3));
    sm.addObjectPoint(new Position(-1, 3));
    sm.addObjectPoint(new Position(0, 2));
    sm.addObjectPoint(new Position(-1, 2));
    sm.addObjectPoint(new Position(0, 1));
    sm.printMap();
    sm.findShortestPath(robotPos, destination);
    System.out.println("Done");
  }
}
