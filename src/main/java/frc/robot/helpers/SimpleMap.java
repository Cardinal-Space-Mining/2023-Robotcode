package frc.robot.helpers;

// import frc.robot.Constants;
import java.util.*;

class Point2D implements Comparable<Point2D> {
  int x;
  int y;
  int cost;
  int costFromSrc = 999;
  boolean visited = false;
  Point2D parent;

  public Point2D(int x, int y, int cost) {
    this.x = x;
    this.y = y;
    this.cost = cost;
  }

  @Override
  public int compareTo(Point2D o) {
    if (this.cost < o.cost) {
      return -1;
    } else if (this.cost > o.cost) {
      return 1;
    } else {
      return 0;
    }
  }
}

public class SimpleMap {
  private Point2D[][] map =
      new Point2D[5][5]; // [Constants.ARENA_WIDTH + 1][Constants.ARENA_HEIGHT + 1];
  private Position currRobotPos;
  private Position destination;
  private ArrayList<Position> shortestPath = new ArrayList<Position>();
  private int halfArenaWidth = 2; // Constants.ARENA_WIDTH / 2;

  public SimpleMap() {
    // for (int i = 0; i <= Constants.ARENA_HEIGHT; i++) {
    // int colIndex = halfArenaWidth;
    // for (int j = 0; j <= Constants.ARENA_WIDTH; j++) {
    // map[j][i] = new Point2D(colIndex, i, Math.abs(colIndex));
    // colIndex -= 1;
    // }
    // }
    for (int i = 0; i < 5; i++) {
      int colIndex = -halfArenaWidth;
      for (int j = 0; j < 5; j++) {
        map[j][i] = new Point2D(colIndex, i, Math.abs(colIndex));
        colIndex += 1;
      }
    }
  }

  public Position currentPosition() {
    return currRobotPos;
  }

  public Position destinationPosition() {
    return destination;
  }

  public void printMap() {
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 5; j++) {
        System.out.print(map[i][j].cost + " ");
      }
      System.out.println();
    }
  }

  public void addObjectPoint(Position p) {
    map[p.getX() + halfArenaWidth][p.getY()].cost = 999;
  }

  public void updateRobotsPos(Position p) {
    currRobotPos = new Position(p.getX() + halfArenaWidth, p.getY());
  }

  public void updateDestination(Position p) {
    destination = new Position(p.getX() + halfArenaWidth, p.getY());
  }

  public ArrayList<Position> getShortestPath() {
    return shortestPath;
  }

  public void findShortestPath(Position src, Position dest) {
    PriorityQueue<Point2D> q = new PriorityQueue<>();

    map[src.getX() + halfArenaWidth][src.getY()].costFromSrc = 0;
    map[src.getX() + halfArenaWidth][src.getY()].parent = null;
    q.add(map[src.getX() + halfArenaWidth][src.getY()]);

    while (!q.isEmpty()) {
      Point2D point = q.remove();
      int x = point.x + halfArenaWidth;
      int y = point.y;
      int newcost;
      map[x][y].visited = true;

      if ((dest.getX() + halfArenaWidth) == x && dest.getY() == y) {
        Point2D curPoint = map[x][y];
        while (curPoint.parent != null) {
          System.out.println("(" + curPoint.x + ", " + curPoint.y + ")");
          curPoint = curPoint.parent;
        }
        System.out.println("(" + curPoint.x + ", " + curPoint.y + ")");
      }

      if (((y - 1) >= 0) && (x < 5) && !map[x][y - 1].visited) {
        newcost = map[x][y].costFromSrc + map[x][y - 1].cost;
        if (newcost < map[x][y - 1].costFromSrc) {
          map[x][y - 1].costFromSrc = newcost;
          map[x][y - 1].parent = map[x][y];
          q.add(map[x][y - 1]);
        }
      }
      if (((y + 1) < 5) && (x < 5) && !map[x][y + 1].visited) {
        newcost = map[x][y].costFromSrc + map[x][y + 1].cost;
        if (newcost < map[x][y + 1].costFromSrc) {
          map[x][y + 1].costFromSrc = newcost;
          map[x][y + 1].parent = map[x][y];
          q.add(map[x][y + 1]);
        }
      }
      if ((y < 5) && ((x - 1) >= 0) && !map[x - 1][y].visited) {
        newcost = map[x][y].costFromSrc + map[x - 1][y].cost;
        if (newcost < map[x - 1][y].costFromSrc) {
          map[x - 1][y].costFromSrc = newcost;
          map[x - 1][y].parent = map[x][y];
          q.add(map[x - 1][y]);
        }
      }
      if ((y < 5) && ((x + 1) < 5) && !map[x + 1][y].visited) {
        newcost = map[x][y].costFromSrc + map[x + 1][y].cost;
        if (newcost < map[x + 1][y].costFromSrc) {
          map[x + 1][y].costFromSrc = newcost;
          map[x + 1][y].parent = map[x][y];
          q.add(map[x + 1][y]);
        }
      }
    }
  }
}
