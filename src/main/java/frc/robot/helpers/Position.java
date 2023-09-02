package frc.robot.helpers;

public class Position {

  private int x;

  private int y;

  private boolean is2D;

  private int z;

  public Position(int x, int y, int z) {
    this.x = x;
    this.y = y;
    this.z = z;
    is2D = false;
  }

  public Position(int x, int y) {
    this.x = x;
    this.y = y;
    this.z = 0;
    is2D = true;
  }

  boolean compareXY(int x, int y) {

    if (this.x == x && this.y == y) {
      return true;
    }
    return false;
  }

  boolean compareXYZ(int x, int y, int z) {

    if (this.x == x && this.y == y && this.z == z) {
      return true;
    }
    return false;
  }

  boolean compareX(int x) {
    if (this.x == x) {
      return true;
    }
    return false;
  }

  boolean compareY(int y) {
    if (this.y == y) {
      return true;
    }
    return false;
  }

  boolean compareZ(int z) {
    if (this.z == z) {
      return true;
    }
    return false;
  }

  public int getX() {
    return x;
  }

  public void setX(int x) {
    this.x = x;
  }

  public int getY() {
    return y;
  }

  public void setY(int y) {
    this.y = y;
  }

  public int getZ() {
    return z;
  }

  public void setZ(int z) {
    this.z = z;
    is2D = false;
  }

  public void set2D(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public void set3D(int x, int y, int z) {
    this.x = x;
    this.y = y;
    this.z = z;
    is2D = false;
  }

  public void translate2D(int x, int y) {
    this.x = this.x + x;
    this.y = this.y + y;
  }

  public void translate3D(int x, int y, int z) {
    this.x = this.x + x;
    this.y = this.y + y;
    this.z = this.z + z;
  }

  public void translate2DDeg(double theta, int dist) {

    theta = Math.toRadians(theta);

    double sinTheta = Math.sin(theta);
    double cosTheta = Math.cos(theta);

    this.x = this.x + (int) (sinTheta * dist);
    this.y = this.y + (int) (cosTheta * dist);
  }

  public void translate2DRad(double theta, int dist) {

    double sinTheta = Math.sin(theta);
    double cosTheta = Math.cos(theta);

    this.x = this.x + (int) (sinTheta * dist);
    this.y = this.y + (int) (cosTheta * dist);
  }

  public void translate3DDeg(double theta, double phi, int dist) {

    theta = Math.toRadians(theta);
    phi = Math.toRadians(phi);

    double sinTheta = Math.sin(theta);
    double cosTheta = Math.cos(theta);

    double sinPhi = Math.sin(phi);
    double cosPhi = Math.cos(phi);

    this.x = this.x + (int) (dist * sinPhi * cosTheta);
    this.y = this.y + (int) (dist * sinTheta * sinPhi);
    this.z = this.z + (int) (dist * cosPhi);
  }

  public void translate3DRad(double theta, double phi, int dist) {

    double sinTheta = Math.sin(theta);
    double cosTheta = Math.cos(theta);

    double sinPhi = Math.sin(phi);
    double cosPhi = Math.cos(phi);

    this.x = this.x + (int) (dist * sinPhi * cosTheta);
    this.y = this.y + (int) (dist * sinTheta * sinPhi);
    this.z = this.z + (int) (dist * cosPhi);
  }

  //Direct distance to a point
  public double euclideanDistance2D(Position p) {
    int diffX = this.x - p.x;
    int diffY = this.y - p.y;
    double diffX2 = diffX * diffX;
    double diffY2 = diffY * diffY;
    return Math.sqrt(diffX2 + diffY2);
  }

  public double euclideanDistance3D(Position p) {
    int diffX = this.x - p.x;
    int diffY = this.y - p.y;
    int diffZ = this.z - p.z;
    double diffX2 = diffX * diffX;
    double diffY2 = diffY * diffY;
    double diffZ2 = diffZ * diffZ;
    return Math.sqrt(diffX2 + diffY2 + diffZ2);
  }

  //Distance to a point following a grid
  public int manhattanDistance2D(Position p) {
    int diffX = this.x - p.x;
    int diffY = this.y - p.y;
    return diffX + diffY;
  }

  public int manhattanDistance3D(Position p) {
    int diffX = this.x - p.x;
    int diffY = this.y - p.y;
    int diffZ = this.z - p.z;
    return diffX + diffY + diffZ;
  }

  public double euclideanDistance2D(int x, int y) {
    Position p = new Position(x, y);
    return this.euclideanDistance2D(p);
  }

  public double euclideanDistance3D(int x, int y, int z) {
    Position p = new Position(x, y, z);
    return this.euclideanDistance3D(p);
  }

  public double manhattanDistance2D(int x, int y) {
    Position p = new Position(x, y);
    return this.manhattanDistance2D(p);
  }

  public double manhattanDistance3D(int x, int y, int z) {
    Position p = new Position(x, y, z);
    return this.manhattanDistance3D(p);
  }

  public double angle2DDeg(Position p) {
    return Math.toDegrees(this.angle2DRad(p));
  }

  public double angle2DDeg(int x, int y) {
    Position p = new Position(x, y);
    return this.angle2DDeg(p);
  }

  public double angle2DRad(Position p) {

    int diffX = this.x - p.x;
    int diffY = this.y - p.y;

    if (diffX == 0) {

      if (diffY == 0) {
        return -1;
      }

      int sign = Math.abs(diffY) / diffY;

      if (sign == 1) {
        return Math.toRadians(270);
      }
      if (sign == -1) {
        return Math.toRadians(90);
      }
    }

    if (diffY == 0) {

      if (diffX == 0) {
        return -1;
      }

      int sign = Math.abs(diffX) / diffX;

      if (sign == 1) {
        return Math.toRadians(180);
      }
      if (sign == -1) {
        return Math.toRadians(0);
      }
    }

    double quotient = diffY / diffX;

    return Math.atan(quotient);
  }

  public double angle2DRad(int x, int y) {
    Position p = new Position(x, y);
    return this.angle2DRad(p);
  }

  /**
   * Takes the current position and calculates the polar equivalent Takes an offset of the polar
   * coordinates to match the result with the arena 0 Degrees is aligned with the y axis
   *
   * @param t Target position that we are going to
   * @return Polar angle in degrees equivalent of the input cartesian positions minus an offset of
   *     π/2
   */
  public double cartesianToPolarDegrees(Position t) {
    double y = -1 * t.y - (-1) * this.y;
    double x = t.x - this.x;
    if (y == 0 && x == 0) return 0;
    // return Math.toDegrees(Math.atan2(y, x) - (Math.PI / 2));
    return Math.toDegrees(Math.atan2(y, x));

    // double theta = Math.atan2(y - t.y, x - t.x);
    // if (theta < 0.0)
    // {
    //     theta += Math.PI * 2;
    // }
    // theta -= Math.PI;
    // return positiveDegrees(Math.toDegrees(theta));

  }

  public double cartesianToPolarDistance(Position t) {
    double y = t.y - this.y;
    double x = t.x - this.x;
    return Math.sqrt((x * x) + (y * y));
  }

  @Override
  public String toString() {
    String str;

    if (is2D) {
      str = String.format("(X:%d,Y:%d)", this.x, this.y);
    } else {
      str = String.format("(X:%d,Y:%d,Z:%d)", this.x, this.y, this.z);
    }

    return str;
  }

  public void print() {
    System.out.println(this.toString());
  }

  /** Returns a positive, normalized angle */
  public static double positiveDegrees(double angle) {
    angle = angle % 360;
    if (angle < 0) {
      angle += 360;
    }
    return Math.abs(angle);
  }
}
