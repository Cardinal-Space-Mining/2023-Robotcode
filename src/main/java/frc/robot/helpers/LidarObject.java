package frc.robot.helpers;

public class LidarObject {

  private int dist;
  private int angle;
  private int width;

  public LidarObject(int dist, int angle, int width) {
    this.dist = dist;
    this.angle = angle;
    this.width = width;
  }

  public int getObjectDist() {
    return dist;
  }

  public int getObjectAngle() {
    return angle;
  }

  public int getObjectWidth() {
    return width;
  }
}
