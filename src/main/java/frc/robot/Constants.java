package frc.robot;

public final class Constants {
  public static class IntakeConstants {
    public static final int INTAKE_PORT = 1;
    public static final int INTAKEPIVOT_PORT = 5;
  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_PORT = 3;
    public static final int TOP_LS_PORT = 1;
    public static final int BOTTOM_LS_PORT = 2;
    public static final double SPEED_CAP = 0.5;
    public static final double TOP_ENC_LIMIT = 50;
    public static final double BOTTOM_ENC_LIMIT = -50;
    public static final double MANUAL_SPEED = 0.2;
  }
}