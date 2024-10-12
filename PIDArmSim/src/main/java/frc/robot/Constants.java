package frc.robot;

public class Constants {
    public static final boolean tuningMode = true;

    // feedforward constants - determined with SysId
    public static final double defkV = 0.11934;
    public static final double defkS = 0.0032289;
    public static final double defkA = 0.10627;
    public static final double defkG = 0.00022342;

    // position PID constants
    public static final double defkPPos = 44.328;
    public static final double defkIPos = 0.0;
    public static final double defkDPos = 2.987;
    public static final int POSITION_GAIN_SLOT = 0;

    // velocity constants
    public static final double defkPVel = 0.1227;
    public static final double defkIVel = 0.0;
    public static final double defkDVel = 0.0;
    public static final int VELOCITY_GAIN_SLOT = 1;

    public static final double FALCON_500_FREE_SPEED = 5800.0;

    public enum ControlMode {
      Position("Position"), Velocity("Velocity");

      private final String name;
    
      ControlMode (String name) {
        this.name = name;
      }

      public String toString() {
        return name;
      }
    }
}
