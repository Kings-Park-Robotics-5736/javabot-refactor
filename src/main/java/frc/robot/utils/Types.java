package frc.robot.utils;


/**
 * @brief A class for creating convenience type wrappers to allow us to more easily pass around values.
 */
public final class Types {
  public static final class PidConstants {
    public  double p; 
    public  double i;
    public  double d;

    public PidConstants(double p, double i, double d) {
      this.p = p;
      this.i = i;
      this.d = d;
    }
  }

  public static final class FeedForwardConstants {
    public final double ks;
    public final double kv;
    public final double ka;
    public final double kg;

    public FeedForwardConstants(double ks, double kv, double ka) {
      this(ks, kv, ka, 0);
    }
    public FeedForwardConstants(double ks, double kv, double ka, double kg) {
      this.ks = ks;
      this.kv = kv;
      this.ka = ka;
      this.kg = kg;
    }
  }

  public static final class Limits {
    public final double low;
    public final double high;


    public Limits(double low, double high) {
      this.low = low;
      this.high = high;
    }
  }

  public static final class MotionProfileConstants{
    public final double kMaxVelocity; //max velocity is 90 deg / sec
    public final double kMaxAcceleration; 
    public final double kMaxJerk;
    public final double maxError;

    public MotionProfileConstants(double maxVel, double maxAccel, double maxJerk, double maxError){
      this.kMaxVelocity = maxVel;
      this.kMaxAcceleration = maxAccel;
      this.kMaxJerk = maxJerk;
      this.maxError = maxError;
    }
  }


  public enum LEDState{
    IN_RANGE,
    HAVE_PIECE,
    NO_PIECE,
    NONE,
    SEE_TAG,
}
  public enum DirectionType{
    UP,
    DOWN
  };

  public enum PositionType{
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
  }

  public enum SysidMechanism{
    NONE,
    INTAKE_TOP,
    INTAKE_BOTTOM,
    DRIVE,
    SHOOTER_LEFT,
    SHOOTER_RIGHT,
    KICKUP_RIGHT,
    ARM,
    ELEVATOR
  }

  
  public enum RobotMode {
    DISABLED,
    AUTON,
    TELEOP,
    TEST
}



}
