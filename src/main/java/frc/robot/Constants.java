package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "ThriftyCam";

    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double ROBOT_LENGTH_IN = 30.0;
    public static final double ROBOT_WIDTH_IN = 25.0;

    public static final double CAM_X_IN = -5.0;
    public static final double CAM_Y_IN = 0.0;
    public static final double CAM_Z_IN = 21.0;

    public static final double CAM_X_METERS = Units.inchesToMeters(CAM_X_IN);
    public static final double CAM_Y_METERS = Units.inchesToMeters(CAM_Y_IN);
    public static final double CAM_Z_METERS = Units.inchesToMeters(CAM_Z_IN);

    public static final double CAM_ROLL_DEG = 0.0;
    public static final double CAM_PITCH_DEG = 20.0;
    public static final double CAM_YAW_DEG = 0.0;

    // Tune this if auto-aim consistently shoots left/right.
    // Positive value aims slightly more one way, negative aims the other.
    public static final double AIM_YAW_OFFSET_DEG = 0.0;

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(CAM_X_METERS, CAM_Y_METERS, CAM_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(CAM_ROLL_DEG),
                Units.degreesToRadians(CAM_PITCH_DEG),
                Units.degreesToRadians(CAM_YAW_DEG)));

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    public static final double MAX_AMBIGUITY = 0.25;
    public static final double MAX_DISTANCE_METERS = 4.5;
  }

  public static final class ShooterConstants {
    private ShooterConstants() {}

    public static final int TOPLEFT_ID = 9;
    public static final int TOPRIGHT_ID = 16;
    public static final int BTMLEFT_ID = 10;
    public static final int BTMRIGHT_ID = 17;
    public static final int FEEDER_ID = 11;
    public static final int HOOD_MOTOR_ID = 12;
    public static final int HOOD_CANCODER_ID = 36;
    public static final String CANBUS = "rio";

    public static final double FLYWHEEL_GEAR_RATIO = 1.0;

    public static final double kP = 0.065;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.15;
    public static final double kS = 0.0;
    public static final double kA = 0.01;

    public static final double SHOOTER_TOP_RPS = 23.0;
    //public static final double SHOOTER_BOTTOM_RPS = 23.0;

    public static final double MANUAL_SHOT_RPS = 50;//37.5;
    public static final double AUTO_SHOT_RPS = 27.0;

    public static final double FEEDER_IDLE_RPS = 0.0;
    public static final double FEEDER_RPS = 85.0;
    public static final double INDEXER_RPS = 80.0;
    public static final double INDEXER_IDLE_RPS = 0.0;

    public static final double SHOOTER_MIN_RPS = 0.0;
    public static final double SHOOTER_MAX_RPS = 50.0;

    public static final double READY_TOL_RPS = 4.5;
    public static final double READY_MIN_RPS = 10.0;

    public static final double SUPPLY_LIMIT_A = 60.0;
    public static final double FEEDER_SUPPLY_LIMIT_A = 35.0;

    public static final boolean TOPLEFT_INVERTED = true;
    public static final boolean TOPRIGHT_INVERTED = false;//change direction of shooter motors here 
    public static final boolean BTMLEFT_INVERTED = true;//they are now belted so all should go sam direction
    public static final boolean BTMRIGHT_INVERTED = false;
    public static final boolean FEEDER_INVERTED = false;

    public static final boolean LIVE_TUNING = true;

    public static final double HOOD_GEAR_RATIO = 98.4;
    public static final double HOOD_kP = 80.0;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.051;
    public static final double HOOD_kV = 0.0;

    public static final double HOOD_MIN_ROT = 0.0;
    public static final double HOOD_MAX_ROT = 0.45; //was 0.84
    public static final double HOOD_READY_TOL_ROT = 0.005;

    public static final double[] SHOT_DISTANCE_M = {
      0.976, 1.368, 1.591, 1.8967, 2.10054, 2.2331, 2.3906, 2.6780, 3.210, 3.482, 3.500
    };

    // PLACEHOLDER - re-tune! Old table (pre gear ratio change, max 0.84) scaled by 0.45/0.84 as a
    // starting guess. Old values: 0.378258, 0.412734, 0.462314, 0.5286, 0.53990, 0.553105,
    // 0.582666, 0.612939, 0.622246, 0.673594, 0.712881
    public static final double[] SHOT_HOOD_ROT = {
      0.203, 0.221, 0.248, 0.283, 0.289, 0.296, 0.312,
      0.328, 0.333, 0.361, 0.382
    };

    public static final double[] SHOT_RPS = {
      18.0, 20.5, 22.0, 23.5, 24.25, 26.00, 28.00,
      29.25, 30.0, 31.5, 33.0
    };
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_ID = 13;
    public static final double INTAKE_PERCENT = -0.7;
    public static final boolean ROLLER_INVERTED = false;
    public static final double ROLLER_SUPPLY_LIMIT_A = 30.0;
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}

    public static final int BELTER_ID = 15;
    public static final double BELT_PERCENT = 0.8;
    public static final boolean BELT_INVERTED = true;
    public static final double BELT_SUPPLY_LIMIT_A = 40.0;
  }

  public static final class IntakePivotConstants {
    private IntakePivotConstants() {}

    public static final int PIVOT_ID = 14;
    public static final int PIVOT_CANCODER_ID = 37;

    public static final double PIVOT_GEAR_RATIO = 36.36 / 1.0;

    public static final double MIN_ROT = 0.001;
    public static final double MAX_ROT = -0.41;

    public static final double kP = 0.0001;//0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;

    public static final boolean PIVOT_INVERTED = true;

    public static final double PIVOT_SUPPLY_LIMIT_A = 40.0;
    public static final double READY_TOL_ROT = 0.01;

    public static final double CANCODER_OFFSET_ROT = -0.35138;
  }
}