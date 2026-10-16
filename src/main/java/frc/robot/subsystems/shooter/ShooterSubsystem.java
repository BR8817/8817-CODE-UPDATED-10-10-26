package frc.robot.subsystems.shooter;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooterhood.HoodIO;
import frc.robot.subsystems.shooterhood.HoodIOTalonFX;

@Logged
public class ShooterSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    PREPARE_SHOT,
    SHOOTING,
    AUTO_SHOOTING,
    MANUAL_SHOT,
    REVERSEMF
  }

  public static class TagAimData {
    public final double distanceM;
    public final double forwardM;
    public final double lateralM;

    public TagAimData(double d, double f, double l) {
      distanceM = d;
      forwardM = f;
      lateralM = l;
    }

    public boolean isValid() {
      return distanceM > 0.0;
    }
  }

  private final ShooterIO shooterIO = new ShooterIOTalonFX();
  private final ShooterIO.ShooterIOInputs shooterInputs = new ShooterIO.ShooterIOInputs();

  private final HoodIO hoodIO = new HoodIOTalonFX();
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  private WantedState wanted = WantedState.IDLE;

  private double targetTopRps = 0.0;
  private double targetBottomRps = 0.0;
  private double targetFeedRps = 0.0;
  private double targetHoodRot = 0.0;

  private double mapRps = 0.0;
  private double mapHoodRot = 0.0;

  private boolean feedEnabled = false;

  private double manualHoodRot = 0.0;
  private boolean manualHoodOverride = false;
  private double manualTopRps = Constants.ShooterConstants.MANUAL_SHOT_RPS;
  private double manualBottomRps = Constants.ShooterConstants.MANUAL_SHOT_RPS;

  private double shooterRpsOffset = 0.0;
  private double shotDistanceOverrideM = -1.0;

  // Keeps the last good tag distance for a moment so one dropped frame doesn't jerk the hood/RPS.
  private static final double DISTANCE_HOLD_SEC = 0.5;
  private double lastValidDistM = -1.0;
  private double lastValidDistTimeSec = 0.0;
  private double lastShotDistM = -1.0;

  private static final double READY_TOL_RPS = Constants.ShooterConstants.READY_TOL_RPS;
  private static final double READY_MIN_RPS = Constants.ShooterConstants.READY_MIN_RPS;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
    SmartDashboard.putNumber("Shooter/RpsOffset", 0.0);
    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);

    if (Constants.ShooterConstants.LIVE_TUNING) {
      SmartDashboard.putNumber("Shooter/FeederRPS", Constants.ShooterConstants.FEEDER_RPS);
    }
  }

  public void setWantedState(WantedState state) {
    wanted = state;

    if (state != WantedState.SHOOTING
        && state != WantedState.AUTO_SHOOTING
        && state != WantedState.MANUAL_SHOT
        && state != WantedState.REVERSEMF) {
      feedEnabled = false;
    }
  }

  public WantedState getWantedState() {
    return wanted;
  }

  public void setFeedEnabled(boolean enable) {
    feedEnabled = enable;
  }

  public void setShotDistanceOverrideMeters(double distanceM) {
    shotDistanceOverrideM = distanceM;
  }

  public void clearShotDistanceOverride() {
    shotDistanceOverrideM = -1.0;
  }

  public boolean hasShotDistanceOverride() {
    return shotDistanceOverrideM > 0.0;
  }

  public void syncManualToCurrentTargets() {
    manualHoodRot =
        clamp(
            hoodInputs.hoodPositionRot,
            Constants.ShooterConstants.HOOD_MIN_ROT,
            Constants.ShooterConstants.HOOD_MAX_ROT);

    manualTopRps = Constants.ShooterConstants.MANUAL_SHOT_RPS;
    manualBottomRps = Constants.ShooterConstants.MANUAL_SHOT_RPS;

    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  public void adjustManualHood(double deltaRot) {
    manualHoodRot =
        clamp(
            manualHoodRot + deltaRot,
            Constants.ShooterConstants.HOOD_MIN_ROT,
            Constants.ShooterConstants.HOOD_MAX_ROT);

    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
  }

  // Testing: when on, the hood holds manualHoodRot in every state instead of using the table.
  public void toggleManualHoodOverride() {
    manualHoodOverride = !manualHoodOverride;
  }

  public boolean isManualHoodOverride() {
    return manualHoodOverride;
  }

  // Tuning: prints the current distance / hood / RPS so a good shot can be copied into the shot table.
  public void printShotTuningPoint() {
    double rps = (targetTopRps + targetBottomRps) * 0.5;
    System.out.printf(
        "SHOT POINT -> dist=%.3f m  hood=%.4f rot  rps=%.2f%n",
        lastShotDistM, hoodInputs.hoodPositionRot, rps);
    SmartDashboard.putString(
        "Hood/LastTuningPoint",
        String.format("%.3f m, %.4f rot, %.2f rps", lastShotDistM, hoodInputs.hoodPositionRot, rps));
  }

  public void adjustManualShooterRps(double deltaRps) {
    manualTopRps =
        clamp(
            manualTopRps + deltaRps,
            Constants.ShooterConstants.SHOOTER_MIN_RPS,
            Constants.ShooterConstants.SHOOTER_MAX_RPS);

    manualBottomRps =
        clamp(
            manualBottomRps + deltaRps,
            Constants.ShooterConstants.SHOOTER_MIN_RPS,
            Constants.ShooterConstants.SHOOTER_MAX_RPS);

    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  public void increaseShooterSpeed() {
    shooterRpsOffset += 1.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public void decreaseShooterSpeed() {
    shooterRpsOffset -= 1.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public void resetShooterOffset() {
    shooterRpsOffset = 0.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public double getShooterOffset() {
    return shooterRpsOffset;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean flywheelsAtSpeed() {
    double avgTarget = (Math.abs(targetTopRps) + Math.abs(targetBottomRps)) * 0.5;

    if (avgTarget < READY_MIN_RPS) {
      return false;

    }

    double top = Math.abs(shooterInputs.topVelocityRps);
    double bottom = Math.abs(shooterInputs.bottomVelocityRps);

    boolean withinTolerance =
        Math.abs(top - Math.abs(targetTopRps)) <= READY_TOL_RPS
            && Math.abs(bottom - Math.abs(targetBottomRps)) <= READY_TOL_RPS;

    double minPercent = 0.93;

    boolean abovePercent =
        top >= Math.abs(targetTopRps) * minPercent
            && bottom >= Math.abs(targetBottomRps) * minPercent;

    return withinTolerance || abovePercent;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean hoodAtTarget() {
    return Math.abs(hoodInputs.hoodPositionRot - targetHoodRot)
        <= Constants.ShooterConstants.HOOD_READY_TOL_ROT;
  }

  @NotLogged
  public TagAimData getHubTagAimData() {
    PhotonPipelineResult res = camera.getLatestResult();

    if (res == null || !res.hasTargets()) {
      return new TagAimData(-1.0, -1.0, 0.0);
    }

    boolean blue =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;

    List<Integer> hubIds =
        blue ? List.of(21, 24, 25, 26, 18, 27) : List.of(2, 11, 8, 5, 9, 10);

    List<Integer> hubCenters =
        blue ? List.of(18, 21, 26) : List.of(2, 5, 10);

    double camX = Constants.VisionConstants.CAM_X_METERS;
    double camY = Constants.VisionConstants.CAM_Y_METERS;

    double dist = 0.0;
    double forward = 0.0;
    double lateral = 0.0;
    int tagCount = 0;

    var targets = res.getTargets();

    if (targets.size() == 1) {
      var t = targets.get(0);

      if (hubIds.contains(t.getFiducialId())) {
        final double robotX = t.getBestCameraToTarget().getX() + camX;
        final double robotY = t.getBestCameraToTarget().getY() + camY;

        forward += robotX;
        lateral += robotY;
        dist += Math.hypot(robotX, robotY);
        tagCount++;
      }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0);
      var t1 = targets.get(1);

      if ((t1.getFiducialId() - t0.getFiducialId()) == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)
          || !(hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId()))) {
        for (var t : targets) { 
          if (hubIds.contains(t.getFiducialId())) { 
            forward += t.getBestCameraToTarget().getX() + camX;
            lateral += t.getBestCameraToTarget().getY() + camY;

            dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          }
        }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            forward += t.getBestCameraToTarget().getX() + camX;
            lateral += t.getBestCameraToTarget().getY() + camY;

            dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        if (hubCenters.contains(t.getFiducialId())) {
          final double robotX = t.getBestCameraToTarget().getX() + camX;
          final double robotY = t.getBestCameraToTarget().getY() + camY;

          forward += robotX;
          lateral += robotY;
          dist += Math.hypot(robotX, robotY);
          tagCount++;
        }
      }
    } else {
      for (var t : targets) {
        if (hubIds.contains(t.getFiducialId())) {
          final double robotX = t.getBestCameraToTarget().getX() + camX;
          final double robotY = t.getBestCameraToTarget().getY() + camY;

          forward += robotX;
          lateral += robotY;
          dist += Math.hypot(robotX, robotY);
          tagCount++;
        }
      }
    }
    


    if (tagCount == 0) return new TagAimData(-1.0, -1.0, 0.0);
    return new TagAimData(dist / tagCount, forward / tagCount, lateral / tagCount);
  }

  @NotLogged
  public boolean hasHubTarget() {
    return getHubTagAimData().isValid();
  }

  @NotLogged
  public double getDistanceToTagMeters() {
    return getHubTagAimData().distanceM;
  }

  @NotLogged
  public double getForwardToTagMeters() {
    return getHubTagAimData().forwardM;
  }

  @NotLogged
  public double getLateralToTagMeters() {
    return getHubTagAimData().lateralM; 
  }

  private double interpolate(double x, double[] xTable, double[] yTable) {
    if (x <= xTable[0]) {
      return yTable[0];
    }

    if (x >= xTable[xTable.length - 1]) {
      return yTable[yTable.length - 1];
    }

    for (int i = 0; i < xTable.length - 1; i++) {
      if (x >= xTable[i] && x <= xTable[i + 1]) {
        double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
        return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
      }
    }

    return yTable[yTable.length - 1];
  }

  public double calcMapHoodForDistance(double dist) {
    if (dist < 0.0) {
      return hoodInputs.hoodPositionRot;
    }

    return clamp(
        interpolate(
            dist,
            Constants.ShooterConstants.SHOT_DISTANCE_M,
            Constants.ShooterConstants.SHOT_HOOD_ROT),
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Constants.ShooterConstants.HOOD_MAX_ROT);
  }

  public double calcMapRpsForDistance(double dist) {
    if (dist < 0.0) {
      return Constants.ShooterConstants.SHOOTER_TOP_RPS;
    }

    return clamp(
        interpolate(
            dist,
            Constants.ShooterConstants.SHOT_DISTANCE_M,
            Constants.ShooterConstants.SHOT_RPS),
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    TagAimData aim = getHubTagAimData();

    double rawDist = aim.distanceM;
    double now = Timer.getFPGATimestamp();

    if (rawDist > 0.0) {
      lastValidDistM = rawDist;
      lastValidDistTimeSec = now;
    }

    double visionDist =
        (now - lastValidDistTimeSec) <= DISTANCE_HOLD_SEC ? lastValidDistM : -1.0;
    double shotDist = shotDistanceOverrideM > 0.0 ? shotDistanceOverrideM : visionDist;
    lastShotDistM = shotDist;

    mapHoodRot = calcMapHoodForDistance(shotDist);
    mapRps = calcMapRpsForDistance(shotDist);

    switch (wanted) {
      case IDLE -> {
        targetTopRps = 0.0;
        targetBottomRps = 0.0;
        targetFeedRps = 0.0;
        targetHoodRot = 0.0;
        feedEnabled = false;
      }

      case PREPARE_SHOT -> {
        targetHoodRot = mapHoodRot;
        targetTopRps = mapRps + shooterRpsOffset;
        targetBottomRps = mapRps + shooterRpsOffset;
        targetFeedRps = 0.0;
      }

      case SHOOTING -> {
        targetHoodRot = mapHoodRot;
        targetTopRps = mapRps + shooterRpsOffset;
        targetBottomRps = mapRps + shooterRpsOffset;

        double feed = Constants.ShooterConstants.FEEDER_RPS;

        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }

        targetFeedRps = feedEnabled ? feed : 0.0;
      }

      case AUTO_SHOOTING -> {
        targetHoodRot = mapHoodRot;
        targetTopRps = Constants.ShooterConstants.AUTO_SHOT_RPS + shooterRpsOffset;
        targetBottomRps = Constants.ShooterConstants.AUTO_SHOT_RPS + shooterRpsOffset;

        double feed = Constants.ShooterConstants.FEEDER_RPS;

        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }

        targetFeedRps = feedEnabled ? feed : 0.0;
      }

      case MANUAL_SHOT -> {
        targetHoodRot = Constants.ShooterConstants.HOOD_MAX_ROT;
        targetTopRps = Constants.ShooterConstants.MANUAL_SHOT_RPS + shooterRpsOffset;
        targetBottomRps = Constants.ShooterConstants.MANUAL_SHOT_RPS + shooterRpsOffset;

        double feed = Constants.ShooterConstants.FEEDER_RPS;

        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }

        targetFeedRps = feedEnabled ? feed : 0.0;
      }

      case REVERSEMF -> {


        targetFeedRps = Constants.ShooterConstants.FEEDER_RPS * -1.0;
      }
    }

    if (wanted == WantedState.IDLE) {
      shooterIO.stop();
    } else {
      shooterIO.setFlywheelVelocityRps(targetTopRps, targetBottomRps, targetFeedRps);
    }

    if (manualHoodOverride) {
      // Lets you type a value into Hood/ManualTargetRot on the dashboard as well as use the D-pad.
      manualHoodRot =
          clamp(
              SmartDashboard.getNumber("Hood/ManualTargetRot", manualHoodRot),
              Constants.ShooterConstants.HOOD_MIN_ROT,
              Constants.ShooterConstants.HOOD_MAX_ROT);
      targetHoodRot = manualHoodRot;
    }
    SmartDashboard.putBoolean("Hood/ManualOverride", manualHoodOverride);

    hoodIO.setHoodPositionRot(targetHoodRot);

    SmartDashboard.putNumber("Vision/TagDistanceM", rawDist);
    SmartDashboard.putNumber("Vision/TagForwardM", aim.forwardM);
    SmartDashboard.putNumber("Vision/TagLateralM", aim.lateralM);

    SmartDashboard.putNumber("Shooter/ShotDistanceUsedM", shotDist);
    SmartDashboard.putBoolean("Shooter/HasShotDistance", shotDist > 0.0);
    SmartDashboard.putBoolean("Shooter/UsingDistanceOverride", shotDistanceOverrideM > 0.0);

    SmartDashboard.putNumber("Shooter/TargetTopRPS", targetTopRps);
    SmartDashboard.putNumber("Shooter/TargetMidRPS", shooterInputs.midVelocityRps);
    SmartDashboard.putNumber("Shooter/TargetBottomRPS", targetBottomRps);

    SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelsAtSpeed());
    SmartDashboard.putNumber("Shooter/MapBaseRPS", mapRps);
    SmartDashboard.putBoolean("Shooter/ManualShotActive", wanted == WantedState.MANUAL_SHOT);

    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);

    SmartDashboard.putNumber("Shooter/ActualTopRPS", shooterInputs.topVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualMidRPS", shooterInputs.midVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualBottomRPS", shooterInputs.bottomVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualFeederRPS", shooterInputs.feederVelocityRps);

    SmartDashboard.putNumber("Hood/PositionDeg", hoodInputs.hoodPositionRot * 360.0);
    SmartDashboard.putNumber("Hood/TargetDeg", targetHoodRot * 360.0);
    SmartDashboard.putBoolean("Hood/AtTarget", hoodAtTarget());
    SmartDashboard.putNumber("Hood/MapBaseRot", mapHoodRot);
    SmartDashboard.putNumber("Hood/ManualRot", manualHoodRot);
  }

  @Override
  public void simulationPeriodic() {
    shooterIO.simulationPeriodic();
    hoodIO.simulationPeriodic();
  }
}