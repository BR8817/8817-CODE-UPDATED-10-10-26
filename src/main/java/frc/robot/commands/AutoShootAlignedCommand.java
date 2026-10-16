package frc.robot.commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoShootAlignedCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final Vision vision;
  private final IntakePivotSubsystem intakePivotSubsystem;
  private final double maxAngularRateRps;

  private static final double kPivotLowRot = 0.25;
  private static final double kPivotHighRot = 0.55;
  private static final double kPivotOpenLoopPercent = 0.20;
  private static final double kPivotStartDelaySec = 1.0;

  // Auto safety: still require flywheels ready, but don't let hood tolerance block forever.
  private static final double kHoodBypassAfterSec = 1.0;

  private boolean pivotMovingUp = true;
  private final Timer pivotTimer = new Timer();
  private final Timer shotTimer = new Timer();

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDeadband(0.0)
          .withRotationalDeadband(0.0);

  private final Set<Subsystem> requirements;

  public AutoShootAlignedCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      Vision vision,
      IntakePivotSubsystem intakePivotSubsystem,
      double maxAngularRateRps) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.maxAngularRateRps = maxAngularRateRps;
    this.requirements = Set.of(drivetrain, shooter, indexer, intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    shooter.setWantedState(ShooterSubsystem.WantedState.AUTO_SHOOTING);
    shooter.setFeedEnabled(false);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);

    double pivotRot = intakePivotSubsystem.getPivotPositionRot();
    pivotMovingUp = pivotRot < (kPivotLowRot + kPivotHighRot) * 0.5;

    pivotTimer.restart();
    shotTimer.restart();
  }

  @Override
  public void execute() {
    double omega = 0.0;

    if (vision.hasTarget()) {
      omega = vision.calcAimOmegaRadPerSec();
      omega = MathUtil.clamp(omega, -maxAngularRateRps, maxAngularRateRps);
    }

    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(omega));

    boolean flywheelsReady = shooter.flywheelsAtSpeed();
    boolean hoodReady = shooter.hoodAtTarget();
    boolean hoodBypass = shotTimer.hasElapsed(kHoodBypassAfterSec);

    boolean feedNow = flywheelsReady && (hoodReady || hoodBypass);

    shooter.setFeedEnabled(feedNow);
    indexer.setWantedState(
        feedNow ? IndexerSubsystem.WantedState.INDEX : IndexerSubsystem.WantedState.IDLE);

    SmartDashboard.putBoolean("AutoShoot/FlywheelsReady", flywheelsReady);
    SmartDashboard.putBoolean("AutoShoot/HoodReady", hoodReady);
    SmartDashboard.putBoolean("AutoShoot/HoodBypass", hoodBypass);
    SmartDashboard.putBoolean("AutoShoot/FeedNow", feedNow);
    SmartDashboard.putNumber("AutoShoot/ShotTimerSec", shotTimer.get());

    if (pivotTimer.hasElapsed(kPivotStartDelaySec)) {
      double pivotRot = intakePivotSubsystem.getPivotPositionRot();

      if (pivotRot >= kPivotHighRot) {
        pivotMovingUp = false;
      } else if (pivotRot <= kPivotLowRot) {
        pivotMovingUp = true;
      }

      intakePivotSubsystem.setOpenLoopPercent(
          pivotMovingUp ? kPivotOpenLoopPercent : -kPivotOpenLoopPercent);
    } else {
      intakePivotSubsystem.stopOpenLoop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivotTimer.stop();
    shotTimer.stop();

    shooter.setFeedEnabled(false);
    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
    intakePivotSubsystem.stopOpenLoop();
    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}