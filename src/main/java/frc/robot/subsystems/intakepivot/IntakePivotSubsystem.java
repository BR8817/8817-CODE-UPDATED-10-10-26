// IntakePivotSubsystem.java
package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IntakePivotSubsystem extends SubsystemBase {

  public enum WantedState {
    DEPLOY,
    STOW,
    IDLE
  }

  private final IntakePivotIO io = new IntakePivotIOTalonFX();
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();

  private WantedState wanted = WantedState.IDLE;

  private static final double DEPLOY_PCT = +0.50;//+1.00;
  private static final double STOW_PCT = -0.700;
  private static final double MAX_ROT_BOTH_SIDES = 0.6200;
  private static final double SLOWDOWN_ZONE_ROT = 0.15;
  private static final double MIN_SLOW_PCT = 0.10;

  public void setWantedState(WantedState state) {
    wanted = state;
  }

  public WantedState getWantedState() {
    return wanted;
  }

  public double getPivotPositionRot() {
    return inputs.pivotPositionRot;
  }

  public double getPivotAbsPositionRot() {
    return inputs.pivotAbsPositionRot;
  }

  public boolean isHomed() {
    return inputs.homed;
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  /** Used by ShootOnMoveCommand for small oscillation movements. */
  public void setOpenLoopPercent(double percent) {
    io.setPercent(MathUtil.clamp(percent, -0.2, 0.2));
  }

  /** Teleop manual control. Ignores min/max rotation limits. */
  public void setTeleopOpenLoopPercent(double percent) {
    wanted = WantedState.IDLE;
    io.setPercent(MathUtil.clamp(percent, -1.0, 1.0));
  }

  public void stopOpenLoop() {
    wanted = WantedState.IDLE;
    io.stop();
  }

  private boolean atOrPastMax() {
    return Math.abs(inputs.pivotPositionRot) >= MAX_ROT_BOTH_SIDES;
  }

  private boolean atOrPastMin() {
    return inputs.pivotPositionRot <= Constants.IntakePivotConstants.MIN_ROT;
  }

  private double calcDecelFactor(double currentRot, double targetRot) {
    double remaining = Math.abs(targetRot - currentRot);

    if (remaining >= SLOWDOWN_ZONE_ROT) {
      return 1.0;
    }

    return remaining / SLOWDOWN_ZONE_ROT;
  }

  private double applyDecel(double fullPct, double currentRot, double targetRot) {
    double factor = calcDecelFactor(currentRot, targetRot);
    double scaled = fullPct * factor;
    double minOutput = Math.copySign(MIN_SLOW_PCT, fullPct);

    if (Math.abs(scaled) < MIN_SLOW_PCT) {
      return minOutput;
    }

    return scaled;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double out = 0.0;

    switch (wanted) {
      case IDLE -> out = 0.0;

      case DEPLOY -> {
        if (atOrPastMax()) {
          out = 0.0;
        } else {
          out = applyDecel(DEPLOY_PCT, inputs.pivotPositionRot, MAX_ROT_BOTH_SIDES);
        }
      }

      case STOW -> {
        if (atOrPastMin()) {
          out = 0.0;
        } else {
          out = applyDecel(STOW_PCT, inputs.pivotPositionRot, Constants.IntakePivotConstants.MIN_ROT);
        }
      }
    }

    if (wanted != WantedState.IDLE) {
      if (out == 0.0) {
        io.stop();
      } else {
        io.setPercent(out);
      }
    }

    SmartDashboard.putNumber("IntakePivot/PosRot", inputs.pivotPositionRot);
    SmartDashboard.putNumber("IntakePivot/AbsRot", inputs.pivotAbsPositionRot);
    SmartDashboard.putNumber("IntakePivot/VelRps", inputs.pivotVelocityRps);
    SmartDashboard.putString("IntakePivot/Wanted", wanted.name());
    SmartDashboard.putBoolean("IntakePivot/AtMin", atOrPastMin());
    SmartDashboard.putBoolean("IntakePivot/AtMax", atOrPastMax());
    SmartDashboard.putBoolean("IntakePivot/Homed", inputs.homed);
    SmartDashboard.putNumber("IntakePivot/OutputPct", out);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}