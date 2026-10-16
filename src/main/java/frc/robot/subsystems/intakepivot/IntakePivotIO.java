package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;

/**
 * Hardware abstraction for the intake pivot arm.
 * The pivot swings the intake down to the ground (deploy) or
 * back up into the robot frame (stow).
 */
public interface IntakePivotIO {

  @Logged
  class IntakePivotIOInputs {
    public double motorPercent;
    public double motorVolts;
    public double motorAmps;
    public double motorTempC;
    public double pivotPositionRot;
    public double pivotVelocityRps;
    public double pivotAbsPositionRot;
    public boolean homed;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setPercent(double percent);

  void stop();

  default void zeroPosition() {}

  default void simulationPeriodic() {}
}