package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * TalonFX + CANcoder implementation of the intake pivot.
 *
 * Instead of zeroing the Talon to 0 wherever the arm happens to be,
 * this seeds the Talon position from the absolute through-bore/CANcoder.
 */
public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivotMotor = new TalonFX(Constants.IntakePivotConstants.PIVOT_ID);
  private final CANcoder pivotEncoder = new CANcoder(Constants.IntakePivotConstants.PIVOT_CANCODER_ID);
  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Angle> motorPos = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> motorVel = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> motorVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> motorAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> motorTemp = pivotMotor.getDeviceTemp();
  private final StatusSignal<Angle> cancoderAbs = pivotEncoder.getAbsolutePosition();

  private double lastPercent = 0.0;
  private boolean seeded = false;

  public IntakePivotIOTalonFX() {
    pivotEncoder.getConfigurator().apply(new CANcoderConfiguration());

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = Constants.IntakePivotConstants.PIVOT_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    cfg.Feedback.SensorToMechanismRatio = Constants.IntakePivotConstants.PIVOT_GEAR_RATIO;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.PIVOT_SUPPLY_LIMIT_A;

    pivotMotor.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorPos,
        motorVel,
        motorVolts,
        motorAmps,
        motorTemp,
        cancoderAbs);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPos, motorVel, motorVolts, motorAmps, motorTemp, cancoderAbs);

    double absRot = cancoderAbs.getValueAsDouble();

    if (!seeded) {
      pivotMotor.setPosition(absToMechanismRot(absRot));
      seeded = true;
    }

    inputs.motorPercent = lastPercent;
    inputs.motorVolts = motorVolts.getValueAsDouble();
    inputs.motorAmps = motorAmps.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();

    inputs.pivotPositionRot = motorPos.getValueAsDouble();
    inputs.pivotVelocityRps = motorVel.getValueAsDouble();
    inputs.pivotAbsPositionRot = absRot;

    inputs.homed = seeded;
  }

  /**
   * Converts absolute encoder position into mechanism rotations.
   *
   * CANCODER_OFFSET_ROT should be the AbsRot value when the intake is physically stowed.
   * That makes stowed = 0.0 rotations.
   */
  private double absToMechanismRot(double absRot) {
    double zeroed = absRot - Constants.IntakePivotConstants.CANCODER_OFFSET_ROT;

    // Wrap from [0, 1) into roughly [-0.5, 0.5]
    if (zeroed > 0.5) {
      zeroed -= 1.0;
    } else if (zeroed < -0.5) {
      zeroed += 1.0;
    }

    return zeroed;
  }

  @Override
  public void setPercent(double percent) {
    lastPercent = percent;
    pivotMotor.setControl(duty.withOutput(percent));
  }

  @Override
  public void stop() {
    lastPercent = 0.0;
    pivotMotor.stopMotor();
  }

  @Override
  public void zeroPosition() {
    pivotMotor.setPosition(0.0);
    seeded = true;
  }
}