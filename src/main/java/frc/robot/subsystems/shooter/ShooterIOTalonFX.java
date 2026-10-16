package frc.robot.subsystems.shooter;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX topLeft = new TalonFX(Constants.ShooterConstants.TOPLEFT_ID);
  private final TalonFX btmLeft = new TalonFX(Constants.ShooterConstants.BTMLEFT_ID);
  private final TalonFX topRight = new TalonFX(Constants.ShooterConstants.TOPRIGHT_ID);
  private final TalonFX btmRight = new TalonFX(Constants.ShooterConstants.BTMRIGHT_ID);
  private final TalonFX feeder = new TalonFX(Constants.ShooterConstants.FEEDER_ID);

  private final List<TalonFX> motors = List.of(topLeft, topRight, btmLeft, btmRight, feeder);

  private final VelocityVoltage topLeftVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage topRightVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  //private final VelocityVoltage btmLeftVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  //private final VelocityVoltage btmRightVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage feederVelCtrl = new VelocityVoltage(0).withEnableFOC(true);

  private final MotorAlignmentValue leftAlign =
      (Constants.ShooterConstants.BTMLEFT_INVERTED == Constants.ShooterConstants.TOPLEFT_INVERTED)
          ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed;
  private final MotorAlignmentValue rightAlign =
      (Constants.ShooterConstants.BTMRIGHT_INVERTED == Constants.ShooterConstants.TOPRIGHT_INVERTED)
          ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed;

  private final StatusSignal<AngularVelocity> topRightVel = topLeft.getVelocity();
  private final StatusSignal<AngularVelocity> topLeftVel = topRight.getVelocity();
  private final StatusSignal<AngularVelocity> btmRightVel = btmRight.getVelocity();
   private final StatusSignal<AngularVelocity> btmLeftVel = btmLeft.getVelocity();
  private final StatusSignal<AngularVelocity> feedVel = feeder.getVelocity();

  private final StatusSignal<Voltage> topRightVolts = topLeft.getMotorVoltage();
  private final StatusSignal<Voltage> topLeftVolts = topRight.getMotorVoltage();
  private final StatusSignal<Voltage> btmRightVolts = btmRight.getMotorVoltage();
  private final StatusSignal<Voltage> btmLeftVolts = btmLeft.getMotorVoltage();
  private final StatusSignal<Voltage> feederVolts = feeder.getMotorVoltage();

  private final StatusSignal<Current> topRightAmps = topLeft.getSupplyCurrent();
  private final StatusSignal<Current> topLeftAmps = topRight.getSupplyCurrent();
  private final StatusSignal<Current> btmRightAmps = btmRight.getSupplyCurrent();
  private final StatusSignal<Current> btmLeftAmps = btmLeft.getSupplyCurrent();
  private final StatusSignal<Current> feederAmps = feeder.getSupplyCurrent();

  private final StatusSignal<Temperature> topRightTemp = topLeft.getDeviceTemp();
  private final StatusSignal<Temperature> topLeftTemp = topRight.getDeviceTemp();
  private final StatusSignal<Temperature> btmRightTemp = btmRight.getDeviceTemp();
   private final StatusSignal<Temperature> btmLeftTemp = btmLeft.getDeviceTemp();
  private final StatusSignal<Temperature> feederTemp = feeder.getDeviceTemp();

  public ShooterIOTalonFX() {
    TalonFXConfiguration topRightCfg = buildFlywheelConfig(
        Constants.ShooterConstants.TOPRIGHT_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration topLeftCfg = buildFlywheelConfig(
        Constants.ShooterConstants.TOPLEFT_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration btmRightCfg = buildFlywheelConfig(
        Constants.ShooterConstants.BTMRIGHT_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);
    
    TalonFXConfiguration btmLeftCfg = buildFlywheelConfig(
        Constants.ShooterConstants.BTMLEFT_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration feedCfg = buildFlywheelConfig(
        Constants.ShooterConstants.FEEDER_INVERTED,
        Constants.ShooterConstants.FEEDER_SUPPLY_LIMIT_A);

    topLeft.getConfigurator().apply(topLeftCfg);
    topRight.getConfigurator().apply(topRightCfg);
    btmLeft.getConfigurator().apply(btmLeftCfg);
    btmRight.getConfigurator().apply(btmRightCfg);
    feeder.getConfigurator().apply(feedCfg);


    topLeft.setControl(new Follower(btmLeft.getDeviceID(), MotorAlignmentValue.Aligned));
    topRight.setControl(new Follower(btmRight.getDeviceID(), MotorAlignmentValue.Aligned));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        topRightVel, topLeftVel, btmLeftVel, btmRightVel, feedVel,
        topRightVolts, topLeftVolts, btmRightVolts, feederVolts,
        topRightAmps, topLeftAmps, btmLeftAmps, feederAmps,
        topRightTemp, topLeftTemp, btmRightTemp, feederTemp);
  }

  private TalonFXConfiguration buildFlywheelConfig(boolean inverted, double currentLimit) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;

    cfg.Slot0.kP = Constants.ShooterConstants.kP;
    cfg.Slot0.kI = Constants.ShooterConstants.kI;
    cfg.Slot0.kD = Constants.ShooterConstants.kD;
    cfg.Slot0.kV = Constants.ShooterConstants.kV;
    cfg.Slot0.kS = Constants.ShooterConstants.kS;
    cfg.Slot0.kA = Constants.ShooterConstants.kA;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = currentLimit;

    // Helps stop huge current spikes when balls load the drum.
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;

    // Smooths shooter voltage changes so the RIO/battery does not get slammed.
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.10;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.10;

    cfg.MotorOutput.Inverted = inverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
       topRightVel, topLeftVel, btmLeftVel, btmRightVel, feedVel,
        topRightVolts, topLeftVolts, btmRightVolts, feederVolts,
        topRightAmps, topLeftAmps, btmLeftAmps, feederAmps,
        topRightTemp, topLeftTemp, btmRightTemp, feederTemp);

    inputs.topVelocityRps = topLeftVel.getValueAsDouble();
    inputs.midVelocityRps = topRightVel.getValueAsDouble();
    inputs.bottomVelocityRps = btmLeftVel.getValueAsDouble();
    inputs.bottomVelocityRps = btmRightVel.getValueAsDouble();
    inputs.feederVelocityRps = feedVel.getValueAsDouble();

    inputs.topAppliedVolts = topLeftVolts.getValueAsDouble();
    inputs.midAppliedVolts = topRightVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = btmLeftVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = btmRightVolts.getValueAsDouble();
    inputs.feederAppliedVolts = feederVolts.getValueAsDouble();

    inputs.topCurrentAmps = topLeftAmps.getValueAsDouble();
    inputs.midCurrentAmps = topRightAmps.getValueAsDouble();
    inputs.bottomCurrentAmps = btmLeftAmps.getValueAsDouble();
    inputs.bottomCurrentAmps = btmRightAmps.getValueAsDouble();
    inputs.feederCurrentAmps = feederAmps.getValueAsDouble();

    inputs.topTempC = topLeftTemp.getValueAsDouble();
    inputs.midTempC = topRightTemp.getValueAsDouble();
    inputs.bottomTempC = btmLeftTemp.getValueAsDouble();
    inputs.bottomTempC = btmRightTemp.getValueAsDouble();
    inputs.feederTempC = feederTemp.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocityRps(double topRps, double bottomRps, double feedRps) {
    topLeft.setControl(topLeftVelCtrl.withVelocity(topRps));
    topRight.setControl(topRightVelCtrl.withVelocity(bottomRps));
    feeder.setControl(feederVelCtrl.withVelocity(feedRps));

    btmLeft.setControl(new Follower(topLeft.getDeviceID(), leftAlign));
    btmRight.setControl(new Follower(topRight.getDeviceID(), rightAlign));

  }

  @Override
  public void setFlywheelVoltage(double topVoltsOut, double bottomVoltsOut, double feedVoltsOut) {
    topLeft.setVoltage(topVoltsOut);
    topRight.setVoltage(bottomVoltsOut);
    feeder.setVoltage(feedVoltsOut);

    btmLeft.setControl(new Follower(topLeft.getDeviceID(), leftAlign));
    btmRight.setControl(new Follower(topRight.getDeviceID(), rightAlign));

  }

  @Override
  public void stop() {
    motors.forEach(TalonFX::stopMotor);
  }
}