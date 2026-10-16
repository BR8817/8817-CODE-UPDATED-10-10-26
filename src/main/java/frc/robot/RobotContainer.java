/* 
 _____  _               _____ _____ _____  _    _  _____           __   __
|  __ \| |        /\   / ____|_   _|  __ \| |  | |/ ____|    /\    \ \ / /
| |__) | |       /  \ | |      | | | |  | | |  | | (___     /  \    \ V / 
|  ___/| |      / /\ \| |      | | | |  | | |  | |\___ \   / /\ \    > <  
| |    | |____ / ____ \ |____ _| |_| |__| | |__| |____) | / ____ \  / . \ 
|_|    |______/_/    \_\_____|_____|_____/ \____/|_____/ /_/    \_\/_/ \_\

 2026 FRC Team 8817 - Buccaneer Robotics
*/
package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.ShootOnMoveCommand;
import frc.robot.commands.autos.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem.WantedState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

  public final Vision vision = new Vision();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final IndexerSubsystem indexer = new IndexerSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.85).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    drivetrain.configurePathPlanner();
    drivetrain.configureOrchestra();
    configureAutoChooser();
    configureBindings();
  }

  private void configureAutoChooser() {
    autoChooser.addOption("Drive Straight", DriveStraight.build(drivetrain));

    autoChooser.addOption(
        "Outpost Single Swipe",
        OutpostSingleSwipe.build(
            drivetrain, shooter, indexer, intake, intakePivot, vision, MaxAngularRate));

    autoChooser.addOption(
        "Outpost Double Swipe",
        OutpostDoubleSwipe.build(
            drivetrain, shooter, indexer, intake, intakePivot, vision, MaxAngularRate));

    autoChooser.addOption(
        "Depot Single", 
        DepotSingleSwipe.build(
            drivetrain, shooter, indexer, intake, intakePivot, vision, MaxAngularRate));

    autoChooser.addOption(
        "Hub",
        Hub.build(drivetrain, shooter, indexer, intake, intakePivot, vision, MaxAngularRate));

    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    autoTab.add(autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(driver.getLeftY() * MaxSpeed)
                    .withVelocityY(driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

    final var idle = new SwerveRequest.Idle();

    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    //driver.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.registerTelemetry(logger::telemeterize);

    // DRIVER CONTROLS................................................................................................

    // regular AprilTag/table-based prep
    driver
        .rightBumper()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed,
                MaxAngularRate,
                true));
        

    // DRIVER: regular AprilTag/table-based shoot-on-move
    driver
        .rightTrigger()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed,
                MaxAngularRate,
                false));
      

    // DRIVER: manual shot. No AprilTag auto-aim. Fixed 50 RPS.
    driver
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  shooter.setFeedEnabled(false);
                  shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT);
                  indexer.setWantedState(IndexerSubsystem.WantedState.INDEX);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.setFeedEnabled(false);
                  shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
                  indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
                }));

                
    // DRIVER: intake
    driver
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE), intake))
        .onFalse(
            Commands.runOnce(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake));

    // DRIVER: spit intake + indexer together
    driver
        .y()
        .whileTrue(
            Commands.run(
                () -> {
                  indexer.setWantedState(IndexerSubsystem.WantedState.SPIT);
                  intake.setWantedState(IntakeSubsystem.WantedState.SPIT);
                },
                indexer,
                intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
                  intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
                },
                indexer,
                intake));

    // DRIVER: manual hood testing. X toggles override, D-pad right/left raises/lowers the hood.
    // No shooter requirement, so these can be used while a shot command is running.
    driver.x().onTrue(Commands.runOnce(shooter::toggleManualHoodOverride));
    driver.povRight().whileTrue(Commands.run(() -> shooter.adjustManualHood(+0.002)));
    driver.povLeft().whileTrue(Commands.run(() -> shooter.adjustManualHood(-0.002)));
    // OPERATOR: X prints distance/hood/RPS to the console after a good shot, for filling in the shot table.
    operator.x().onTrue(Commands.runOnce(shooter::printShotTuningPoint));
    
    //DRIVER: reverse shooter
    driver.povDown().whileTrue(
            Commands.run(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.REVERSEMF), shooter))
        .onFalse(
            Commands.runOnce(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE), shooter));
    // DRIVER: pivot
driver
    .a()
    .whileTrue(
        Commands.run(
            () -> intakePivot.setTeleopOpenLoopPercent(+0.35),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

driver
    .b()
    .whileTrue(
        Commands.run(
            () -> intakePivot.setTeleopOpenLoopPercent(-0.4),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

    // DRIVER: indexer
    //driver.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        

   // driver.y().onTrue(Commands.runOnce(drivetrain::playMusic));

    // OPERATOR CONTROLS............................................................................
   /* operator
        .rightBumper()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed,
                MaxAngularRate,
                true));
*/
   operator
    .y()
    .whileTrue(
        Commands.run(
       /*()-> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW),intakePivot))
        .onFalse(
            Commands.runOnce(
                () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE), intakePivot));

       */
       
       
         () -> intakePivot.setTeleopOpenLoopPercent(+0.30),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

operator
    .a()
    .whileTrue(
        Commands.run(
              /*  ()-> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),intakePivot))
        .onFalse(
            Commands.runOnce(
                () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE), intakePivot));
                */
           () -> intakePivot.setTeleopOpenLoopPercent(-0.30),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

    // OPERATOR: indexer
   /* operator
        .x()
        .whileTrue(
            Commands.run(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX), indexer))
        .onFalse(
            Commands.runOnce(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE), indexer));
*/
    operator
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  indexer.setWantedState(IndexerSubsystem.WantedState.SPIT);
                  intake.setWantedState(IntakeSubsystem.WantedState.SPIT);
                },
                indexer,
                intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
                  intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
                },
                indexer,
                intake));

    operator
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE), intake))
        .onFalse(
            Commands.runOnce(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake));

    // OPERATOR: snapshot hood only, manual RPS remains fixed 50 by ShooterSubsystem
   // operator.back().onTrue(new InstantCommand(() -> shooter.syncManualToCurrentTargets()));

  
    // These still adjust dashboard values, but MANUAL_SHOT is locked to MANUAL_SHOT_RPS.
    

    operator
        .rightStick()
        .whileTrue(
            Commands.run(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.REVERSEMF), shooter))
        .onFalse(
            Commands.runOnce(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE), shooter));
        operator
        .leftBumper()
        .whileTrue(
            Commands.run(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX), indexer))
        .onFalse(
            Commands.runOnce(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE), indexer));

    // OPERATOR: table-based shot prep
    operator
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  shooter.setFeedEnabled(false);
                  shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.setFeedEnabled(false);
                  shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
                }));

// DRIVER: manual shot. No AprilTag auto-aim. Fixed 50 RPS.
    operator
        .povUp()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed,
                MaxAngularRate,
                true));
            
    operator
        .povLeft()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed, MaxAngularRate, true));
            
             operator
        .povRight()
        .whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                intakePivot,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                MaxSpeed,
                MaxAngularRate,
                true));
            }

  public void periodic() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}