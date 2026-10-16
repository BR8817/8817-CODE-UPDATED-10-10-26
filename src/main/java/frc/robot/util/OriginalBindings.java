//changing controllers for Off season drivers
/*driver
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
        .povDown()
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

    // DRIVER: manual hood trim
    driver.povRight().whileTrue(Commands.run(() -> shooter.adjustManualHood(+0.01), shooter));
    driver.povLeft().whileTrue(Commands.run(() -> shooter.adjustManualHood(-0.01), shooter));

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
    driver
        .x()
        .whileTrue(
            Commands.run(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX), indexer))
        .onFalse(
            Commands.runOnce(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE), indexer));

   // driver.y().onTrue(Commands.runOnce(drivetrain::playMusic));

    // OPERATOR: pivot
   // OPERATOR: pivot
operator
    .y()
    .whileTrue(
        Commands.run(
            () -> intakePivot.setTeleopOpenLoopPercent(+0.2),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

operator
    .a()
    .whileTrue(
        Commands.run(
            () -> intakePivot.setTeleopOpenLoopPercent(-0.2),
            intakePivot))
    .onFalse(
        Commands.runOnce(
            () -> intakePivot.stopOpenLoop(),
            intakePivot));

    // OPERATOR: indexer
    operator
        .x()
        .whileTrue(
            Commands.run(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX), indexer))
        .onFalse(
            Commands.runOnce(
                () -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE), indexer));
 */