// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class FireAmpTimeLimited extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;

  /**
   * When this command started, in milliseconds. 
   */
  private final long startTimeMillis;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FireAmpTimeLimited(Shooter shooter) {
    this.shooter = shooter;
    this.startTimeMillis = System.currentTimeMillis();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //running intake at 50% power
    shooter.shootAmp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopRunning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - this.startTimeMillis) >= AutoConstants.kShooterDurationMillis;
  }
}
