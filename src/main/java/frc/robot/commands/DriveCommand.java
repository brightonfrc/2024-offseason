// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TankDrivetrainSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDrivetrainSubsystem tankDrivetrainSubsystem;
  private final PS4Controller ps4Controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(TankDrivetrainSubsystem _tankDrivetrainSubsystem, PS4Controller _ps4Controller) {
    tankDrivetrainSubsystem = _tankDrivetrainSubsystem;
    ps4Controller = _ps4Controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_tankDrivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Left joystick for speed
    double leftX = ps4Controller.getLeftX();
    double leftY = -ps4Controller.getLeftY(); // Invert Y to make forward positive
    double speed = Math.hypot(leftX, leftY);  // Magnitude of the left joystick

    // Right joystick for bearing
    double rightX = ps4Controller.getRightX();
    double rightY = -ps4Controller.getRightY(); // Invert Y to match field orientation
    double desiredBearing = Math.toDegrees(Math.atan2(rightY, rightX)); // Angle in degrees

    // If the right joystick is near the center, keep the current heading
    if (Math.hypot(rightX, rightY) < 0.1) { // Deadband for right stick
        desiredBearing = tankDrivetrainSubsystem.getHeading();
    }

    // Current heading from NavX
    double currentHeading = tankDrivetrainSubsystem.getHeading();

    // Calculate the turn power based on heading error
    double headingError = desiredBearing - currentHeading;
    headingError = normalizeAngle(headingError); // Normalize to -180 to 180 degrees
    double turnPower = headingError * 0.02; // Proportional control constant (tune this value)

    // Left and right motor powers for turning and speed
    double leftPower = speed - turnPower;
    double rightPower = speed + turnPower;

    // Set power to drivetrain
    tankDrivetrainSubsystem.SetPower(leftPower, rightPower, leftPower, rightPower);
  }

  /**
  * Normalize an angle to the range -180 to 180 degrees.
  */
  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
