package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A tele-op swerve drive command.
 * Mode: Field-oriented
 * Controls:
 * - Left Joystick
 *   - Left/Right: X Speed
 *   - Up/Down: Y Speed
 * - Right Joystick
 *   - Left/Right: Rotation Speed
 */
public class ManualDrive extends Command {
  private final DriveSubsystem drive;
  private final PS4Controller controller;

  /**
   * Creates a new Manual Drive subsystem for teleop driving.
   */
  public ManualDrive(DriveSubsystem drive, PS4Controller controller) {
    this.drive = drive;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = controller.getLeftX();
    double ySpeed = controller.getLeftY();
    double turnSpeed = controller.getRightX();

    SmartDashboard.putNumber("ManualDrive / X Speed", xSpeed);
    SmartDashboard.putNumber("ManualDrive / Y Speed", ySpeed);
    SmartDashboard.putNumber("ManualDrive / Turn Speed", turnSpeed);

    // Drive field-oriented, with rate limiting
    // turnSpeed is clockwise, I believe.
    drive.drive(xSpeed, ySpeed, turnSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

