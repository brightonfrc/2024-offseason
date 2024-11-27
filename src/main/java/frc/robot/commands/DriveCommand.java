package frc.robot.commands;

import frc.robot.subsystems.TankDrivetrainSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
  private final TankDrivetrainSubsystem tankDrivetrainSubsystem;
  private final PS4Controller ps4Controller;
  private PIDController turnPID;
  private double oldSetpoint;

  public DriveCommand(TankDrivetrainSubsystem _tankDrivetrainSubsystem, PS4Controller _ps4Controller) {
    tankDrivetrainSubsystem = _tankDrivetrainSubsystem;
    ps4Controller = _ps4Controller;

    // Declare subsystem dependencies
    addRequirements(_tankDrivetrainSubsystem);

    
  }

  @Override
  public void initialize() {
    turnPID = new PIDController(0, 0, 0);
  }

  @Override
  public void execute() {
    // Left joystick for speed
    double leftX = ps4Controller.getLeftX();
    double leftY = -ps4Controller.getLeftY();
    double speed = Math.hypot(leftX, leftY); // Magnitude of left joystick

    // Right joystick for bearing
    double rightX = ps4Controller.getRightX();
    double rightY = -ps4Controller.getRightY();
    double desiredBearing = Math.toDegrees(Math.atan2(rightY, rightX)); // Calculate bearing

    if (Math.abs(desiredBearing-oldSetpoint)>10){ // If there is a large enough difference (10°) between the old and new bearings, then make the change
      turnPID.setSetpoint(desiredBearing);
      oldSetpoint=desiredBearing;
      turnPID.reset();
    }
    // Get current heading
    double currentHeading = tankDrivetrainSubsystem.getHeading();

    // Calculate turn power with PID
    double turnPower = turnPID.calculate(currentHeading);

    double leftPower = speed;
    double rightPower = speed;

    if (turnPower != 0){ // If still need to turn, 
      leftPower = turnPower;
      rightPower = -turnPower;
    }

    // Set motor power
    tankDrivetrainSubsystem.SetPower(leftPower, rightPower, leftPower, rightPower);
  }

  @Override
  public void end(boolean interrupted) {
    tankDrivetrainSubsystem.SetPower(0, 0, 0, 0); // Stop motors
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}