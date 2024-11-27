package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveTrain;

public class DriveCommand extends Command {
    
    private final TankDriveTrain tankDriveTrain;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
    private final PIDController pidController;

    public DriveCommand(TankDriveTrain tankDriveTrain, Joystick leftJoystick, Joystick rightJoystick) {
        this.tankDriveTrain = tankDriveTrain;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;

        pidController = new PIDController(0.02, 0.0, 0.0);
        pidController.enableContinuousInput(0, 360); 
        pidController.setTolerance(2);
        addRequirements(tankDriveTrain);
    }


    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double forwardPower = leftJoystick.getY();
        
        double rightX = rightJoystick.getX();
        double rightY = -rightJoystick.getY();
        double joystickBearing = Math.toDegrees(Math.atan2(rightY, rightX));
        if (joystickBearing < 0){
            joystickBearing += 360;
        }

        double currentBearing = tankDriveTrain.getBearing();
    
        double turnPower = pidController.calculate(currentBearing, joystickBearing);

        double leftPower = forwardPower - turnPower;
        double rightPower = forwardPower + turnPower;
        
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0){
            leftPower /= maxPower;
            rightPower /= maxPower;
        }
        
        
        tankDriveTrain.setLeftPower(leftPower);
        tankDriveTrain.setRightPower(rightPower);
    }

    @Override
    public void end(boolean interrupted) {
        tankDriveTrain.setLeftPower(0);
        tankDriveTrain.setRightPower(0);
    }


    @Override
  public boolean isFinished() {
    return false;
  }

}
