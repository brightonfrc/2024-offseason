package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;
import frc.robot.Constants.ShootConstants;

public class ShootMotor extends Command {
    private final Motor motor;
    private long startTimeMilli;
    
    public ShootMotor (Motor motor) {
        this.motor = motor;

        addRequirements(motor);
    }

    @Override
  public void initialize() {
    startTimeMilli = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    motor.SetMotorPower(ShootConstants.shootMotorPower);
  }

  @Override
  public void end(boolean interrupted) {
    motor.SetMotorPower(0);
    
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() - startTimeMilli >= ShootConstants.shootMotorTime) {
        return true;
    }
    return false;
  }
}
