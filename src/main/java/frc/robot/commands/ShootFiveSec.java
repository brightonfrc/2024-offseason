package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;
import frc.robot.Constants.ShootMPower;

public class ShootFiveSec extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Motor motor;
    private float startTimeMillis;

    public ShootFiveSec(Motor motor) {

        this.motor = motor;

        
        addRequirements(motor);

    }
        
    @Override
    public void initialize() {
        startTimeMillis = System.currentTimeMillis();
    }    
        
    @Override
    public void execute() {
        motor.SetPower(ShootMPower.shootPower);
    }

    @Override
    public void end(boolean interrupted) {
        motor.SetPower(0);
    }

    @Override
    public boolean isFinished() {
        if(System.currentTimeMillis() - startTimeMillis >= ShootMPower.shootTime) {
            return true;
        }
        else {
            return false;
        }
    }
}
