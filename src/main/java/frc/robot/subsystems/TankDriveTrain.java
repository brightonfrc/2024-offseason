package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveTrain extends SubsystemBase{
    
    private Motor leftfrontMotor;
    private Motor leftbackMotor;
    private Motor rightfrontMotor;
    private Motor rightbackMotor;
    private GyroscopeSubsystem navxIMU;

    public TankDriveTrain(VictorSP left1, VictorSP left2, VictorSP right1, VictorSP right2) {
        leftfrontMotor = new Motor(left1);
        leftbackMotor = new Motor(left2);
        rightfrontMotor = new Motor(right1);
        rightbackMotor = new Motor(right2);

        navxIMU = new GyroscopeSubsystem();
    }

    public double getBearing() {
        return navxIMU.getBearing();
    }

    public void setLeftPower(double power) {
        leftfrontMotor.SetMotorPower(power);
        leftbackMotor.SetMotorPower(power);
    }

    public void setRightPower(double power) {
        rightfrontMotor.SetMotorPower(power);
        rightbackMotor.SetMotorPower(power);
    }


}
