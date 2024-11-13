package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrivetrainSubsystem extends SubsystemBase{

    private final Motor backLeft;
    private final Motor backRight;
    private final Motor frontLeft;
    private final Motor frontRight;

    public TankDrivetrainSubsystem(Motor _backLeftMotor, Motor _backRightMotor, Motor _frontLeftMotor, Motor _frontRightMotor){
        backLeft = _backLeftMotor;
        backRight = _backRightMotor;
        frontLeft = _frontLeftMotor;
        frontRight = _frontRightMotor;
    }

    public void SetPower(double backLeftPower, double backRightPower, double frontLeftPower, double frontRightPower){
        backLeft.SetPower(backLeftPower);
        backRight.SetPower(backRightPower);
        frontLeft.SetPower(frontLeftPower);
        frontRight.SetPower(frontRightPower);
    }

    public double[] GetPower()
    {
        double[] temp = new double[4];
        temp[0] = backLeft.GetPower();
        temp[1] = backRight.GetPower();
        temp[2] = frontLeft.GetPower();
        temp[3] = frontRight.GetPower();
        return temp;
    }
}