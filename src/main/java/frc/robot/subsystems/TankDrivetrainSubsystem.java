package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrivetrainSubsystem extends SubsystemBase {

    private final Motor backLeft;
    private final Motor backRight;
    private final Motor frontLeft;
    private final Motor frontRight;

    private final AHRS navx;

    public TankDrivetrainSubsystem(Motor _backLeftMotor, Motor _backRightMotor, Motor _frontLeftMotor, Motor _frontRightMotor) {
        backLeft = _backLeftMotor;
        backRight = _backRightMotor;
        frontLeft = _frontLeftMotor;
        frontRight = _frontRightMotor;

        // Initialize the NavX on the SPI port
        navx = new AHRS(SPI.Port.kMXP);
    }

    public void SetPower(double backLeftPower, double backRightPower, double frontLeftPower, double frontRightPower) {
        backLeft.SetPower(backLeftPower);
        backRight.SetPower(backRightPower);
        frontLeft.SetPower(frontLeftPower);
        frontRight.SetPower(frontRightPower);
    }

    public void SetPowerBack(double backLeftPower, double backRightPower) {
        backLeft.SetPower(backLeftPower);
        backRight.SetPower(backRightPower);
    }

    public void SetPowerFront(double frontLeftPower, double frontRightPower) {
        frontLeft.SetPower(frontLeftPower);
        frontRight.SetPower(frontRightPower);
    }

    public double[] GetPowerBack() {
        return new double[] { backLeft.GetPower(), backRight.GetPower() };
    }

    public double[] GetPowerFront() {
        return new double[] { frontLeft.GetPower(), frontRight.GetPower() };
    }

    public double[] GetPower() {
        return new double[] { backLeft.GetPower(), backRight.GetPower(), frontLeft.GetPower(), frontRight.GetPower() };
    }

    // Method to get the current heading (yaw) from the NavX
    public double getHeading() {
        return navx.getYaw();
    }

    // Method to get pitch from NavX
    public double getPitch() {
        return navx.getPitch();
    }

    // Method to get roll from NavX
    public double getRoll() {
        return navx.getRoll();
    }

    // Method to reset the NavX heading to zero
    public void resetHeading() {
        navx.zeroYaw();
    }
}