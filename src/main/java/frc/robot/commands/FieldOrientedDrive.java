// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.kauailabs.navx.frc.AHRS;

/** An example command that uses an example subsystem. */
public class FieldOrientedDrive extends Command {
    private DriveSubsystem driveSubsystem;
    private CommandJoystick joystick;
    private AHRS gyro;
    private PIDController bearingPIDController;

    private double goalBearing;
    private double joystickTurnBearing;
    private double joystickMoveMagnitude;
    private double joystickMoveBearing;
    private double robotBearing;
    private double rotSpeed;
    private double xSpeed;
    private double ySpeed;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FieldOrientedDrive(DriveSubsystem driveSubsystem, CommandJoystick joystick) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        gyro = new AHRS(SPI.Port.kMXP);
        bearingPIDController = new PIDController(FieldOrientedDriveConstants.kFODP, FieldOrientedDriveConstants.kFODI, FieldOrientedDriveConstants.kFODD);
        //setting a tolerance of 2 degrees
        bearingPIDController.setTolerance(Math.PI/90);
        bearingPIDController.setSetpoint(0);
        bearingPIDController.enableContinuousInput(0, 2*Math.PI);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //The robot refuses to stafe while it is being commanded to turn. 
        SmartDashboard.putNumber("Goal bearing", goalBearing);

        
        joystickTurnBearing=joystick.getTwist();
        SmartDashboard.putNumber("Turn: Joystick twist", joystickTurnBearing);

        //error tolerance of 2 degrees
        if (Math.abs(joystickTurnBearing-goalBearing)>Math.PI/180*FieldOrientedDriveConstants.bearingTolerance){
            goalBearing=joystickTurnBearing;
            bearingPIDController.reset();
            bearingPIDController.setSetpoint(goalBearing);
        }
        robotBearing=gyro.getAngle()%360;
        //converting to radians
        robotBearing=robotBearing/180*Math.PI;
        SmartDashboard.putNumber("Robot bearing", robotBearing);

        joystickMoveBearing=joystick.getDirectionRadians();
        SmartDashboard.putNumber("Drive: Joystick bearing", joystickMoveBearing);

        joystickMoveBearing=joystickMoveBearing-robotBearing;
        SmartDashboard.putNumber("Drive: Robot Relative bearing", joystickMoveBearing);

        joystickMoveMagnitude=joystick.getMagnitude();
        SmartDashboard.putNumber("Drive: Joystick magnitude", joystickMoveMagnitude);

        xSpeed=joystickMoveMagnitude*Math.cos(joystickMoveBearing)*TestingConstants.maximumSpeed;
        SmartDashboard.putNumber("xSpeed", xSpeed);

        ySpeed=joystickMoveMagnitude*Math.sin(joystickMoveBearing)*TestingConstants.maximumSpeed;
        SmartDashboard.putNumber("ySpeed", ySpeed);

        rotSpeed=bearingPIDController.calculate(robotBearing)*FieldOrientedDriveConstants.rotationScalar*TestingConstants.maximumSpeed;
        SmartDashboard.putNumber("rotSpeed", rotSpeed);

        // Uncomment when joystick drift is resolved
        // driveSubsystem.drive(ySpeed, xSpeed, rotSpeed, false, true);
        driveSubsystem.drive(xSpeed,ySpeed, rotSpeed, false, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
