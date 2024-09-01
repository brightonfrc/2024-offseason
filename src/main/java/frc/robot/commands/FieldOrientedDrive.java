// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.FieldOrientedDriveConstants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.kauailabs.navx.frc.AHRS;

/** An example command that uses an example subsystem. */
public class FieldOrientedDrive extends Command {
    private DriveSubsystem driveSubsystem;
    private CommandPS4Controller ps4Controller;
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
    public FieldOrientedDrive(DriveSubsystem driveSubsystem, CommandPS4Controller ps4Controller) {
        this.driveSubsystem = driveSubsystem;
        this.ps4Controller = ps4Controller;
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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // I can't remember which one of the joysticks were swapped
        joystickTurnBearing=Math.atan2(ps4Controller.getRightY(), ps4Controller.getRightX());
        SmartDashboard.putNumber("Right Joystick bearing", joystickTurnBearing);

        //error tolerance of 2 degrees
        if (Math.abs(joystickTurnBearing-goalBearing)>Math.PI/90){
            goalBearing=joystickTurnBearing;
            bearingPIDController.reset();
            bearingPIDController.setSetpoint(goalBearing);
        }
        robotBearing=gyro.getAngle()%360;
        //converting to radians
        robotBearing=robotBearing/180*Math.PI;
        SmartDashboard.putNumber("Robot bearing", robotBearing);
        joystickMoveBearing=Math.atan2(ps4Controller.getLeftY(), ps4Controller.getLeftX());
        SmartDashboard.putNumber("Left joystick bearing", joystickMoveBearing);
        joystickMoveBearing=joystickMoveBearing-robotBearing;
        SmartDashboard.putNumber("Robot Relative bearing", joystickMoveBearing);
        joystickMoveMagnitude=Math.pow(Math.pow(ps4Controller.getLeftX(),2)+Math.pow(ps4Controller.getLeftY(),2), 0.5);
        SmartDashboard.putNumber("Left joystick magnitude", joystickMoveMagnitude);
        xSpeed=joystickMoveMagnitude*Math.cos(joystickMoveBearing);
        ySpeed=joystickMoveMagnitude*Math.sin(joystickMoveBearing);
        rotSpeed=bearingPIDController.calculate(robotBearing)*FieldOrientedDriveConstants.rotationScalar;
        driveSubsystem.drive(ySpeed, xSpeed, rotSpeed, false, true);
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
