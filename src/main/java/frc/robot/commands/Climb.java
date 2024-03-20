// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Lift lift;
  private boolean reverse;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Climb (Lift lift, boolean reverse) {
    this.reverse = reverse;
    this.lift = lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climb Initialise");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Climb Exec");
    if(this.reverse) {
      lift.setSpeed(-0.15);
    } else {
      lift.setSpeed(0.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}