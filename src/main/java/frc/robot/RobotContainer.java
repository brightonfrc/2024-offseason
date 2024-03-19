// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Ports;
// import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Climb;
import frc.robot.commands.EjectNote;
import frc.robot.commands.FireAmp;
import frc.robot.commands.IntakeNote;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PathPlannerAuto m_autoCommand;

  // The driver's controller
  CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);

  private final VictorSPX intakeMotor = new VictorSPX(Ports.kIntakeMotorPort);
  private final VictorSPX leftShooterMotor = new VictorSPX(Ports.kLeftShooterMotor);
  private final VictorSPX rightShooterMotor = new VictorSPX(Ports.kRightShooterMotor);
  private final CANSparkMax liftMotor = new CANSparkMax(Ports.kRightShooterMotor, MotorType.kBrushless);

  private final Intake intake = new Intake(intakeMotor);
  private final Shooter shooter = new Shooter(leftShooterMotor, rightShooterMotor);
  private final Lift lift = new Lift(liftMotor);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PathPlanner Named commands
    // NamedCommands.registerCommand("ShootIntoAmp", new ShootIntoAmp()); // TODO: Create this command
    // PathPlanner commands
    // TODO: Set this to correct command
    m_autoCommand = new PathPlannerAuto("Leave Left");

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getR2Axis(), OIConstants.kDriveDeadband), // Weirdly this gets right stick X
                true, true),
            m_robotDrive));
        // new ManualDrive(m_robotDrive, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.cross()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.triangle().whileTrue(new Climb(lift));

    m_driverController.L1().whileTrue(new IntakeNote(intake));
    m_driverController.R1().whileTrue(new EjectNote(intake));

    m_driverController.L2().whileTrue(new FireAmp(shooter));
  }

  
  // /**
  //  * Use this method to define your trigger->command mappings. Triggers can be created via the
  //  * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
  //  * predicate, or via the named factories in {@link
  //  * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
  //  * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
  //  * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
  //  * joysticks}.
  //  */
  // private void configureBindings() {


  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}