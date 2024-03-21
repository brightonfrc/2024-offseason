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
import frc.robot.Constants.GameSetup;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.CANIds;
// import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Climb;
import frc.robot.commands.EjectNote;
import frc.robot.commands.FireAmp;
import frc.robot.commands.FireAmp;
import frc.robot.commands.FireAmpTimeLimited;
import frc.robot.commands.FireSpeaker;
import frc.robot.commands.FireSpeakerTimeLimited;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeNoteTimeLimited;
import frc.robot.commands.SlowDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final VictorSPX intakeMotor = new VictorSPX(CANIds.kIntakeMotor);
  private final VictorSPX leftShooterMotor = new VictorSPX(CANIds.kLeftShooterMotor);
  private final VictorSPX rightShooterMotor = new VictorSPX(CANIds.kRightShooterMotor);
  private final CANSparkMax liftMotor = new CANSparkMax(12/*CANIds.kLiftMotor*/, MotorType.kBrushless);

  private final Intake intake = new Intake(intakeMotor);
  private final Shooter shooter = new Shooter(leftShooterMotor, rightShooterMotor);
  private final Lift lift = new Lift(liftMotor);

  public boolean slowed = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    intakeMotor.setInverted(true);
    rightShooterMotor.setInverted(true);

    System.out.println("Hello World");
    // PathPlanner Named commands
    NamedCommands.registerCommand("ShootIntoAmp", new ParallelCommandGroup(new FireAmpTimeLimited(shooter), new SequentialCommandGroup(new WaitCommand(1.5), new IntakeNoteTimeLimited(intake))));
    NamedCommands.registerCommand("ShootIntoSpeaker", new ParallelCommandGroup(new FireSpeakerTimeLimited(shooter), new SequentialCommandGroup(new WaitCommand(1.5), new IntakeNoteTimeLimited(intake))));
    NamedCommands.registerCommand("IntakeNote", new IntakeNoteTimeLimited(intake));

    // PathPlanner commands
    // TODO: Set this to correct command
    m_autoCommand = new PathPlannerAuto(GameSetup.pathPlannerAutoStrategy);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                processDriveInput(/* Maybe remove - */-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)) * (slowed ? 0.2 : 1),
                processDriveInput(-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)) * (slowed ? 0.2 : 1),
                processDriveInput(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)) * (slowed ? 0.0125 : 0.025), // Weirdly this gets right stick X
                GameSetup.isFieldRelative, true),
            m_robotDrive));
        // new ManualDrive(m_robotDrive, m_driverController));
  }

  private double processDriveInput(double input) {
    // Make a joystick input input become a swerve input value
    if(input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
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
    System.out.println("Configure Button Bindings");
    m_driverController.cross()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.triangle().whileTrue(new Climb(lift, false));
    m_driverController.circle().whileTrue(new Climb(lift, true));

    m_driverController.square().whileTrue(new SlowDrivetrain(this));

    m_driverController.L1().whileTrue(new IntakeNote(intake));
    m_driverController.R1().whileTrue(new EjectNote(intake));

    m_driverController.L2().whileTrue(new FireSpeaker(shooter));
    m_driverController.R2().whileTrue(new FireAmp(shooter));
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
    // return new ParallelCommandGroup(new FireSpeakerTimeLimited(shooter), new IntakeNoteTimeLimited(intake));
    
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