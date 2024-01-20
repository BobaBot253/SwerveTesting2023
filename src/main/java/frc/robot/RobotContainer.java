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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private Object kinematics;
  private static final XboxController 
    driverController = new XboxController(OIConstants.kDriverControllerPort),
    operatorController = new XboxController(OIConstants.kDriverControllerPort);
  private static final JoystickButton 
    driver_A = new JoystickButton(driverController, 1),
    driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
    driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
    driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
    driver_MENU = new JoystickButton(driverController, 8);
  
  private static final JoystickButton 
    operator_A = new JoystickButton(operatorController, 1),
    operator_B = new JoystickButton(operatorController, 2), operator_X = new JoystickButton(operatorController, 3),
    operator_Y = new JoystickButton(operatorController, 4), operator_LB = new JoystickButton(operatorController, 5),
    operator_RB = new JoystickButton(operatorController, 6), operator_VIEW = new JoystickButton(operatorController, 7),
    operator_MENU = new JoystickButton(operatorController, 8);
  
  public static NetworkTable limelight;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    limelight = NetworkTableInstance.getDefault().getTable("limelight-twoplus");
  
  }

  /**x``
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driver_X
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

            //ryan says hi lol
  }

  public double getAprilID() {
    return limelight.getEntry("tid").getDouble(0);
  }


  public double getAprilTheta() {
    return limelight.getEntry("tx").getDouble(0);
    // return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  // public double getAprilTag (){
  //   return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  // }

  // public static double getXOffset() {
  //   return -limelight.getEntry("tx").getDouble(0);
  // }

  // public static double getYOffset() {
  //   return -limelight.getEntry("ty").getDouble(0);
  // }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

  // public Command getAutonomousCommand() {
  //   // Create config for trajectory - EXAMPLE
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecond,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);

  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       config);

  //   var thetaController = new ProfiledPIDController(
  //       AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       m_robotDrive::getPose, // Functional interface to feed supplier
  //       DriveConstants.kDriveKinematics,

  //       //weeeeeeeeee

  //       // Position controllers
  //       new PIDController(AutoConstants.kPXController, 0, 0),
  //       new PIDController(AutoConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  // }
  }
