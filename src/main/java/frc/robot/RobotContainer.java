// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.resetGyro;
import frc.robot.subsystems.Climber_Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake_subsystem;
import frc.robot.subsystems.Shooter_subsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter_subsystem mShooter_subsystem = new Shooter_subsystem();
  private final Intake_subsystem mIntake_subsystem = new Intake_subsystem();
  private final Climber_Subsystem mClimber_Subsystem = new Climber_Subsystem();
  private final AnalogPotentiometer sensor = new AnalogPotentiometer(3);
  //private final climber_group Clibm
  ;//private final HashMap autoEventHashMap = new OIConstants.autoEventMap;

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //var updateSmartDashBoard= new RunCommand();

    // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));
            //}
        //mIntake_subsystem.setDefaultCommand(new RunCommand(()->mIntake_subsystem., null));
    mShooter_subsystem.setDefaultCommand(
        new ShootCommand(mShooter_subsystem, ()-> m_driverController.getRawAxis(3), -100));
    
    mIntake_subsystem.setDefaultCommand(
        new intakeCommand(mIntake_subsystem, ()-> m_driverController.getRawAxis(3), -100, ()-> sensor.get()));
        //, ()-> sensor.get()
    //mClimber_Subsystem.setDefaultCommand(
      //new CommandGroup()
    //);

    
    NamedCommands.registerCommand("ShootCommand", new ShootCommand(mShooter_subsystem, ()-> m_driverController.getRawAxis(3), 1));
        autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);
  }

protected void execute() {
    SmartDashboard.putNumber("Controller_pos", m_driverController.getRightX());
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
    //new JoystickButton(m_driverController, Button.kR1.value)
        //.whileTrue(new RunCommand(9
          //  () -> m_robotDrive.setX(),
            //m_robotDrive));
    //m_driverController.y().whileTrue(new m_robotDrive.setX());
   /*  
    new JoystickButton(m_driverController, 5). whileTrue(new ShootCommand(
      mShooter_subsystem, null, 0.5));

    
    
    new JoystickButton(m_driverController, 6). whileTrue(new ShootCommand(
      mShooter_subsystem, null, -0.5));
    */
    new JoystickButton(m_driverController, 5).onTrue(new intakeCommand(mIntake_subsystem,
     ()-> m_driverController.getRawAxis(3), -1, ()-> sensor.get()));
//() -> sensor.get()

  // The getRawAxis on here is to identify if the shooter is being used.
    new JoystickButton(m_driverController, 6).onTrue(new intakeCommand(
      mIntake_subsystem, ()-> m_driverController.getRawAxis(3), 1, ()-> sensor.get()));
    new JoystickButton(m_driverController, 1).whileTrue(new ShootCommand(mShooter_subsystem, null, 0.3));
    new JoystickButton(m_driverController, 3).onTrue(new resetGyro(m_robotDrive));
    new JoystickButton(m_driverController, 2).whileTrue(new ClimbCommand(mClimber_Subsystem, true, ()-> m_driverController.getRawAxis(6), -100));
    new JoystickButton(m_driverController, 4).whileTrue(new ClimbCommand(mClimber_Subsystem, true, ()-> m_driverController.getRawAxis(6), -100));
    // NOTE THAT -100 REPRESENTS NOT USING THE CONSTANT SPEED VAR
    //, () -> sensor.get()

    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    //return autoChooser.getSelected();
    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    */
  }
  //private void configureAutoCommands(){
    //ArrayList<PathPlannerTrajectory> autoPaths = PathPlanner

  //}
}
