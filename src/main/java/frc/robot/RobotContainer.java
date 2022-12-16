/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.AutoConstants;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  XboxController xboxController = new XboxController(Constants.OIConstants.driverControllerPort);
  XboxController operatorController = new XboxController(Constants.OIConstants.operatorControllerPort);
  Joystick controller = new Joystick(Constants.OIConstants.driverControllerPort);
  Joystick operator = new Joystick(Constants.OIConstants.operatorControllerPort);

  Button  driverA_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_A_Button),
          driverB_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_B_Button),
          driverY_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_Y_Button),
          driverX_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_X_Button),
          driverLeftBumper  = new JoystickButton(controller, Constants.OIConstants.DRIVER_leftBumper),
          driverRightBumper  = new JoystickButton(controller, Constants.OIConstants.DRIVER_rightBumper),

          //driverStartButton = new JoystickButton(joystick, 10),

          //OPERATOR
          operatorStart_Button = new JoystickButton(operator, Constants.OIConstants.OPERATOR_startButton),
          operatorA_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_A_Button),
          operatorB_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_B_Button),
          operatorY_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_Y_Button),
          operatorX_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_X_Button),
          operatorLeftBumper  = new JoystickButton(operator, Constants.OIConstants.OPERATOR_leftBumper),
          operatorRightBumper  = new JoystickButton(operator, Constants.OIConstants.OPERATOR_rightBumper);




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    robotDrive.setDefaultCommand(
            new RunCommand(
                    () -> robotDrive.curvatureDrive(-xboxController.getRawAxis(1), xboxController.getRawAxis(2)),
                    robotDrive)
            );


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating  a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      driverLeftBumper.whileHeld(
              new StartEndCommand(
                      () -> shooterSubsystem.lowerShooter(), () -> shooterSubsystem.stopRotation())
                      .alongWith(
                              new StartEndCommand(() ->shooterSubsystem.intake(), () -> shooterSubsystem.stopShooter())
                      ).alongWith(
                        new StartEndCommand(() -> robotDrive.setMaxOutput(0.2), () -> robotDrive.setMaxOutput(1))
                )
      );
      driverB_Button.whileHeld( new StartEndCommand( () -> shooterSubsystem.raiseShooter(), () -> shooterSubsystem.stopRotation()));
      driverRightBumper.whileHeld( new StartEndCommand(
              () -> shooterSubsystem.outtake(), () -> shooterSubsystem.stopShooter())
              .alongWith(
                      new StartEndCommand(() -> robotDrive.setMaxOutput(0.2), () -> robotDrive.setMaxOutput(1))
              )
      );


  }




    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      var autoVoltageConstraint =
              new DifferentialDriveVoltageConstraint(
                      new SimpleMotorFeedforward(DriveConstants.kS,
                              DriveConstants.kV,
                              DriveConstants.kA),
                      DriveConstants.kinematics,
                      10);

      // Create config for trajectory
      TrajectoryConfig config =
              new TrajectoryConfig(AutoConstants.maxVel,
                      AutoConstants.maxAccel)
                      // Add kinematics to ensure max speed is actually obeyed
                      .setKinematics(DriveConstants.kinematics)
                      // Apply the voltage constraint
                      .addConstraint(autoVoltageConstraint);

/*
      String trajectoryJSON = "paths/trajectory1.wpilib.json";
      String trajectory2JSON = "paths/trajectory2.wpilib.json";

      try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);
        Trajectory traj2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);

        robotDrive.resetOdometry(traj.getInitialPose());

        RamseteCommand ramseteCommand = new RamseteCommand(
                traj,
                robotDrive::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                        Constants.DriveConstants.kV,
                        Constants.DriveConstants.kA), Constants.DriveConstants.kinematics,
                robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPVel, 0, 0),
                // RamseteCommand passes volts to the callback
                robotDrive::setVoltage,
                robotDrive
        );

        RamseteCommand ramseteCommand2 = new RamseteCommand(
                traj2,
                robotDrive::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                        Constants.DriveConstants.kV,
                        Constants.DriveConstants.kA), Constants.DriveConstants.kinematics,
                robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPVel, 0, 0),
                // RamseteCommand passes volts to the callback
                robotDrive::setVoltage,
                robotDrive
        );

        return ramseteCommand.andThen(ramseteCommand2.andThen(() -> robotDrive.setVoltage(0, 0)));

    }
    catch (IOException io){
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, io.getStackTrace());
        return null;
      }
      */



      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(
                      new Translation2d(1, 1),
                      new Translation2d(2, -1)
              ),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              // Pass config
              config
      );

      robotDrive.resetOdometry(exampleTrajectory.getInitialPose());


      RamseteCommand ramseteCommand = new RamseteCommand(
              exampleTrajectory,
              robotDrive::getPose,
              new RamseteController(AutoConstants.ramseteB, AutoConstants.ramseteZeta),
              new SimpleMotorFeedforward(DriveConstants.kS,
                      DriveConstants.kV,
                      DriveConstants.kA),
              DriveConstants.kinematics,
              robotDrive::getWheelSpeeds,
              new PIDController(DriveConstants.kPVel, 0, 0),
              new PIDController(DriveConstants.kPVel, 0, 0),
              // RamseteCommand passes volts to the callback
              robotDrive::tankDriveVolts,
              robotDrive
      );


      return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));

  }

}
