/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
  /**
   * Creates a new VisionCommand.
   */
  private DriveSubsystem drive;
  private VisionSubsystem vision;
  public Joystick joystick;
  double distanceOffset;
  double angleOffset;
  //Error values for the control loop
  double rotationError;
  double distanceError;
  boolean isFinished;

  //Control loop constants
  /*
      This example uses proportional control loop with constant force
      After you master proportional control use might want to try PID control loop
  */
  double KpRot=-0.1;
  double KpDist=-0.1;

  //Deadzone is necessary because the robot can only get so accurate and cannot be pefectly head on the target
  double angleTolerance=5;//Deadzone for the angle control loop
  double distanceTolerance=5;//Deadzone for the distance control loop

  /*
  There is a minimum power that you need to give to the drivetrain in order to overcome friction
  It helps the robot move and rotate at low speeds
  */
  double constantForce=0.05;

  /*
  rotationAjust is rotational signal for the drivetrain
  distanceAjust is forward signal for the drivetrain
  */
  double rotationAjust;
  double distanceAjust;


 
  
  public VisionCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    drive = driveSubsystem;
    vision = visionSubsystem;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.toggleSolenoid(true);
    vision.setDriverMode(true);
    distanceOffset = vision.x_offset();
    angleOffset = vision.angle_offset();
    drive(distanceOffset, angleOffset);
    vision.setDriverMode(false);
    vision.toggleSolenoid(false);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }



  public void drive(double rotationError, double distanceError){
      rotationAjust=0;
      distanceAjust=0;
    /*
        Fetches the rotation and distance values from the vision co processor
        sets the value to 0.0 if the value doesnt exist in the database
    */
    this.rotationError=rotationError;
    this.distanceError=distanceError;

    /*
        Proportional (to targetX) control loop for rotation
        Deadzone of angleTolerance
        Constant power is added to the direction the control loop wants to turn (to overcome friction)
    */
    if(rotationError>angleTolerance)
        rotationAjust=KpRot*rotationError+constantForce;
    else
        if(rotationError<angleTolerance)
            rotationAjust=KpRot*rotationError-constantForce;
    /*
        Proportional (to targetY) control loop for distance
        Deadzone of distanceTolerance
        Constant power is added to the direction the control loop wants to turn (to overcome friction)
    */
    if(distanceError>distanceTolerance)
        distanceAjust=KpDist*distanceError+constantForce;
    else
        if(distanceError<distanceTolerance)
            distanceAjust=KpDist*distanceError-constantForce;


    //Output the power signals to a arcade drivetrain
    drive.arcadeDrive(distanceAjust,rotationAjust);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

}