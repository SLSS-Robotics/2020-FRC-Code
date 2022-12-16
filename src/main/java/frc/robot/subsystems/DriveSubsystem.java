package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

    SpeedController leftTalonSRX = new WPI_TalonSRX(Constants.DriveConstants.leftMotor1Port);
    SpeedController leftVictorSPX = new WPI_VictorSPX(Constants.DriveConstants.leftMotor2Port);
  
    private final SpeedControllerGroup leftMotors =
        new SpeedControllerGroup(leftTalonSRX,leftVictorSPX);
  
    // The motors on the right side of the drive.
    SpeedController rightTalonSRX = new WPI_TalonSRX(Constants.DriveConstants.rightMotor1Port);
    SpeedController rightVictorSPX = new WPI_VictorSPX(Constants.DriveConstants.rightMotor2Port);
    private final SpeedControllerGroup rightMotors =
        new SpeedControllerGroup(rightTalonSRX,rightVictorSPX);
  
    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);


  
  // The left-side drive encoder
  private final Encoder leftEncoder =
      new Encoder(Constants.DriveConstants.leftEncoderPortA, Constants.DriveConstants.leftEncoderPortB,
                  Constants.DriveConstants.leftEncoderReversed);

  // The right-side drive encoder
  private final Encoder rightEncoder =
      new Encoder(Constants.DriveConstants.rightEncoderPortA, Constants.DriveConstants.rightEncoderPortB,
                  Constants.DriveConstants.rightEncoderReversed);

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;



    public DriveSubsystem(){
        leftEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistPerPulse);
        rightEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistPerPulse);
    
        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("l_encoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("r_encoder", rightEncoder.getDistance());
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
    }

    public void curvatureDrive(double fwd, double rot){
        drive.arcadeDrive(fwd, rot);
    }
 /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (Constants.DriveConstants.gyroReversed ? -1.0 : 1.0);
  }
}
