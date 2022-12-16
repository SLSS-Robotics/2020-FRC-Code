package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonSRX rotationMotor = new WPI_TalonSRX(Constants.ShooterConstants.rotationMotorPort);
    private WPI_VictorSPX shooterMotor =  new WPI_VictorSPX(Constants.ShooterConstants.shooterMotorPort);
    private DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ShooterConstants.lowerLimitSwitchPort);
    private DigitalInput upperLimitSwitch = new DigitalInput(Constants.ShooterConstants.upperLimitSwitchPort);


    @Override
    public void periodic(){

    }


    public void outtake(){
        shooterMotor.set(-1);
    }

    public void intake(){
        shooterMotor.set(1);
    }

    public void raiseShooter() {
        rotationMotor.set(0.5);
    }
    public void lowerShooter(){
        rotationMotor.set(-0.5);
    }

    public void stopRotation(){
        rotationMotor.set(0);
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }

    public boolean getUpperLimitSwitch(){return upperLimitSwitch.get();}
    public boolean getLowerLimitSwitch(){return lowerLimitSwitch.get();}

}
