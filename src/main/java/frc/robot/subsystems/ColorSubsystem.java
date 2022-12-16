package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSubsystem extends SubsystemBase {

    private final WPI_VictorSPX raiseMotor = new WPI_VictorSPX(Constants.ControlConstants.raiseMotorPort);
    private final WPI_VictorSPX turnMotor = new WPI_VictorSPX(Constants.ControlConstants.turnMotorPort);

    private final DigitalInput topLimitSwitch = new DigitalInput(Constants.ControlConstants.topLimitSwitchPort);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ControlConstants.bottomLimitSwitchPort);

    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private final Color blue;
    private final Color green;
    private final Color red;
    private final Color yellow;
    private Color detectedColor;
    private ColorMatchResult match;
    private static String colorString;

    private int currentRotations;
    String lastColor;

    public ColorSubsystem(){
        I2C.Port i2cPort = I2C.Port.kOnboard;
        m_colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher = new ColorMatch();
        blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
        green = ColorMatch.makeColor(0.197, 0.561, 0.240);
        red = ColorMatch.makeColor(0.561, 0.232, 0.114);
        yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);

        m_colorMatcher.addColorMatch(blue);
        m_colorMatcher.addColorMatch(green);
        m_colorMatcher.addColorMatch(red);
        m_colorMatcher.addColorMatch(yellow);

        detectedColor =  m_colorSensor.getColor();
        match = m_colorMatcher.matchClosestColor(detectedColor);

    }

    @Override
    public void periodic(){
        detectedColor = m_colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color
         */
        match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == blue) {
            colorString = "Blue";
        } else if (match.color == red) {
            colorString = "Red";
        } else if (match.color == green) {
            colorString = "Green";
        } else if (match.color == yellow) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);

    }

    public String getColor(){
        return colorString;
    }

    public void rotateWheel(){
        if(!lastColor.equals(colorString)) {
            currentRotations++;
        }
        lastColor = colorString;
    }


    public void raiseArm(){
        raiseMotor.set(0.8);
    }

    public void lowerArm(){
        raiseMotor.set(-0.8);
    }

    public void stopArm(){
        raiseMotor.set(0);
    }

    public boolean getTopLimitSwitch(){
        return topLimitSwitch.get();
    }

    public boolean getBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }

    public void reset(){
        this.currentRotations = 0;
    }


    public void startSpin() {
        turnMotor.set(1);
    }

    public void endSpin(){
        turnMotor.set(0);
    }

    public boolean spinTimes(int num){
        if(currentRotations >= num * 8){
            currentRotations = 0;
            return true;
        }
        else return false;
    }
}
