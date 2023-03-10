/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.control;

import frc.robot.subsystems.ColorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Spin extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ColorSubsystem subsystem;
    private int num;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Spin(ColorSubsystem subsystem, int spins) {
        this.num = spins;
        this.subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.startSpin();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.rotateWheel();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.endSpin();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem.spinTimes(this.num);
    }
}
