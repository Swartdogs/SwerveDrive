package frc.robot.drive.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.drive.Drive;

public class CmdDriveWithJoystick extends CommandBase
{
    private Drive           _drive;
    private DoubleSupplier  _driveInput;
    private DoubleSupplier  _strafeInput;
    private DoubleSupplier  _rotateInput;
    private BooleanSupplier _isAbsolute;

    public CmdDriveWithJoystick(SubsystemContainer subsystemContainer, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier rotateInput, BooleanSupplier isAbsolute)
    {
        _drive       = subsystemContainer.getDrive();
        _driveInput  = driveInput;
        _strafeInput = strafeInput;
        _rotateInput = rotateInput;
        _isAbsolute  = isAbsolute;

        addRequirements(_drive);
    }

    @Override
    public void execute() 
    {
        _drive.drive(_driveInput.getAsDouble(), _strafeInput.getAsDouble(), _rotateInput.getAsDouble(), _isAbsolute.getAsBoolean());
        // System.out.println(_drive.getOdometer());
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
