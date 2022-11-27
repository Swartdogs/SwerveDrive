package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.RobotLog;
import frc.robot.drive.Drive;

public class CmdDriveRotate extends CommandBase
{
    private Drive    _drive;
    private RobotLog _log;

    private double   _heading;
    private double   _maxRotateSpeed;

    private boolean  _absolute;

    public CmdDriveRotate(SubsystemContainer subsystemContainer, double heading, double maxRotateSpeed, boolean absolute)
    {
        _drive          = subsystemContainer.getDrive();
        _log            = subsystemContainer.getRobotLog();

        _heading        = heading;
        _maxRotateSpeed = maxRotateSpeed;
        _absolute       = absolute;
    }

    @Override
    public void initialize()
    {
        double heading = _heading;
        
        if (!_absolute)
        {
            heading += _drive.getHeading();
        }
        
        _drive.rotateInit(heading, _maxRotateSpeed);
        
        _log.log(String.format("Starting Drive Rotate; Target Heading: %6.2f, Current Heading: %6.2f, Max Speed: %5.2f", heading, _drive.getHeading(), _maxRotateSpeed));
    }

    @Override
    public void execute()
    {
        _drive.drive(0, 0, _drive.rotateExec());
    }

    @Override
    public void end(boolean interrupted)
    {
        _drive.drive(0, 0, 0);
        
        _log.log(String.format("Rotate Complete, Final Heading: %6.2f", _drive.getHeading()));
    }

    @Override
    public boolean isFinished()
    {
        return _drive.rotateIsFinished();
    }
}
