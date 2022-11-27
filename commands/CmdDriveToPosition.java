package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.RobotLog;
import frc.robot.drive.Drive;
import frc.robot.drive.Vector;

public class CmdDriveToPosition extends CommandBase 
{
    private Drive    _drive;
    private RobotLog _log;
    
    private Vector   _finalTranslation;
    private double   _finalHeading;

    private double  _maxRotateSpeed;
    private double  _maxDriveSpeed;
    private double  _minDriveSpeed;

    private double  _distanceThreshold;
    private double  _maxCloseDriveSpeed;

    private boolean  _absolute;

    public CmdDriveToPosition(SubsystemContainer subsystemContainer, Vector finalPosition, double finalRotation, double maxRotateSpeed, double distanceThreshold, double maxDriveSpeed, double maxCloseDriveSpeed, double minDriveSpeed, boolean absolute)
    {
        _drive              = subsystemContainer.getDrive();
        _log                = subsystemContainer.getRobotLog();
        
        _finalTranslation   = finalPosition;
        _finalHeading       = finalRotation;

        _maxRotateSpeed     = maxRotateSpeed;
        _distanceThreshold  = distanceThreshold;
        _maxDriveSpeed      = maxDriveSpeed;
        _maxCloseDriveSpeed = maxCloseDriveSpeed;
        _minDriveSpeed      = minDriveSpeed;

        _absolute           = absolute;

        addRequirements(_drive);
    }

    public CmdDriveToPosition(SubsystemContainer subsystemContainer, Vector finalPosition, double finalRotation, double maxRotateSpeed, double maxDriveSpeed, double minDriveSpeed, boolean absolute)
    {
        this(subsystemContainer, finalPosition, finalRotation, maxRotateSpeed, 0, maxDriveSpeed, maxDriveSpeed, minDriveSpeed, absolute);
    }

    @Override
    public void initialize()
    {
        Vector translation = _finalTranslation;

        if (!_absolute)
        {
            translation = translation.add(_drive.getOdometer());
            _finalHeading += _drive.getHeading();
        }
        
        _drive.translateInit(translation, _distanceThreshold, _maxDriveSpeed, _maxCloseDriveSpeed, _minDriveSpeed, false);
        _drive.rotateInit(_finalHeading, _maxRotateSpeed);

        _log.log(String.format("Starting Drive to Position; Target Position: %s, Current Position: %s, Target Rotation: %6.2f, Current Rotation: %6.2f", translation.toString(), _drive.getOdometer().toString(), _finalHeading, _drive.getHeading()));
    }

    @Override
    public void execute() 
    {   
        double rotateSpeed = _drive.rotateExec();
        Vector translateVector = _drive.translateExec();
        
        _drive.drive(translateVector, rotateSpeed, true);
    }

    @Override
    public void end(boolean interrupted)
    {
        _drive.drive(0, 0, 0);

        _log.log(String.format("Drive to Position complete; Current Position: %s, Current Rotation: %6.2f", _drive.getOdometer().toString(), _drive.getHeading()));
    }

    @Override
    public boolean isFinished() 
    {
        return _drive.translateIsFinished() && _drive.rotateIsFinished();
    }
}
