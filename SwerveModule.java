package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import PIDControl.PIDControl;

import frc.robot.abstraction.Motor;
import frc.robot.abstraction.PositionSensor;

public class SwerveModule extends Vector
{
    private Motor           _driveMotor;
    private Motor           _rotateMotor;
    private PositionSensor  _positionSensor;

    private PIDControl      _rotatePID;
    private double          _rotatePIDSetpoint;

    private DoubleSupplier  _relativeZeroSupplier;

    public SwerveModule(Motor          driveMotor, 
                        Motor          rotateMotor, 
                        PositionSensor positionSensor, 
                        PIDControl     rotatePID, 
                        double         x, 
                        double         y,
                        DoubleSupplier relativeZero)
    {
        super(x, y);

        _driveMotor           = driveMotor;
        _rotateMotor          = rotateMotor;
        _positionSensor       = positionSensor;

        _rotatePID            = rotatePID;
        _rotatePIDSetpoint    = 0;

        _relativeZeroSupplier = relativeZero;
    }

    public void drive(Vector moduleCommand)
    {
        double driveSpeed  = moduleCommand.getR();
        _rotatePIDSetpoint = moduleCommand.getTheta();
        double position    = getPIDPosition();

        if (Math.abs(_rotatePIDSetpoint - position) > 90)
        {
            driveSpeed *= -1;

            if (_rotatePIDSetpoint < 180)
            {
                _rotatePIDSetpoint += 180;
            }

            else
            {
                _rotatePIDSetpoint -= 180;
            }

            position = getPIDPosition();
        }

        _rotatePID.setSetpoint(_rotatePIDSetpoint, position);
        _rotateMotor.set(_rotatePID.calculate(position));

        _driveMotor.set(driveSpeed);
    }

    public double getPosition()
    {
        return _positionSensor.get() - _relativeZeroSupplier.getAsDouble();
    }

    public double getPIDPosition()
    {
        double position = getPosition();

        if ((_rotatePIDSetpoint - position) > 180)
        {
            position += 360;
        }

        else if ((_rotatePIDSetpoint - position) < -180)
        {
            position -= 360;
        }

        return position;
    }

    public double getSetpoint()
    {
        return _rotatePIDSetpoint;
    }

    public Motor getDriveMotor()
    {
        return _driveMotor;
    }

    public Motor getRotateMotor()
    {
        return _rotateMotor;
    }

    public PositionSensor getPositionSensor()
    {
        return _positionSensor;
    }

    public PIDControl getRotatePID()
    {
        return _rotatePID;
    }

    public double getRelativeZero()
    {
        return _relativeZeroSupplier.getAsDouble();
    }
}
