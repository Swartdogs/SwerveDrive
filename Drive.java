package frc.robot.subsystems.drive;

import PIDControl.PIDControl;
import frc.robot.abstraction.PositionSensor;
import frc.robot.abstraction.SwartdogSubsystem;

public class Drive extends SwartdogSubsystem
{
    private PositionSensor _gyro;
    private PIDControl     _drivePID;
    private PIDControl     _rotatePID;

    private Vector         _origin;

    private SwerveModule[] _swerveModules;
    private double         _maxModuleDistance;

    private double         _rotateSetpoint;

    private double         _rotateScaler;

    private boolean        _driveInUse;

    public Drive(PositionSensor gyro, PIDControl drivePID, PIDControl rotatePID, SwerveModule... swerveModules) 
    {
        _gyro               = gyro;
        _drivePID           = drivePID;
        _rotatePID          = rotatePID;

        _origin             = new Vector();

        _swerveModules      = swerveModules;
        _maxModuleDistance  = 0;

        _rotateSetpoint     = 0;

        _rotateScaler       = 1;

        _driveInUse         = false;

        resetEncoders();
        setOrigin(0, 0);
    }

    public void drive(double drive, double strafe, double rotate)
    {
        Vector translateVector = new Vector(strafe, drive);

        drive(translateVector, rotate, true);
    }

    public void drive(Vector translateVector, double rotate, boolean absolute)
    {
        if (absolute)
        {
            translateVector.setTheta(translateVector.getTheta() - getHeading());
        }

        drive(translateVector, rotate);
    }

    public void drive(Vector translateVector, double rotate)
    {
        Vector[] moduleCommands = new Vector[_swerveModules.length];

        for (int i = 0; i < _swerveModules.length; i++)
        {
            Vector modulePosition = _swerveModules[i].clone();
            modulePosition.subtract(_origin);

            Vector rotateVector = new Vector(modulePosition.getY(), -modulePosition.getX());
            rotateVector.multiply((rotate * _rotateScaler) / _maxModuleDistance);

            Vector outputVector = translateVector.clone();
            outputVector.add(rotateVector);

            moduleCommands[i] = outputVector;
        }

        double maxSpeed = 0;

        for (int i = 0; i < moduleCommands.length; i++)
        {
            if (moduleCommands[i].getR() > maxSpeed)
            {
                maxSpeed = moduleCommands[i].getR();
            }
        }

        if (maxSpeed > 1.0)
        {
            for (int i = 0; i < moduleCommands.length; i++)
            {
                moduleCommands[i].multiply(1 / maxSpeed);
            }
        }

        for (int i = 0; i < _swerveModules.length; i++)
        {
            _swerveModules[i].drive(moduleCommands[i]);
        }
    }

    public void setOrigin(double x, double y)
    {
        setOrigin(new Vector(x, y));
        
    }

    public void setOrigin(Vector newOrigin)
    {
        _origin = newOrigin;

        _maxModuleDistance = 0;

        for (int i = 0; i < _swerveModules.length; i++)
        {
            Vector modulePosition = _swerveModules[i].clone();
            modulePosition.subtract(_origin);

            if (modulePosition.getR() > _maxModuleDistance)
            {
                _maxModuleDistance = modulePosition.getR();
            }
        }
    }

    public Vector getOrigin() 
    {
        return _origin;
    }

    public SwerveModule getSwerveModule(int index)
    {
        return _swerveModules[index];
    }

    public void driveInit(double distance, double maxSpeed, double minSpeed, boolean resetEncoders)
    {
        maxSpeed = Math.abs(maxSpeed);
        minSpeed = Math.abs(minSpeed);

        if (resetEncoders) 
        {
            resetEncoders();
        }

        _drivePID.setSetpoint(distance, getAverageDistance());
        _drivePID.setOutputRange(-maxSpeed, maxSpeed, minSpeed);

        rotateInit(getHeading(), 0.4);
    }

    public double driveExec()
    {
        return _drivePID.calculate(getAverageDistance());
    }

    public boolean driveIsFinished()
    {
        return _drivePID.atSetpoint();
    }

    public void rotateInit(double heading, double maxSpeed)
    {
        maxSpeed = Math.abs(maxSpeed);

        _rotateSetpoint = normalizeAngle(heading);

        _rotatePID.setSetpoint(_rotateSetpoint, getPIDHeading());
        _rotatePID.setOutputRange(-maxSpeed, maxSpeed);
    }

    public double rotateExec()
    {
        return _rotatePID.calculate(getPIDHeading());
    }

    public boolean rotateIsFinished()
    {
        return _rotatePID.atSetpoint();
    }

    public double getHeading()
    {
        return normalizeAngle(_gyro.get());
    }

    public double getPIDHeading()
    {
        double heading = getHeading();

        if ((_rotateSetpoint - heading) > 180)
        {
            heading += 360;
        }

        else if ((_rotateSetpoint - heading) < -180)
        {
            heading -= 360;
        }

        return heading;
    }

    public void setGyro(double heading) 
    {
        _gyro.set(heading);
    }

    public void resetEncoders()
    {
        for (SwerveModule swerveModule : _swerveModules)
        {
            swerveModule.getDriveMotor().getPositionSensor().set(0);
        }
    }

    public double getAverageDistance()
    {
        double sum = 0;

        for (SwerveModule swerveModule : _swerveModules)
        {
            sum += Math.abs(swerveModule.getDriveMotor().getPositionSensor().get());
        }

        return sum / _swerveModules.length;
    }

    private double normalizeAngle(double angle)
    {
        if (angle < 0)
        {
            angle += 360 * (((int)angle / -360) + 1);
        }

        angle %= 360;

        if (angle > 180)
        {
            angle -= 360;
        }

        return angle;
    }

    public void setRotateScaler(double scaler) 
    {
        _rotateScaler = scaler;
    }

    public void setDriveInUse(boolean driveInUse)
    {
        _driveInUse = driveInUse;
    }

    @Override
    public void periodic()
    {
        if (!_driveInUse) drive(0, 0, 0);
    }

    @Override
    public void setGameMode(GameMode mode) {
        switch (mode)
        {
            case Disabled:
                drive(0, 0, 0);
                break;
            
            default:
                break;
        }
    }
}
