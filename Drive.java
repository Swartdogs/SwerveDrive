package frc.robot.subsystems.drive;

import PIDControl.PIDControl;
import frc.robot.abstraction.PositionSensor;
import frc.robot.abstraction.SwartdogSubsystem;

public abstract class Drive extends SwartdogSubsystem
{
    protected PositionSensor      _gyro;
    protected PIDControl          _translatePID;
    protected PIDControl          _rotatePID;

    private   Vector              _origin = new Vector();

    protected SwerveModule[]      _swerveModules;
    private   double              _maxModuleDistance = 0;

    private   SwerveDriveOdometry _odometer;

    private   Position            _targetPosition;

    public void init()
    {
        resetEncoders();
        setOrigin(0, 0);

        _odometer = new SwerveDriveOdometry(_gyro, _swerveModules);
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

        double maxSpeed = 1;

        for (int i = 0; i < _swerveModules.length; i++)
        {
            Vector modulePosition = _swerveModules[i].subtract(_origin);
            Vector rotateVector = new Vector(modulePosition.getY(), -modulePosition.getX()).multiply(rotate / _maxModuleDistance);
            Vector outputVector = translateVector.add(rotateVector);

            moduleCommands[i] = outputVector;

            maxSpeed = Math.max(maxSpeed, moduleCommands[i].getR());
        }

        for (int i = 0; i < moduleCommands.length; i++)
        {
            moduleCommands[i] = moduleCommands[i].divide(maxSpeed);

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
            modulePosition = modulePosition.subtract(_origin);

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

    public void autoDriveInit(Position targetPosition, double rotateSpeed, double maxSpeed, double minSpeed)
    {
        _targetPosition = targetPosition;

        rotateSpeed = Math.abs(rotateSpeed);
        maxSpeed    = Math.abs(maxSpeed);
        minSpeed    = Math.abs(minSpeed);

        Vector translateErrorVector = getOdometer().subtract(_targetPosition);

        _translatePID.setSetpoint(0, translateErrorVector.getR());
        _translatePID.setOutputRange(0, maxSpeed, minSpeed);

        double pidPosition = Math.toRadians(_targetPosition.getAngle() - getHeading()) / 2;
        pidPosition = Math.sin(pidPosition) * (Math.cos(pidPosition) / -Math.abs(Math.cos(pidPosition)));

        _rotatePID.setSetpoint(0, pidPosition);
        _rotatePID.setOutputRange(-maxSpeed, maxSpeed);
    }

    public Vector translateExec()
    {
        Vector translateError = getOdometer().subtract(_targetPosition);
        translateError.setR(_translatePID.calculate(translateError.getR()));
        
        return translateError;
    }

    public double rotateExec()
    {
        double pidPosition = Math.toRadians(_targetPosition.getAngle() - getHeading()) / 2;
        pidPosition = Math.sin(pidPosition) * (Math.cos(pidPosition) / -Math.abs(Math.cos(pidPosition)));

        return _rotatePID.calculate(pidPosition);
    }

    public boolean translateIsFinished()
    {
        return _translatePID.atSetpoint();
    }

    public boolean rotateIsFinished()
    {
        return _rotatePID.atSetpoint();
    }

    public boolean autoDriveIsFinished()
    {
        return translateIsFinished() && rotateIsFinished();
    }

    public double getHeading()
    {
        return normalizeAngle(_gyro.get());
    }

    public void setGyro(double heading) 
    {
        _gyro.set(heading);
    }

    public void resetEncoders()
    {
        for (SwerveModule swerveModule : _swerveModules)
        {
            swerveModule.resetDrivePosition();            
        }
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

    @Override
    public void periodic()
    {
        for (int i = 0; i < _swerveModules.length; i++)
        {
            _swerveModules[i].periodic();
        }

        _odometer.update();
    }

    public void resetOdometer()
    {
        resetOdometer(new Position());
    }

    public void resetOdometer(Position newPosition)
    {
        _odometer.setPosition(newPosition);

        for (int i = 0; i < _swerveModules.length; i++)
        {
            _swerveModules[i].resetDrivePosition();
        }
    }

    public Position getOdometer()
    {
        return _odometer.getPosition();
    }
}
