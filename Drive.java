package frc.robot.drive;

import PIDControl.PIDControl;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstraction.PositionSensor;

public abstract class Drive extends SubsystemBase
{
    protected PositionSensor _gyro;
    protected PIDControl     _translatePID;
    protected PIDControl     _rotatePID;

    private   Vector         _origin;

    protected SwerveModule[] _swerveModules;
    private   double         _maxModuleDistance;

    private   double         _rotateSetpoint;

    private   Vector         _odometer;

    private   Vector         _targetTranslation; 
    private   double         _translationDistanceThreshold;
    private   double         _translationCloseMaxSpeed;

    public Drive() 
    {
        _origin             = new Vector();

        _maxModuleDistance  = 0;

        _rotateSetpoint     = 0;
    }

    public void init()
    {
        resetEncoders();
        setOrigin(0, 0);
        resetOdometer();
    }

    public void drive(double drive, double strafe, double rotate)
    {
        drive(drive, strafe, rotate, true);
    }

    public void drive(double drive, double strafe, double rotate, boolean absolute)
    {
        Vector translateVector = new Vector(strafe, drive);
    
        drive(translateVector, rotate, absolute);
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

    public void translateInit(Vector targetTranslation, double distanceThreshold, double maxSpeed, double closeMaxSpeed, double minSpeed, boolean resetEncoders)
    {
        _targetTranslation = targetTranslation;
        _translationDistanceThreshold = distanceThreshold;
        _translationCloseMaxSpeed = closeMaxSpeed;

        maxSpeed = Math.abs(maxSpeed);
        minSpeed = Math.abs(minSpeed);

        if (resetEncoders) 
        {
            resetEncoders();
        }

        Vector translateErrorVector = _targetTranslation.subtract(getOdometer());

        _translatePID.setSetpoint(0, translateErrorVector.getR());
        _translatePID.setOutputRange(0, maxSpeed, minSpeed);
    }

    public Vector translateExec()
    {
        Vector translateErrorVector = _targetTranslation.subtract(getOdometer());

        if (translateErrorVector.getR() > _translationDistanceThreshold)
        {
            translateErrorVector.setR(_translatePID.calculate(-translateErrorVector.getR()));
        }

        else
        {
            translateErrorVector.setR(Math.min(_translatePID.calculate(-translateErrorVector.getR()), _translationCloseMaxSpeed));
        }

        return translateErrorVector;
    }

    public boolean translateIsFinished()
    {
        return _translatePID.atSetpoint();
    }

    public void rotateInit(double heading, double maxSpeed)
    {
        maxSpeed = Math.abs(maxSpeed);

        _rotateSetpoint = heading;

        double PIDPosition = Math.toRadians(_rotateSetpoint - getHeading());
        PIDPosition /= 2;
        PIDPosition = Math.sin(PIDPosition) * (Math.cos(PIDPosition) / -Math.abs(Math.cos(PIDPosition)));

        _rotatePID.setSetpoint(0, PIDPosition);
        _rotatePID.setOutputRange(-maxSpeed, maxSpeed);
    }

    public double rotateExec()
    {
        double PIDPosition = Math.toRadians(_rotateSetpoint - getHeading());
        PIDPosition /= 2;
        PIDPosition = Math.sin(PIDPosition) * (Math.cos(PIDPosition) / -Math.abs(Math.cos(PIDPosition)));
        return _rotatePID.calculate(PIDPosition);
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
        updateOdometry();

        for (int i = 0; i < _swerveModules.length; i++)
        {
            _swerveModules[i].drive();
        }
    }

    public void resetOdometer()
    {
        resetOdometer(new Vector());
    }

    public void resetOdometer(Vector newPosition)
    {
        _odometer = newPosition;

        for (int i = 0; i < _swerveModules.length; i++)
        {
            _swerveModules[i].resetDrivePosition();
        }
    }

    public Vector getOdometer()
    {
        return _odometer;
    }

    public void updateOdometry()
    {
        Vector change = new Vector();

        for (int i = 0; i < _swerveModules.length; i++)
        {
            change = change.add(_swerveModules[i].getOffset());
        }

        change = change.divide(_swerveModules.length);

        change.translatePolarPosition(0.0, getHeading());

        _odometer = _odometer.add(change);
    }
}
