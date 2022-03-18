package frc.robot.subsystems.drive;

import org.ejml.simple.SimpleMatrix;

import frc.robot.Constants;
import frc.robot.abstraction.PositionSensor;

public class SwerveDriveOdometry 
{
    private Position       _currentPosition;
    private SwerveModule[] _modules;
    private PositionSensor _gyro;

    private double _gyroOffset;
    private double _prevAngle;

    private SimpleMatrix _inverseKinematics;
    private SimpleMatrix _forwardKinematics;

    public SwerveDriveOdometry(PositionSensor gyro, SwerveModule... modules)
    {
        _currentPosition = new Position();
        _gyro            = gyro;
        _modules         = modules;

        _inverseKinematics = new SimpleMatrix(_modules.length * 2, 3);

        for (int i = 0; i < _modules.length; i++)
        {
            _inverseKinematics.setRow(i * 2 + 0, 0, 1, 0, -_modules[i].getY());
            _inverseKinematics.setRow(i * 2 + 1, 0, 0, 1,  _modules[i].getX());
        }

        _forwardKinematics = _inverseKinematics.pseudoInverse();
    }

    public Position getPosition()
    {
        return _currentPosition;
    }

    public void resetPosition()
    {
        _currentPosition = new Position();
    }

    public void setPosition(Position position)
    {
        _currentPosition = new Position(position);
    }

    public Vector getTranslation()
    {
        return _currentPosition;
    }

    public double getX()
    {
        return _currentPosition.getX();
    }

    public double getY()
    {
        return _currentPosition.getY();
    }

    public double getAngle()
    {
        return _currentPosition.getAngle();
    }

    public void update()
    {
        double angle = Position.addAngles(_gyro.get(), _gyroOffset);

        ChassisSpeeds chassisSpeeds = getChassisSpeeds();

        Position newPosition = _currentPosition.move(new Position(chassisSpeeds.getXVelocity() * Constants.PERIOD,
                                                                  chassisSpeeds.getYVelocity() * Constants.PERIOD,
                                                                  Position.addAngles(angle, -_prevAngle)));

        _prevAngle = angle;
        _currentPosition = new Position(newPosition, angle);
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        SimpleMatrix stateMatrix = new SimpleMatrix(_modules.length * 2, 1);

        for (int i = 0; i < _modules.length; i++)
        {
            SwerveModule module   = _modules[i];
            double       velocity = module.getDriveVelocity();
            double       sin      = Math.sin(Math.toRadians(module.getPosition()));
            double       cos      = Math.cos(Math.toRadians(module.getPosition()));

            stateMatrix.set(i * 2, 0, velocity * cos);
            stateMatrix.set(i * 2 + 1, velocity * sin);
        }

        SimpleMatrix speeds = _forwardKinematics.mult(stateMatrix);

        return new ChassisSpeeds(speeds.get(1, 0), speeds.get(0, 0));
    }

    private static class ChassisSpeeds
    {
        private double _vx;
        private double _vy;

        public double getXVelocity()
        {
            return _vx;
        }

        public double getYVelocity()
        {
            return _vy;
        }

        public ChassisSpeeds(double vx, double vy)
        {
            _vx = vx;
            _vy = vy;
        }
    }
}
