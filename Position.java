package frc.robot.subsystems.drive;

public class Position extends Vector
{
    private double _angle;

    public double getAngle()
    {
        return _angle;
    }

    public void setAngle(double angle)
    {
        _angle = angle;
    }

    public Position move(Position delta)
    {
        double x     = delta.getX();
        double y     = delta.getY();
        double theta = Math.toRadians(delta.getAngle());
        double sin   = Math.sin(theta);
        double cos   = Math.cos(theta);

        double s;
        double c;

        if (Math.abs(theta) < 1e-9)
        {
            s = 1.0 - 1.0 / 6.0 * theta * theta;
            c = 0.5 * theta;
        }
        else
        {
            s = sin / theta;
            c = (1 - cos) / theta;
        }

        Position transform = new Position(new Vector(x * s - y * c, x * c + y * s), composeAngle(cos, sin));

        return this.transformBy(transform);
    }

    public Position transformBy(Position other)
    {
        return new Position(this.add(other.rotate(_angle)), addAngles(_angle, other._angle));
    }

    public static double addAngles(double angle1, double angle2)
    {
        double cos1 = Math.cos(Math.toRadians(angle1));
        double sin1 = Math.sin(Math.toRadians(angle1));

        double cos2 = Math.cos(Math.toRadians(angle2));
        double sin2 = Math.sin(Math.toRadians(angle2));

        return composeAngle(cos1 * cos2 - sin1 * sin2, cos1 * sin2 + sin1 * cos2);
    }

    public static double composeAngle(double x, double y)
    {
        double magnitude = Math.hypot(x, y);
        double angle;

        if (magnitude > 1e-6)
        {
            angle = Math.atan2(y / magnitude, x / magnitude);
        }
        else
        {
            angle = Math.atan2(0.0, 1.0);
        }

        return Math.toDegrees(angle);
    }

    public Position()
    {
        this(0);
    }

    public Position(double angle)
    {
        this(0, 0, angle);
    }

    public Position(double x, double y, double angle)
    {
        this(x, y, angle, true);
    }

    public Position(double first, double second, double angle, boolean isCartesian)
    {
        super(first, second, isCartesian);

        _angle = angle;
    }

    public Position(Vector position)
    {
        this(position, 0);
    }

    public Position(Vector position, double angle)
    {
        super(position.getX(), position.getY());

        _angle = angle;
    }

    public Position(Position other)
    {
        super(other.getX(), other.getY());

        _angle = other._angle;
    }

    @Override
    public String toString()
    {
        return String.format("(%6.2f, %6.2f, %6.2f)", getX(), getY(), getAngle());
    }
}
