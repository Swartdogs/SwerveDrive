package frc.robot.subsystems.drive;

public class Vector
{
    private double _x;
    private double _y;
    
    private double _r;
    private double _theta;

    public Vector()
    {
        this(0, 0);
    }

    public Vector(double x, double y)
    {
        this(x, y, true);
    }

    public Vector(double first, double second, boolean isCartesian)
    {
        if (isCartesian)
        {
            _x = first;
            _y = second;
            
            updatePolar();
        }
        else
        {
            _r     = first;
            _theta = second;

            updateCartesian();
        }
    }

    public Vector clone()
    {
        return new Vector(_x, _y);
    }

    public double getX()
    {
        return _x;
    }

    public void setX(double x)
    {
        _x = x;

        updatePolar();
    }

    public double getY()
    {
        return _y;
    }

    public void setY(double y)
    {
        _y = y;

        updatePolar();
    }

    public double getR()
    {
        return _r;
    }

    public void setR(double r)
    {
        _r = r;

        updateCartesian();
    }

    public double getTheta()
    {
        return _theta;
    }

    public void setTheta(double theta)
    {
        _theta = normalizeAngle(theta);

        updateCartesian();
    }

    public void setCartesianPosition(double x, double y)
    {
        _x = x;
        _y = y;

        updatePolar();
    }

    public void translateCartesianPosition(double dx, double dy)
    {
        setCartesianPosition(_x + dx, _y + dy);
    }

    public void setPolarPosition(double r, double theta)
    {
        _r     = r;
        _theta = normalizeAngle(theta);

        updateCartesian();
    }

    public void translatePolarPosition(double dr, double dtheta)
    {
        setPolarPosition(_r + dr, _theta + dtheta);
    }

    public Vector multiply(double scalar)
    {
        Vector v = clone();

        v._r *= scalar;

        v.updateCartesian();

        return v;
    }

    public Vector divide(double scalar)
    {
        Vector v = clone();

        v._r /= scalar;

        v.updateCartesian();

        return v;
    }

    public Vector add(Vector other)
    {
        return new Vector(getX() + other.getX(), getY() + other.getY());
    }

    public Vector subtract(Vector other)
    {
        return new Vector(getX() - other.getX(), getY() - other.getY());
    }

    private void updateCartesian()
    {
        _y = _r * Math.cos(Math.toRadians(_theta));
        _x = _r * Math.sin(Math.toRadians(_theta));
    }

    private void updatePolar()
    {
        _r     = Math.sqrt((_x * _x) + (_y * _y));
        _theta = normalizeAngle(Math.toDegrees(Math.atan2(_x, _y)));
    }

    public static double normalizeAngle(double angle)
    {
        if (angle < 0)
        {
            angle += 360 * (((int)angle / -360) + 1);
        }

        return angle % 360;
    }

    @Override
    public String toString()
    {
        return String.format("(%6.2f, %6.2f)", getX(), getY());
    }
}
