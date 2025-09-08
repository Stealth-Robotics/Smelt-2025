package org.stealthrobotics.library;

public class AnglePIDController {
    private final double kP;
    private final double kI;
    private final double kD;

    private double reference;
    private double measuredValue;

    private double integralSum;
    private double lastError;

    private double tolerance;
    private double velocityTolerance;

    private double lastMeasuredValue;
    private double lastTime;

    public AnglePIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastTime = (double) System.nanoTime() / 1E9;
    }

    public void setTolerance(double newTolerance) {
        tolerance = newTolerance;
    }

    public void setVelocityTolerance(double velocityTolerance) {
        this.velocityTolerance = velocityTolerance;
    }

    public void setSetPoint(double setPoint) {
        reference = setPoint;
    }

    public double getSetPoint() {
        return reference;
    }

    public double getVelocityError() {
        return wrapAngle(measuredValue - lastMeasuredValue) / (getTimeSinceLastUpdate());
    }

    public double getPositionError() {
        return wrapAngle(reference - measuredValue);
    }

    public boolean atSetPoint() {
        double positionError = Math.abs(wrapAngle(reference - measuredValue));
        double velocityError = Math.abs(wrapAngle(measuredValue - lastMeasuredValue)) / (getTimeSinceLastUpdate());

        return positionError <= tolerance && velocityError <= velocityTolerance;
    }

    public double calculate(double measuredValue) {
        this.measuredValue = measuredValue;

        double time = (double) System.nanoTime() / 1E9;
        double error = wrapAngle(reference - measuredValue);
        double derivative = (error - lastError) / (time - lastTime);

        integralSum += error * (time - lastTime);

        lastError = error;
        lastMeasuredValue = measuredValue;
        lastTime = time;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    // Get time since last update
    private double getTimeSinceLastUpdate() {
        return (double) System.nanoTime() / 1E9 - lastTime;
    }
}
