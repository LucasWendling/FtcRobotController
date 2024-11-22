package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class TurnPIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private double error;
    private double KpTerm;
    private double KiTerm;
    private double KdTerm;
    private double deltaTime;
    private double deltaError;
    public TurnPIDController(double target, double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        targetAngle = target;
    }

    public double update(double currentAngle) {
        // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
        //   and ensure direction is correct

        // P
        error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
            deltaTime = (timer.milliseconds() - lastTime);
            deltaError = (error - lastError);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.08 * Math.signum(error)
                + 0.92 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);

        KpTerm = kP * error;
        KiTerm = kI * accumulatedError;
        KdTerm = kD * slope;
        return motorPower;
    }

    public double getLastSlope() {
        return lastSlope;
    }
    public double getError() {
        return error;
    }
    public double getKpTerm(){return KpTerm;}
    public double getKiTerm(){return KiTerm;}
    public double getKdTerm(){return KdTerm;}
    public double getLastTime(){return lastTime;}
    public double getDeltaTime(){return deltaTime;}
    public double getDeltaError(){return deltaError;}

}
