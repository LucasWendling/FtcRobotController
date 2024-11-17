package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class ArmPIDController {
    private double kP, kI, kD, fF;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private double error;
    private double FfTerm;
    private double KpTerm;
    private double KiTerm;
    private double KdTerm;
    private double deltaTime;
    private double deltaError;
    private double cosTerm;
    public ArmPIDController(double p, double i, double d, double ff) {
        kP = p;
        kI = i;
        kD = d;
        fF = Math.min(ff,1.0);
    }

    public void updateKpKd(double p, double d)
    {
        kP = p;
        kD = d;
    }
    public void updateFf(double ff)
    {
        fF = Math.min(ff,1.0);
    }

    public double update(double targetPosition, double currentPosition) {
        // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
        //   and ensure direction is correct

        // P
        error = targetPosition / 19.7924893140647 - currentPosition / 19.7924893140647;

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

        cosTerm = Math.cos(Math.toRadians((currentPosition / 19.7924893140647)-14));
        FfTerm = fF * Math.cos(Math.toRadians((currentPosition / 19.7924893140647)-14));
        KpTerm = kP * error;
        KiTerm = kI * accumulatedError;
        KdTerm = kD * slope;

        double motorPower = FfTerm + (KpTerm + KiTerm + KdTerm);
              /*+ (1.0-fF) * Math.tanh(kP * error + kI * accumulatedError - kD * slope);*/
        motorPower = Math.min(1.0,motorPower);
        motorPower = Math.max(-1.0,motorPower);

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
    public double getFfTerm(){return FfTerm;}
    public double getCosTerm(){return cosTerm;}

}
