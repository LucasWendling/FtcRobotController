/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="FTC 21115 Into The Deep Autonomous", group="Robot")
//@Disabled
public class Team21115RobotAutonomous_IntoTheDeep extends LinearOpMode
{
    MecanumDrive drive = new MecanumDrive();
    public DcMotor  armMotor    = null;
    public Servo    wrist       = null;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private IMU imu;

    /* Wrist constants */
    final double WRIST_FOLDED_OUT  = 0.19;
    final double WRIST_FOLDED_IN   = 0.55;

    /* Arm constants */
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647 * 117.0/60.0;  //Adjustment for new motor
    final double ARM_SCORE_SPECIMEN = 130 * ARM_TICKS_PER_DEGREE;

    /* Drive Motor position variables */
    private int leftpos;
    private int rightpos;

    /* Arm variables */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;


    @Override
    public void runOpMode()
    {
        /* Hardware Mapping */
        imu = hardwareMap.get(IMU.class, "imu");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");

        /* Arm Motor configuration */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Initialize Drive Motors */
        ResetEncoders();
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Initialize IMU */
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));

        /* Command wrist to starting position */
        wrist.setPosition(WRIST_FOLDED_IN);

        waitForStart();

        /* Extend wrist */
        wrist.setPosition(WRIST_FOLDED_OUT);

        /* Drive toward submersible */
        drive(1185, 1185, 0.30);

        /* Move arm into specimen hang position */
        armPosition = ARM_SCORE_SPECIMEN;
        armMotor.setTargetPosition((int) (armPosition));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.75);

        while(armMotor.isBusy())
        {
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
                telemetry.update();
            }
        }

        /* Reverse away from submersible */
        drive(-200, -200, 0.50);

        /* Move arm back to starting position */
        armMotor.setTargetPosition(0);
        while(armMotor.isBusy())
        {
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
                telemetry.update();
            }
        }

        /* Turn towards basket */
        turnToPID(270);

        /* Drive towards basket */
        ResetEncoders();
        drive(1500, 1500, 0.50);

        /* Turn away from starting wall */
        turnToPID(0);

        /* Drive enough past sample we want to push */
        ResetEncoders();
        drive(1000, 1000, 0.50);

        /* Turn towards basket and sample to push */
        turnToPID(210);

        /* Drive to push sample into score zone */
        ResetEncoders();
        drive(1750, 1750, 0.50);

        /* Reverse out of score zone */
        ResetEncoders();
        drive(-200, -200, 0.50);

        /* Turn towards observation zone */
        turnToPID(90);

        /* Drive towards observation zone */
        ResetEncoders();
        drive(5000, 5000, 0.60);

        /* Turn to face observation zone */
        turnToPID(180);

        /* Drive into observation zone */
        ResetEncoders();
        drive(500, 500, 0.50);

        telemetry.addLine("Autonomous Done");
        telemetry.update();
    }

    private void drive( int leftTarget, int rightTarget, double speed)
    {
        leftpos += leftTarget;
        rightpos += rightTarget;
        leftRear.setTargetPosition(leftpos);
        leftFront.setTargetPosition(leftpos);
        rightRear.setTargetPosition(rightpos);
        rightFront.setTargetPosition(rightpos);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setPower(speed);
        leftFront.setPower(speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);
        telemetry.addData("status:", "Driving");
        telemetry.addData("wheelpos", leftRear.getCurrentPosition());
        telemetry.update();
         while (opModeIsActive() && leftRear.isBusy() && rightRear.isBusy()) ;
         while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) ;
    }

    public void ResetEncoders()
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftpos = 0;
        rightpos = 0;
    }

    public double getHeading(AngleUnit angleUnit)
    {
        double angle = -imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

        if (angle < 0.0)
        {
            angle = angle+360.0;
        }

        return angle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getHeading(AngleUnit.DEGREES));
    }

    void turnToPID(double targetAngle) {
        double motorPower;

        TurnPIDController pid = new TurnPIDController(targetAngle, 0.015, 0, 0.04);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Call this to update initial error term */
        motorPower = pid.update(getHeading(AngleUnit.DEGREES));

        leftRear.setPower(motorPower);
        leftFront.setPower(motorPower);
        rightRear.setPower(-motorPower);
        rightFront.setPower(-motorPower);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        //while ((Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) > 1) || (Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) <-359) || pid.getLastSlope() > 0.75)
        while ((Math.abs(pid.getError()) > 2) || pid.getLastSlope() > 0.75)
        {
            motorPower = pid.update(getHeading(AngleUnit.DEGREES));

            leftRear.setPower(motorPower);
            leftFront.setPower(motorPower);
            rightRear.setPower(-motorPower);
            rightFront.setPower(-motorPower);

            telemetry.addData("Turn Target:", targetAngle);
            telemetry.addData("Turn Heading:", getHeading(AngleUnit.DEGREES));
            telemetry.addData("Turn Error:", pid.getError());
            telemetry.update();
        }
        leftRear.setPower(0.0);
        leftFront.setPower(0.0);
        rightRear.setPower(0.0);
        rightFront.setPower(0.0);
    }

    public void timeDelay(double waitTime) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < waitTime)
        {
            /* Do Nothing */
        }
    }


}

