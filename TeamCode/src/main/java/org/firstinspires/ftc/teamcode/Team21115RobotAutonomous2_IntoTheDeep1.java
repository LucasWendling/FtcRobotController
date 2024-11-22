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

@Autonomous(name="FTC 21115 Into The Deep Autonomous 1", group="Robot")
//@Disabled
public class Team21115RobotAutonomous2_IntoTheDeep1 extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    public DcMotor linearSlide;
    public DcMotor armMotor = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public boolean TestVar = true;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private IMU imu;
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_OUT = 0.19;
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647 * 117.0 / 60.0;  //Adjustment for new motor
    final double ARM_SCORE_SPECIMEN = 150 * ARM_TICKS_PER_DEGREE;
    final double WRIST_FOLDED_IN = 0.55;
    final int LINEARSLIDE_IN = 5;
    final int LINEARSLIDE_OUT = 2450;
    final int LINEARSLIDE_COLLECT_OUT = 1000;
    private int leftpos;
    private int rightpos;
    ArmPIDController ArmPID = new ArmPIDController(0.015,0.0,0.0, 0.15);


    @Override
    public void runOpMode()
    {
        double armPower=0;
        imu = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(CRServo.class, "intake");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");
        linearSlide = hardwareMap.dcMotor.get("linear_slide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftpos = 0;
        rightpos = 0;

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));

        wrist.setPosition(WRIST_FOLDED_IN);

        waitForStart();

        drive(550, 550, 0.50);

        turnToPID(270);

        ResetEncoders();
        drive(2500, 2500, 0.50);

        turnToPID(235);

        wrist.setPosition(WRIST_FOLDED_OUT);

        armPosition = ARM_SCORE_SPECIMEN;

        armPower = ArmPID.update((armPosition), armMotor.getCurrentPosition());

        armMotor.setPower(armPower);

        while (Math.abs(ArmPID.getError()) > 2 * ARM_TICKS_PER_DEGREE)
        {
            armPower = ArmPID.update((armPosition), armMotor.getCurrentPosition());
            armMotor.setPower(armPower);

            telemetry.addData("ArmError", armPower);
            telemetry.update();
        }
        armMotor.setPower(0.0);


        linearSlide.setTargetPosition(LINEARSLIDE_OUT);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1.0);

        while (linearSlide.isBusy())
        {}
        linearSlide.setPower(0.1);

        ResetEncoders();
        drive(200,200,0.50);

        intake.setPower(INTAKE_DEPOSIT);
        timeDelay(1000);
        intake.setPower(INTAKE_OFF);

        ResetEncoders();
        drive(-200,-200,0.50);

        linearSlide.setTargetPosition(LINEARSLIDE_IN);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.5);

        while (linearSlide.isBusy())
        {}
        linearSlide.setPower(0.0);

        armPosition = ARM_COLLAPSED_INTO_ROBOT;

        armPower = ArmPID.update((armPosition), armMotor.getCurrentPosition());
        armMotor.setPower(armPower);

        telemetry.addData("ArmError", ArmPID.getError());
        telemetry.update();

        while (Math.abs(ArmPID.getError()) > 10)
        {
            armPower = ArmPID.update(armPosition, armMotor.getCurrentPosition());
            armMotor.setPower(armPower);

            telemetry.addData("ArmError", ArmPID.getError());
            telemetry.addData("ArmPower", armPower);
            telemetry.update();
        }
        armMotor.setPower(0.0);


        turnToPID(90);

        ResetEncoders();
        drive(4300, 4300,0.60);

        turnToPID(180);

        ResetEncoders();
        drive(500,500,0.50);




        //

        //turnToPID(180);

        //ResetEncoders();
        //drive(300, 300, 0.50);
















        telemetry.addLine("Done");
        telemetry.update();
    }

    private void timeDelay(double waitTime) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < waitTime)
        {
            /* Do Nothing */
        }
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
     //..   idle();
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

        motorPower = pid.update(getHeading(AngleUnit.DEGREES));

        // Checking lastSlope to make sure that it's not oscillating when it quits
        //while ((Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) > 1) || (Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) <-359) || pid.getLastSlope() > 0.75)
        while ((Math.abs(pid.getError()) > 2) || pid.getLastSlope() > 0.75)
        {
            motorPower = pid.update(getHeading(AngleUnit.DEGREES));


            leftRear.setPower(motorPower);
            leftFront.setPower(motorPower);
            rightRear.setPower(-motorPower);
            rightFront.setPower(-motorPower);

            telemetry.addData("target:", targetAngle);
            telemetry.addData("heading:", getHeading(AngleUnit.DEGREES));
            //telemetry.addData("time:", pid.getLastTime());
            //telemetry.addData("error:", pid.getError());
            telemetry.addData("LastSlope", pid.getLastSlope());
            telemetry.addData("DeltaTime:", pid.getDeltaTime());
            telemetry.addData("DeltaError:", pid.getDeltaError());
            //telemetry.addData("KpTerm:", pid.getKpTerm());
            //telemetry.addData("KiTerm:", pid.getKiTerm());
            //telemetry.addData("KdTerm:", pid.getKdTerm());
            telemetry.update();


        }
        leftRear.setPower(0.0);
        leftFront.setPower(0.0);
        rightRear.setPower(0.0);
        rightFront.setPower(0.0);
    }



}

