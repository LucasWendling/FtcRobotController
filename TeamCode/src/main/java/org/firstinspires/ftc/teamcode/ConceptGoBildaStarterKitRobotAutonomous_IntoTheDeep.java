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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="FTC Starter Kit Autonomous Robot (INTO THE DEEP)", group="Robot")
//@Disabled
public class ConceptGoBildaStarterKitRobotAutonomous_IntoTheDeep extends LinearOpMode
{
    MecanumDrive drive = new MecanumDrive();
    public DcMotor  armMotor    = null;
    public Servo    wrist       = null;
    public boolean TestVar = true;
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private IMU imu;
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;
    final double ARM_SCORE_SPECIMEN = 130 * ARM_TICKS_PER_DEGREE;
    final double WRIST_FOLDED_IN   = 0.5;
    private int leftpos;
    private int rightpos;


    @Override
    public void runOpMode()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "right_front_drive"); //hardwareMap.get(DcMotor.class, "left_arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        while (opModeIsActive()) {
            if (TestVar == true) {
                TestVar =false;

                //turnToPID(40.0);
                drive(2000, 2000, 0.5);
                turnPID(-90.0);
            }
            telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
            //telemetry.update();
        }
        /*
        while (opModeIsActive()) {
            if (TestVar==true)
            {
                drive(1235, 1235, 0.25);
                turnRight(180.0, 0.5);
                ResetEncoders();
                drive(1235, 1235, 0.25);
                //turnLeft(90.0, 0.5);
                //turnRight(225.0, 0.5);
                //turnLeft(45.0, 0.5);
                TestVar =false;

            }
            telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        */

        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        drive(1235, 1235, 0.25);


        armPosition = ARM_SCORE_SPECIMEN;
        armMotor.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.75);

        while(armMotor.isBusy())
        {
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.update();
            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
        }

        drive(-1000, -1000, 0.25);

        armMotor.setTargetPosition(0);
        while(armMotor.isBusy())
        {
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.update();
            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
        }

        telemetry.addLine("Done");
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

         while (opModeIsActive() && leftRear.isBusy() && rightRear.isBusy()) ;
         while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) ;
     //..   idle();
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
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.0078, 0, 0.1);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) > 0.5 || pid.getLastSlope() > 0.75)
        {
            double motorPower = pid.update(getHeading(AngleUnit.DEGREES));

            leftRear.setPower(motorPower);
            leftFront.setPower(motorPower);
            rightRear.setPower(-motorPower);
            rightFront.setPower(-motorPower);

            telemetry.addData("target:", targetAngle);
            telemetry.addData("heading:", getHeading(AngleUnit.DEGREES));
            //telemetry.addData("time:", pid.getLastTime());
            //telemetry.addData("error:", pid.getError());
            //telemetry.addData("LastSlope", pid.getLastSlope());
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







    private void turnRight(double targetAngle,double speed)
    {
        double CurrentHeading;
        double error;
        boolean NeedToCrossZero = false;

        CurrentHeading = getHeading(AngleUnit.DEGREES);

        if (CurrentHeading >= targetAngle)
        {
            NeedToCrossZero = true;
        }

        while ((CurrentHeading < targetAngle) || (NeedToCrossZero == true))
        {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            error = targetAngle - CurrentHeading;

/*            if ((NeedToCrossZero == false) && (error < 30))
            {
                leftRear.setPower(speed*(error/80));
                leftFront.setPower(speed*(error/80));
                rightRear.setPower(-speed*(error/80));
                rightFront.setPower(-speed*(error/80));
            }
            else*/
            {
                leftRear.setPower(speed);
                leftFront.setPower(speed);
                rightRear.setPower(-speed);
                rightFront.setPower(-speed);
            }

            telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
            telemetry.addData("speed", speed);
            telemetry.update();

            CurrentHeading = getHeading(AngleUnit.DEGREES);

            if ((NeedToCrossZero == true) && (CurrentHeading < targetAngle))
            {
                NeedToCrossZero = false;
            }

        }

        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
        telemetry.addData("speed", 0);
        telemetry.update();
    }

    private void turnLeft(double targetAngle,double speed)
    {
        double CurrentHeading;
        boolean NeedToCrossZero = false;
        double error;

        CurrentHeading = getHeading(AngleUnit.DEGREES);

        if (CurrentHeading <= targetAngle)
        {
            NeedToCrossZero = true;
        }

        while ((CurrentHeading > targetAngle) || (NeedToCrossZero == true))
        {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            error = CurrentHeading - targetAngle;

/*            if ((NeedToCrossZero == false) && (error < 30))
            {
                leftRear.setPower(-speed*(error/80));
                leftFront.setPower(-speed*(error/80));
                rightRear.setPower(speed*(error/80));
                rightFront.setPower(speed*(error/80));
            }
            else*/
            {
                leftRear.setPower(-speed);
                leftFront.setPower(-speed);
                rightRear.setPower(speed);
                rightFront.setPower(speed);
            }

            telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
            telemetry.addData("speed", speed);
            telemetry.update();

            CurrentHeading = getHeading(AngleUnit.DEGREES);

            if ((NeedToCrossZero == true) && (CurrentHeading > targetAngle))
            {
                NeedToCrossZero = false;
            }

        }

        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
        telemetry.addData("speed", 0);
        telemetry.update();
    }

    private void ResetEncoders()
    {
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftpos = 0;
        rightpos = 0;
    }

}

