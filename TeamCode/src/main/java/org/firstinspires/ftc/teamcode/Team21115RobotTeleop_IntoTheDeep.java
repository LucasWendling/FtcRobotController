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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * '
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */

/* 21115 */
@TeleOp(name="FTC 21115 Into The Deep Teleop", group="Robot")
//@Disabled
public class Team21115RobotTeleop_IntoTheDeep extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    MecanumDrive drive = new MecanumDrive();
    public DcMotor linearSlide;
    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 218 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 112 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 1  * ARM_TICKS_PER_DEGREE;
    final int LINEARSLIDE_IN            = 5;
    final int LINEARSLIDE_OUT           = 2450;
    final int LINEARSLIDE_COLLECT_OUT   = 1000;
    boolean buttonIsReleased = true;
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.5;
    final double WRIST_FOLDED_OUT  = 0.19;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    //ArmPIDController ArmPID = new ArmPIDController(0.1,0.0,0.3, 0.1);
    //ArmPIDController ArmPID = new ArmPIDController(0.08,0.0,1, 0.14);
    //ArmPIDController ArmPID = new ArmPIDController(0.03,0.0,0.0, 0.15);
    ArmPIDController ArmPID = new ArmPIDController(0.015,0.0,0.0, 0.15);


    @Override
    public void runOpMode() {
        double armPower=0;
        boolean CanLatchNewFF = true;
        double PermFudgeFactor = 0.0;

        drive.init(hardwareMap);
        /* Define and Initialize Motors */
        armMotor   = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
        linearSlide = hardwareMap.dcMotor.get("linear_slide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
//        armMotor.setTargetPosition(0);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();  /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            boolean update = true;

            drive.drive(forward, right, rotate);

            if (gamepad1.x)
            {
               if (linearSlide.getPower() == 0.0)
               {
                    armPosition = ARM_SCORE_SPECIMEN;
               }
            }
            else if (gamepad1.y)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_COLLECT;
                }
            }
            else if (gamepad1.a)
            {

            }
            else if (gamepad1.b)
            {
                if ((wrist.getPosition()== WRIST_FOLDED_OUT) && (armPosition== ARM_SCORE_SPECIMEN))
                {
                    if ((linearSlide.getTargetPosition() == LINEARSLIDE_OUT ) && !linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition( LINEARSLIDE_IN);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                    else if (!linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition(LINEARSLIDE_OUT);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                }

                if ((wrist.getPosition()== WRIST_FOLDED_OUT) && (armPosition== ARM_COLLECT))
                {
                    if ((linearSlide.getTargetPosition() == LINEARSLIDE_COLLECT_OUT ) && !linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition( LINEARSLIDE_IN);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                    else if (!linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition(LINEARSLIDE_COLLECT_OUT);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                }
            }

            else if (gamepad1.start)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_COLLAPSED_INTO_ROBOT;
                }
            }

            if (!linearSlide.isBusy())
            {
                if (linearSlide.getTargetPosition() == LINEARSLIDE_IN)
                {
                    linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linearSlide.setPower(0.0);
                    linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    /* Try below code */
                    linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else if ((linearSlide.getTargetPosition() == LINEARSLIDE_OUT) || (linearSlide.getTargetPosition() == LINEARSLIDE_COLLECT_OUT))
                {
                    linearSlide.setPower(0.5);
                }
            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
            if ((gamepad1.a) && (CanLatchNewFF ==true))
            {
                PermFudgeFactor += armPositionFudgeFactor;
                armPositionFudgeFactor = 0;
                CanLatchNewFF = false;
            }
            else if ((armPositionFudgeFactor != 0) && (CanLatchNewFF ==false))
            {
                armPositionFudgeFactor = 0;
            }
            else if ((armPositionFudgeFactor == 0) && (CanLatchNewFF ==false))
            {
                CanLatchNewFF = true;
            }


            if(gamepad1.right_bumper)
            {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else if(gamepad1.left_bumper)
            {
                intake.setPower(INTAKE_COLLECT);
            }
            else
            {
                intake.setPower(INTAKE_OFF);
            }

            if (gamepad1.dpad_left)
            {
            }
            else if (gamepad1.dpad_right)
            {
            }
            else if (gamepad1.dpad_up)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_ATTACH_HANGING_HOOK;
                }
            }
            else if (gamepad1.dpad_down)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_WINCH_ROBOT;
                }
            }

//            armMotor.setTargetPosition((int) (armPosition +armPositionFudgeFactor));

//            if ((armPosition == ARM_COLLAPSED_INTO_ROBOT) && !armMotor.isBusy()) {
//                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            else {
//                ((DcMotorEx) armMotor).setVelocity(2100);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

            if (gamepad1.a)
            {
                //armPower = armPower-.001;
                //ArmPID.updateKpKd(0.015, 3);
                //armPosition = 212 * 19.7924893140647;
            }
            else {
                //ArmPID.updateKpKd(0.015,0);
                //armPosition = 212 * 19.7924893140647;
            }

            armPower = ArmPID.update((armPosition+armPositionFudgeFactor + PermFudgeFactor), armMotor.getCurrentPosition());

            if (armPosition == ARM_WINCH_ROBOT)
            {

                {
                    armPower = -1;
                }
            }
            armMotor.setPower(armPower);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* send telemetry to the driver of the arm's current position and target position */
            //telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            //telemetry.addData("IsBusy: ", linearSlide.isBusy());
            //telemetry.addData("slidePower: ", linearSlide.getPower());
            //telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            //telemetry.addData("intakePower", intake.getPower());
            telemetry.addData("ArmPower:", armMotor.getPower());
            //telemetry.addData("KpTerm:", ArmPID.getKpTerm());
            //telemetry.addData("KdTerm:", ArmPID.getKdTerm());
            //telemetry.addData("FfTerm:", ArmPID.getFfTerm());
            telemetry.addData("ArmPos:", armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE);
            telemetry.addData("ArmPosTarget:", armPosition/ARM_TICKS_PER_DEGREE);
            //telemetry.addData("ArmPosFF:", armPositionFudgeFactor);
            //telemetry.addData("PidError:", ArmPID.getError());
            //telemetry.addData("KdTerm:", ArmPID.getKdTerm());
            //telemetry.addData("KdTerm:", ArmPID.getKdTerm());

            telemetry.update();


        }

    }
}
