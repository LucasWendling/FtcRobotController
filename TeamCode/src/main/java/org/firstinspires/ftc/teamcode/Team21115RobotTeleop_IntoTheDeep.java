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

    /* Arm constants */
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647 * 117.0/60.0; //exact fraction is (194481/9826)
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 218 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 112 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 1  * ARM_TICKS_PER_DEGREE;
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE;

    /* Linear Slide constants */
    final int LINEARSLIDE_IN            = 5;
    final int LINEARSLIDE_OUT           = 2450;
    final int LINEARSLIDE_COLLECT_OUT   = 1000;

    /* Intake constants */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Wrist constants */
    final double WRIST_FOLDED_IN   = 0.5;
    final double WRIST_FOLDED_OUT  = 0.19;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    ArmPIDController ArmPID = new ArmPIDController(0.015,0.0,0.0, 0.15);

    @Override
    public void runOpMode() {
        double armPower=0;
        boolean CanLatchNewFF = true;
        double PermFudgeFactor = 0.0;

        /* Drive motor initialization */
        drive.init(hardwareMap);

        /* Map Hardware */
        armMotor   = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
        linearSlide = hardwareMap.dcMotor.get("linear_slide");

        /* Initialize Linear Slide */
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Initialize Arm */
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);

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

            /* Process joystick driving control */
            drive.drive(forward, right, rotate);

            /* Button X -> Move Arm to basket score position */
            if (gamepad1.x)
            {
               if (linearSlide.getPower() == 0.0)
               {
                    armPosition = ARM_SCORE_SPECIMEN;
               }
            }
            /* Button Y -> Move Arm to sample collection position */
            else if (gamepad1.y)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_COLLECT;
                }
            }
            /* Button B -> Linear Slide control */
            else if (gamepad1.b)
            {
                /* Control Slide in Scoring position */
                if ((wrist.getPosition()== WRIST_FOLDED_OUT) && (armPosition== ARM_SCORE_SPECIMEN))
                {
                    /* if slide is out, move it in */
                    if ((linearSlide.getTargetPosition() == LINEARSLIDE_OUT ) && !linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition( LINEARSLIDE_IN);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                    /* if slide is in, move it out */
                    else if (!linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition(LINEARSLIDE_OUT);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                }

                /* Control Slide in Collection position */
                if ((wrist.getPosition()== WRIST_FOLDED_OUT) && (armPosition== ARM_COLLECT))
                {
                    /* if slide is out, move it in */
                    if ((linearSlide.getTargetPosition() == LINEARSLIDE_COLLECT_OUT ) && !linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition( LINEARSLIDE_IN);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                    /* if slide is in, move it out */
                    else if (!linearSlide.isBusy())
                    {
                        linearSlide.setTargetPosition(LINEARSLIDE_COLLECT_OUT);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(1.0);
                    }
                }
            }
            /* Start button will reset arm position to start position */
            else if (gamepad1.start)
            {
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_COLLAPSED_INTO_ROBOT;
                }
            }

            /* If linear slide is done moving, reduce power to 50% if slide is out, otherwise turn power off */
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

            /* Pressing A will latch in a permanent fudge factor based on whatever is current active and
            what has been latched already in the past, you will not be able to latch a new one until you
            release the triggers */
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

            /* Holding right bumper will cause intake to deposit */
            if(gamepad1.right_bumper)
            {
                intake.setPower(INTAKE_DEPOSIT);
            }
            /* Holding left bumper will cause intake to collect */
            else if(gamepad1.left_bumper)
            {
                intake.setPower(INTAKE_COLLECT);
            }
            /* If neither bumper is being pressed, turn intake off */
            else
            {
                intake.setPower(INTAKE_OFF);
            }

            if (gamepad1.dpad_left)
            {
                /* Nothing Currently */
            }
            else if (gamepad1.dpad_right)
            {
                /* Nothing Currently */
            }
            else if (gamepad1.dpad_up)
            {
                /* Move Arm position to prepare to hang on submersible for L2 ascent */
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_ATTACH_HANGING_HOOK;
                }
            }
            else if (gamepad1.dpad_down)
            {
                /* Move Arm to cause L2 ascent */
                if (linearSlide.getPower() == 0.0)
                {
                    armPosition = ARM_WINCH_ROBOT;
                }
            }

            /* Calculate new Arm Power based on PID control */
            armPower = ArmPID.update((armPosition+armPositionFudgeFactor + PermFudgeFactor), armMotor.getCurrentPosition());

            /* If we are winching robot for L2 ascent, force full power to motor */
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
            telemetry.addData("ArmPower:", armMotor.getPower());
            telemetry.addData("ArmPos:", armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE);
            telemetry.addData("ArmPosTarget:", armPosition/ARM_TICKS_PER_DEGREE);

            telemetry.update();
        }
    }
}
