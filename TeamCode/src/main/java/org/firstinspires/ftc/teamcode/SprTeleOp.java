package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "SPR TeleOp")
public class SprTeleOp extends LinearOpMode{
    SampleMecanumDrive srobot;
    public double frMotorPower = 1.0;
    public double flMotorPower = 1.0;
    public double brMotorPower = 1.0;
    public double blMotorPower = 1.0;
    public boolean hasBlock = false;

    public double incTemp= 0.0;
    public double decTemp= 1.0;

    public boolean specimenIsTilt = false;

    /*
    DcMotorEx lf, DcMotorEx lr, DcMotorEx rr, DcMotorEx rf, DcMotorEx ta, DcMotorEx rRig, DcMotorEx lRig
            , CRServo col, Servo boxOuttake, Servo pivot, Servo rpIntake, Servo linearSlideRight, Servo linearSlideLeft,
                 Servo specimenOuttake
     */
//    Robot robot = new Robot(srobot.leftFront, srobot.leftRear, srobot.rightRear, srobot.rightFront,
//            srobot.teleArm, srobot.linearSlide, srobot.intake, srobot.boxOuttake, srobot.pivot,
//            srobot.rpIntake, srobot.linearSlideRight, srobot.linearSlideLeft, srobot.specimenOuttake, srobot.openClose);
//
    double joystick1LeftX,joystick1RightX,joystick1LeftY;
    @Override public void runOpMode(){
        srobot = new SampleMecanumDrive(hardwareMap);

        srobot.boxTilt.setPosition(0.45);
        srobot.boxArm.setPosition(0.3);

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.ascendArm.setMode(STOP_AND_RESET_ENCODER);
        srobot.ascendArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.telescopicArm.setPosition(0.0);

        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.7);

        srobot.rightReleaseArm.setPosition(0.5);
        srobot.leftReleaseArm.setPosition(0.5);

        srobot.intakeTilt.setPosition(1.0);
        srobot.intakeArm.setPosition(0.8);
        incTemp = 0.0;
        decTemp = 1.0;
        waitForStart();

        //While the opmode is running, the method controls is running and the telemetry is updating
        while (opModeIsActive())
        {
            robotMovement();
            //teleArm();
            tiltBox();
            intakeWheel();
            teleArm();
            specimenTilt();
            linearSlide();
            specimenGrab();
            tiltArm();
            ascend();
            telemetry.addData("linearSlide", srobot.linearSlide.getCurrentPosition());
            telemetry.addData("ascendArm", srobot.ascendArm.getCurrentPosition());
            telemetry.update();
        }

    }
    /*
    public void teleArm(){
        if(gamepad1.right_trigger > 0.0) {
            srobot.intakeTilt.setDirection(Servo.Direction.FORWARD);
            srobot.intakeTilt.setPosition(0.2);
            srobot.intakeArm.setDirection(Servo.Direction.FORWARD);
            srobot.intakeArm.setPosition(0.075);
            intakeWheel();
            telemetry.addData("leftTrigger", gamepad1.left_trigger);
            srobot.telescopicArm.setDirection(Servo.Direction.FORWARD);
            if (srobot.telescopicArm.getPosition() == 0.2 || srobot.telescopicArm.getPosition() == 0.1 || srobot.telescopicArm.getPosition() == 0.05) {
                srobot.telescopicArm.setPosition(0.1);
            } else {
                srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition()+0.01);
            }
            //srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition()+0.05);
            if (gamepad1.right_trigger == 1){
                srobot.telescopicArm.setPosition(0.32);
             } else if (gamepad1.right_trigger < 1 && gamepad1.right_trigger >= 0.5) {
                 srobot.telescopicArm.setPosition(0.2);
             } else if (gamepad1.right_trigger < 0.5) {
                 srobot.telescopicArm.setPosition(0.1);
             }
            //hasBlock = true;
        } else if (gamepad1.left_trigger > 0.0) {
            srobot.telescopicArm.setDirection(Servo.Direction.FORWARD);
            if (gamepad1.left_trigger < 0.5) {
                srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition()-0.1);
            } else if (gamepad1.left_trigger < 1) {
                srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition()-0.2);
            } else if (gamepad1.left_trigger == 1) {
                srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition()-0.32);
            }
        } else
    }
    */
    public void teleArm(){
        if(gamepad1.right_trigger > 0.0){
            srobot.intakeTilt.setDirection(Servo.Direction.FORWARD);
            srobot.intakeTilt.setPosition(0.19);
            srobot.intakeArm.setDirection(Servo.Direction.FORWARD);
            srobot.intakeArm.setPosition(0.07);
            srobot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.intake.setPower(1);
            intakeWheel();
            if(incTemp < gamepad1.right_trigger) {
                srobot.telescopicArm.setPosition(.5 * gamepad1.right_trigger);
                telemetry.addData("incTemp: ", incTemp);
                telemetry.addData("teleArm: ", srobot.telescopicArm.getPosition());
                incTemp = gamepad1.right_trigger;
            }
            decTemp = 1 - incTemp;
            telemetry.addData("rightTigger: ", gamepad1.right_trigger);
            telemetry.addData("incTemp: ", incTemp );
            telemetry.addData("decTemp:", decTemp);
        }
        if(gamepad1.left_trigger > 0.0){
            if(decTemp < gamepad1.left_trigger){
                srobot.telescopicArm.setPosition(.5 - .5 * gamepad1.left_trigger);
                decTemp = gamepad1.left_trigger;
            }
            incTemp = 1 - decTemp;
            telemetry.addData("leftTigger: ", gamepad1.left_trigger);
            telemetry.addData("incTemp: ", incTemp );
            telemetry.addData("decTemp: ", decTemp );

            // srobot.telescopicArm.setPosition(srobot.telescopicArm.getPosition() + .01);
            intakeWheel();
        }
        if (gamepad1.a) {
            srobot.intakeTilt.setPosition(1.0);
            srobot.intakeArm.setPosition(0.7);
            srobot.telescopicArm.setPosition(0.05);
        }


    }
    public void intakeWheel() {
        if (gamepad1.right_bumper) {
            srobot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.intake.setPower(0);
        } else if (gamepad1.left_bumper) {
            srobot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            srobot.intake.setPower(1);
        }
    }

    public void robotMovement(){
        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = -gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = -joystick1LeftY - joystick1LeftX - joystick1RightX;
        blMotorPower  = -joystick1LeftY + joystick1LeftX - joystick1RightX;
        frMotorPower = -joystick1LeftY + joystick1LeftX + joystick1RightX;
        brMotorPower = -joystick1LeftY - joystick1LeftX + joystick1RightX;

        //runs when the right stick buttons is pressed down
        if (gamepad1.right_stick_button)
        {
            //boosts the robot's speed by 35%
            srobot.leftFront.setPower(flMotorPower * -1);
            srobot.rightFront.setPower(frMotorPower * -1);
            srobot.leftRear.setPower(blMotorPower * -1);
            srobot.rightRear.setPower(brMotorPower * -1);
        }
        //runs when the right stick is not pressed
        else
        {
            //speed of robot is normal
            srobot.leftFront.setPower(flMotorPower * -0.3);
            srobot.rightFront.setPower(frMotorPower * -0.3);
            srobot.leftRear.setPower(blMotorPower * -0.3);
            srobot.rightRear.setPower(brMotorPower * -0.3);
        }

    }
/*
    public void moveBox() {
        //move to position
        if (gamepad1.a) {
            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
            srobot.boxTilt.setPosition(1.0);
            srobot.boxArm.setPosition(0.85);
        //move back to position
        } else if (gamepad1.b) {
            srobot.boxTilt.setPosition(0.4);
            srobot.boxArm.setPosition(0.25);
        }
    }
*/
    public void specimenTilt(){
        if(gamepad2.dpad_down){
            srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
            srobot.specimenTilt.setPosition(0.45);
        } else if (gamepad2.dpad_up) {
            srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
            srobot.specimenTilt.setPosition(0.75);
        }
    }

    public void specimenGrab() {
        if (gamepad2.dpad_right) {
            srobot.specimenHolder.setPosition(.3);

        } else if (gamepad2.dpad_left) {
            srobot.specimenHolder.setPosition(0.7);
        }
    }
    public void tiltArm(){
        if(gamepad1.dpad_down){
            srobot.intakeArm.setPosition(.05);
        }
        if(gamepad1.dpad_left){
            srobot.intakeArm.setPosition(.08);
        }
    }

    public void tiltBox() {
        if (gamepad1.b) {
            srobot.boxTilt.setPosition(0.83);
        }
    }

    public void ascend() {
        if (gamepad2.left_bumper) {
            srobot.leftReleaseArm.setPosition(0.8);
            srobot.rightReleaseArm.setPosition(0.3);
        }

        if(gamepad2.right_bumper) {
            srobot.telescopicArm.setPosition(0.0);

            srobot.specimenTilt.setPosition(0.75);
            srobot.specimenHolder.setPosition(0.7);

            srobot.intakeTilt.setPosition(1.0);
            srobot.intakeArm.setPosition(0.8);

            srobot.ascendArm.setTargetPosition(4800);
            srobot.ascendArm.setMode(RUN_TO_POSITION);
            srobot.ascendArm.setPower(0.9);
        }
    }

    public void linearSlide(){
        //High Bucket
        if(gamepad1.x) {
            srobot.telescopicArm.setPosition(0.3);
            sleep(300);
            srobot.linearSlide.setTargetPosition(900);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setPosition(0.85);
            sleep(500);
            srobot.boxTilt.setPosition(1);
            srobot.linearSlide.setTargetPosition(1800);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
        //Brings linear slide to high bar for specimen
        } else if (gamepad1.dpad_up) {
            srobot.telescopicArm.setPosition(0.3);
            sleep(300);
            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
            srobot.linearSlide.setTargetPosition(950);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);

        //lowers specimen onto high bar
        }
//        else if (gamepad1.dpad_right) {
//            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
//            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
//            srobot.linearSlide.setTargetPosition(830);
//            srobot.linearSlide.setMode(RUN_TO_POSITION);
//            srobot.linearSlide.setPower(1);
//            //all the way down
//        }
        else if (gamepad1.y) {
            srobot.telescopicArm.setPosition(0.3);
            sleep(300);
            srobot.linearSlide.setTargetPosition(0);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(0.85);
            srobot.boxTilt.setPosition(0.5);
            srobot.boxArm.setPosition(0.25);
            srobot.specimenHolder.setPosition(0.75);
            //while(srobot.linearSlide.isBusy()){

            //}
            //srobot.linearSlide.setPower(0.1);
            //srobot.linearSlide.setMode(RUN_USING_ENCODER);
        }
    }
}
