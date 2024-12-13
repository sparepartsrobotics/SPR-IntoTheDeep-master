package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    //public double incTemp= 0.0;
    //public double decTemp= 1.0;
    public double inc = 0.0005;
    public double telePosition = 0.0;

    public double clawArmUp = 0.02;//0.7;
    public double clawTiltUp = 0.3;
    public double clawArmDownHigh = 0.48;
    public double clawTiltDown = 0.87;
    public double clawArmDownLow = 0.6;
    public double clawArmOut = 0.2;

    public boolean specimenIsTilt = false;
    public boolean isArmReleased = false;

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

//        srobot.boxTilt.setPosition(0.4);
//        srobot.boxArm.setPosition(0.25);
//        srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.ascendArm.setMode(STOP_AND_RESET_ENCODER);
        srobot.ascendArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        srobot.rightReleaseArm.setPosition(0.5);
//        srobot.leftReleaseArm.setPosition(0.5);

        waitForStart();

        srobot.box.setPosition(.7);
        srobot.clawArm.setPosition(0.35);
        srobot.clawRotate.setPosition(0.5);
        srobot.claw.setPosition(1.0);
        srobot.clawTilt.setPosition(clawTiltUp);
        srobot.telescopicArm.setPosition(0.0);
        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.75);
        sleep(500);
        srobot.clawArm.setPosition(clawArmUp);

        //While the opmode is running, the method controls is running and the telemetry is updating
        while (opModeIsActive())
        {
            robotMovement();
            //teleArm();
            outtake();
            //claw();
            pushClawDown();
            teleArm();
            rotateClaw();
            openCloseClaw();
            specimenTilt();
            linearSlide();
            specimenGrab();
//            tiltArm();
            //`     ascend();
           // telemetry.addData("linearSlide", srobot.linearSlide.getCurrentPosition());
           // telemetry.addData("ascendArm", srobot.ascendArm.getCurrentPosition());
            telemetry.addData("telescopicArm", srobot.telescopicArm.getPosition());
            telemetry.addData("clawArm", srobot.clawArm.getPosition());
            telemetry.addData("clawTilt", srobot.clawTilt.getPosition());
            telemetry.update();
        }

    }
    /**if(gamepad1.right_trigger > 0.0){
     //srobot.clawRotate.setPosition(.5);
     srobot.clawArm.setPosition(clawArmDownHigh);
     /**srobot.claw.setPosition(1);
     //srobot.claw.setPosition(1.0);
     sleep(300);
     srobot.clawTilt.setPosition(clawTiltDown);
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
     } */
    public void teleArm(){
        if (gamepad1.right_trigger > 0.0) {
            srobot.clawArm.setPosition(clawArmDownHigh);
            srobot.clawTilt.setPosition(clawTiltDown);
            telePosition += inc;
        }
        if (gamepad1.left_trigger > 0.0) {
            telePosition -= inc;
        }

        if (telePosition < 0.0) {
            telePosition = 0.0;
        } else if (telePosition > 0.5) {
            telePosition = 0.5;
        }

        srobot.telescopicArm.setPosition(telePosition);

        if (gamepad1.a) {

            if (srobot.clawArm.getPosition() != clawArmUp) {
                //srobot.clawArm.setPosition(0.4);
                srobot.clawTilt.setPosition(clawTiltUp);
                srobot.box.setPosition(.7);
                sleep(500);
                if (srobot.claw.getPosition() >= 0.5) {
                    srobot.claw.setPosition(1);
                    srobot.clawRotate.setPosition(0.5);
                } else if (srobot.claw.getPosition() < 0.5) {
                    srobot.claw.setPosition(0);
                    srobot.clawRotate.setPosition(.85);
                }
                sleep(250);
                srobot.clawArm.setPosition(clawArmUp);
            }
            telePosition = 0.0;
            srobot.telescopicArm.setPosition(telePosition);
        }
    }
    public void pushClawDown(){
        if(gamepad1.dpad_down){
            srobot.clawArm.setPosition(clawArmDownLow);
            srobot.clawTilt.setPosition(clawTiltDown);
        }
    }
    /** public void claw() {
        //close
        if (gamepad1.right_bumper) {
            srobot.claw.setPosition(.5);
        //open
        } else if (gamepad1.left_bumper) {
            srobot.claw.setPosition(0);
        }
    } */

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
        }
        else if(gamepad2.x){
            srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
            srobot.specimenTilt.setPosition(0.6);
        }
        else if (gamepad2.dpad_up) {
            srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
            srobot.specimenTilt.setPosition(0.75);
        }
    }

    public void specimenGrab() {
        if (gamepad2.dpad_right) {
            srobot.specimenHolder.setPosition(.3);

        } else if (gamepad2.dpad_left) {
            srobot.specimenHolder.setPosition(0.75);
        }
    }
//    public void tiltArm(){
//        if(gamepad1.dpad_down){
//            srobot.intakeArm.setPosition(.05);
//        }
//        if(gamepad1.dpad_left){
//            srobot.intakeArm.setPosition(.08);
//        }
//    }



//    public void ascend() {
//        if (gamepad2.left_bumper && gamepad2.left_trigger > 0) {
//            srobot.leftReleaseArm.setPosition(0.8);
//            srobot.rightReleaseArm.setPosition(0.3);
//            isArmReleased = true;
//        }
//
//        if(gamepad2.right_bumper && isArmReleased) {
//            srobot.telescopicArm.setPosition(0.0);
//
//            srobot.specimenTilt.setPosition(0.75);
//            srobot.specimenHolder.setPosition(0.7);
//
//            srobot.intakeTilt.setPosition(1.0);
//            srobot.intakeArm.setPosition(0.8);
//
//            srobot.ascendArm.setTargetPosition(4800);
//            srobot.ascendArm.setMode(RUN_TO_POSITION);
//            srobot.ascendArm.setPower(0.9);
//        }
//    }

    public void linearSlide(){
        //High Bucket
        if(gamepad1.x) {
            //srobot.telescopicArm.setPosition(0.3);
            srobot.clawArm.setPosition(clawArmOut);
            sleep(300);
            srobot.linearSlide.setTargetPosition(900);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);

            sleep(500);

            srobot.linearSlide.setTargetPosition(2300);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
        //Brings linear slide to high bar for specimen
        } else if (gamepad1.dpad_up) {
            srobot.clawArm.setPosition(clawArmOut);
            sleep(300);

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
            srobot.clawArm.setPosition(clawArmOut);
            srobot.box.setPosition(.7);
            sleep(300);
            srobot.linearSlide.setTargetPosition(0);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(0.85);
            while (srobot.linearSlide.isBusy()) {
                robotMovement();
            }
            srobot.clawArm.setPosition(clawArmUp);
            //srobot.specimenHolder.setPosition(0.75);
            //while(srobot.linearSlide.isBusy()){

            //}
            //srobot.linearSlide.setPower(0.1);
            //srobot.linearSlide.setMode(RUN_USING_ENCODER);
        }
    }
    public void rotateClaw(){
        if(gamepad1.dpad_right){
            srobot.clawRotate.setPosition(0.5);
        }
        else if(gamepad1.dpad_left){
            srobot.clawRotate.setPosition(.85);
        }
    }
    public void openCloseClaw(){
        if(gamepad1.right_bumper){
            srobot.claw.setPosition(1);
        }
        else if(gamepad1.left_bumper){
            srobot.claw.setPosition(0);
        }
    }
    public void outtake() {

        if (gamepad1.b ) {
            srobot.clawArm.setPosition(clawArmOut);
            sleep(300);
            srobot.box.setPosition(0.2);

        }

    }

}
