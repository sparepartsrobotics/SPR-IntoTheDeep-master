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
    double incTemp= 0.0;
    double decTemp=1.0;

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

        srobot.boxTilt.setPosition(0.4);
        srobot.boxArm.setPosition(0.25);

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.telescopicArm.setPosition(0.0);

        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.7);

        srobot.intakeTilt.setPosition(1.0);
        srobot.intakeArm.setPosition(0.8);

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
            srobot.intakeTilt.setPosition(0.2);
            srobot.intakeArm.setDirection(Servo.Direction.FORWARD);
            srobot.intakeArm.setPosition(0.075);
            intakeWheel();
            if(incTemp<gamepad1.right_trigger) {
                srobot.telescopicArm.setPosition(.5 * gamepad1.right_trigger);
                telemetry.addData("teleArm: ", srobot.telescopicArm.getPosition());
                incTemp = gamepad1.right_trigger;
            }
            decTemp=1 - incTemp;
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
            srobot.intakeArm.setPosition(0.8);
            srobot.telescopicArm.setPosition(0.05);
        }


    }
    public void intakeWheel() {
        if (gamepad1.right_bumper) {
            srobot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            srobot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            srobot.intake.setPower(1);
        } else {
            srobot.intake.setPower(0);
        }
    }

    public void robotMovement(){
        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
        blMotorPower  = joystick1LeftY - joystick1LeftX - joystick1RightX;
        frMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
        brMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

        //runs when the right stick buttons is pressed down
        if (gamepad1.right_stick_button)
        {
            //boosts the robot's speed by 35%
            srobot.leftFront.setPower(flMotorPower * -.85);
            srobot.rightFront.setPower(frMotorPower * -.85);
            srobot.leftRear.setPower(blMotorPower * 0.85);
            srobot.rightRear.setPower(brMotorPower * 0.85);
        }
        //runs when the right stick is not pressed
        else
        {
            //speed of robot is normal
            srobot.leftFront.setPower(flMotorPower * -0.5);
            srobot.rightFront.setPower(frMotorPower * -0.5);
            srobot.leftRear.setPower(blMotorPower * 0.5);
            srobot.rightRear.setPower(brMotorPower * 0.5);
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
        if(gamepad1.dpad_down){
            srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
            srobot.specimenTilt.setPosition(0.5);
        } else if (gamepad1.dpad_up) {
            srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
            srobot.specimenTilt.setPosition(0.75);
        }
    }

    public void specimenGrab() {
        if (gamepad1.dpad_left) {
            srobot.specimenHolder.setPosition(.65);

        } else if (gamepad1.dpad_right) {
            srobot.specimenHolder.setPosition(0.7);
        }
    }


    public void tiltBox() {
        if (gamepad1.b) {
            srobot.boxTilt.setPosition(0.9);
        }
    }

    public void linearSlide(){
        if(gamepad1.x) {
            srobot.telescopicArm.setPosition(0.2);
            sleep(350);
            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
            srobot.boxTilt.setPosition(1.0);
            srobot.boxArm.setPosition(0.85);
            srobot.linearSlide.setTargetPosition(1800);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
        } else if (gamepad1.y) {
            srobot.telescopicArm.setPosition(0.2);
            sleep(350);
            srobot.linearSlide.setTargetPosition(0);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
            srobot.boxTilt.setPosition(0.4);
            srobot.boxArm.setPosition(0.25);
            //while(srobot.linearSlide.isBusy()){

            //}
            //srobot.linearSlide.setPower(0.1);
            //srobot.linearSlide.setMode(RUN_USING_ENCODER);
        }
    }
}
