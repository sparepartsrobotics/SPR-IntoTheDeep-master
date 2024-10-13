package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
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
    public boolean isExtended = false;
    public boolean isPivot = false;

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

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.boxTilt.setPosition(0.4);
        srobot.boxArm.setPosition(0.175);

        srobot.telescopicArm.setPosition(0.0);

        waitForStart();

        //While the opmode is running, the method controls is running and the telemetry is updating
        while (opModeIsActive())
        {
            robotMovement();
            moveBox();
            intake();
            specimenTilt();
            linearSlide();
            openClose();
            telemetry.update();
        }

    }
    public void intake(){
        if(gamepad1.left_trigger > 0.0) {
            srobot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            srobot.intake.setPower(1);
            telemetry.addData("leftTrigger", gamepad1.left_trigger);
            srobot.telescopicArm.setDirection(Servo.Direction.FORWARD);
            if (gamepad1.left_trigger < 0.5) {
                srobot.telescopicArm.setPosition(0.1);
            } else if (gamepad1.left_trigger < 1) {
                srobot.telescopicArm.setPosition(0.2);
            } else if (gamepad1.left_trigger == 1){
                srobot.telescopicArm.setPosition(0.32);
            }
        } else {
            srobot.telescopicArm.setDirection(Servo.Direction.FORWARD);
            srobot.telescopicArm.setPosition(0.0);
            srobot.intake.setPower(0);
        }


    }
    public void robotMovement(){
        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
        blMotorPower  = joystick1LeftY + joystick1LeftX + joystick1RightX;
        frMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;
        brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;

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

    public void teleArm(){
        if(gamepad1.x){
            while (System.currentTimeMillis() < 3000) {
                srobot.intake.setPower(1);
            }
        }
    }

    public void moveBox() {
        //move to position
        if (gamepad1.dpad_down) {
            srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
            srobot.boxArm.setDirection(Servo.Direction.FORWARD);
            srobot.boxTilt.setPosition(1.0);
            srobot.boxArm.setPosition(0.85);
        //move back to position
        } else if (gamepad1.dpad_left) {
            srobot.boxTilt.setPosition(0.4);
            srobot.boxArm.setPosition(0.175);
        }
    }

    public void specimenTilt(){
        if(gamepad1.a){
            srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
            srobot.specimenTilt.setPosition(.5);
        } else if (gamepad1.dpad_up) {
            srobot.specimenTilt.setPosition(0.0);
        }
    }

    public void openClose() {
        if (gamepad1.y) {
            srobot.specimenHolder.setPosition(.05);

        } else if (gamepad1.b) {
            srobot.specimenHolder.setPosition(0);
        }
    }

    public void linearSlide(){

        if(gamepad1.dpad_right) {
            srobot.linearSlide.setTargetPosition(900);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);
        } else if (gamepad1.left_bumper) {
            srobot.linearSlide.setTargetPosition(0);
            srobot.linearSlide.setMode(RUN_TO_POSITION);
            srobot.linearSlide.setPower(1);

            //while(srobot.linearSlide.isBusy()){

            //}
            //srobot.linearSlide.setPower(0.1);
            //srobot.linearSlide.setMode(RUN_USING_ENCODER);
        }
    }
}
