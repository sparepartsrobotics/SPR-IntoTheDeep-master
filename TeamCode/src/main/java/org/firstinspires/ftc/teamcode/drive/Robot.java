package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot extends LinearOpMode{
    private double frMotorPower = 1.0;
    private double flMotorPower = 1.0;
    private double brMotorPower = 1.0;
    private double blMotorPower = 1.0;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private DcMotorEx teleArm;
    private DcMotorEx rightRig;
    private DcMotorEx leftRig;
    private DcMotorEx intake;
    //Servos
    private CRServo collect;
    private Servo boxOuttake;
    private Servo pivot;
    private Servo rpIntake;
    private Servo linearSlideRight;
    private Servo LinearSlideLeft;
    private Servo specimenOuttake;
    private double joystick1LeftX,joystick1RightX,joystick1LeftY;
    private boolean isExtended = false;
    public Robot(){

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void intake(){
        if(gamepad1.left_trigger > 0){
            collect.setPower(1);
        }
        else if(gamepad1.right_trigger > 0) {
            collect.setDirection(DcMotorSimple.Direction.REVERSE);
            collect.setPower(1);
        }
        else{
            collect.setPower(0);
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
            leftFront.setPower(flMotorPower * -.85);
            rightFront.setPower(frMotorPower * -.85);
            leftRear.setPower(blMotorPower * 0.85);
            rightRear.setPower(brMotorPower * 0.85);
        }
        //runs when the right stick is not pressed
        else
        {
            //speed of robot is normal
            leftFront.setPower(flMotorPower * -0.5);
            rightFront.setPower(frMotorPower * -0.5);
            leftRear.setPower(blMotorPower * 0.5);
            rightRear.setPower(brMotorPower * 0.5);
        }

    }
    public void teleArm(){
        teleArm.setMode(STOP_AND_RESET_ENCODER);

        if(gamepad1.x && !isExtended){
            teleArm.setTargetPosition(0);
            teleArm.setMode(RUN_TO_POSITION);
            teleArm.setPower(1);
            isExtended = true;
        }
        else if(gamepad1.x && isExtended){
            teleArm.setDirection(DcMotorSimple.Direction.REVERSE);
            teleArm.setTargetPosition(0);
            teleArm.setMode(RUN_TO_POSITION);
            teleArm.setPower(1);
            isExtended = false;
        }

    }
    public void rigging(){
        rightRig.setMode(STOP_AND_RESET_ENCODER);
        leftRig.setMode(STOP_AND_RESET_ENCODER);
        if(gamepad2.y){
            rightRig.setTargetPosition(900);
            rightRig.setMode(RUN_TO_POSITION);
            rightRig.setPower(1);
        }
    }
    public void pivot(){
        if(gamepad1.a){
            pivot.setPosition(.5);
        }

    }
}

