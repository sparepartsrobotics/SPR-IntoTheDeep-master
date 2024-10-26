package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name = "auto2")
public class Odometry_Test2 extends LinearOpMode
{
    SampleMecanumDrive srobot;
    @Override public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        srobot = new SampleMecanumDrive((hardwareMap));
        srobot.boxTilt.setPosition(0.4);
        srobot.boxArm.setPosition(0.25);

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.telescopicArm.setPosition(0.0);

        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.3);

        srobot.intakeTilt.setPosition(1.0);
        srobot.intakeArm.setPosition(0.8);

        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
        srobot.specimenTilt.setPosition(0.5);

        srobot.specimenHolder.setPosition(.65);
        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(24, 72, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,60))
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), 90)
                .lineTo(new Vector2d(0,38),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), 90)
                .lineTo(new Vector2d(0,60))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), 90)
                .lineToLinearHeading(new Pose2d(58,63, Math.toRadians(-130)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), -130)
                .lineToLinearHeading(new Pose2d(58,48, Math.toRadians(90)))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), 90)
                .lineToLinearHeading(new Pose2d(58,63, Math.toRadians(-130)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), -130)
                .lineToLinearHeading(new Pose2d(55,34,Math.toRadians(0)))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), 0)
                .lineToLinearHeading(new Pose2d(58,63, Math.toRadians(-130)))
                .build();


        //Next two lines are just for FTC OpModes
        waitForStart();

        if(isStopRequested()) return;

        //Robot drives along trajectory
        drive.followTrajectory(traj1);
        linSlideHigh();
        specimentTiltUp();
        sleep(2000);
        drive.followTrajectory(traj2);
        sleep(1000);
        specimentTiltDown();
        linSlideLow();
        sleep(1000);
        specimenOpen();
        drive.followTrajectory(traj3);
        sleep(1000);
        drive.followTrajectory(traj4);
        sleep(1000);
        drive.followTrajectory(traj5);
        sleep(1000);
        drive.followTrajectory(traj6);
        sleep(1000);
        drive.followTrajectory(traj7);
//        sleep(1000);
//        drive.followTrajectory(traj8);

//        sleep(2000);
//        telemetry.addLine("spline 2");
    }
    public void linSlideHigh(){
        srobot.telescopicArm.setPosition(0.3);
        sleep(300);
        srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
        srobot.boxArm.setDirection(Servo.Direction.FORWARD);
        srobot.linearSlide.setTargetPosition(1050);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void linSlideLow(){
        srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
        srobot.boxArm.setDirection(Servo.Direction.FORWARD);
        srobot.linearSlide.setTargetPosition(830);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void specimentTiltUp(){
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.6);
        srobot.specimenHolder.setPosition(.3);
    }
    public void specimentTiltDown(){
        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
        srobot.specimenTilt.setPosition(0.45);
        srobot.specimenHolder.setPosition(.3);
    }
    public void specimenOpen(){
        srobot.specimenHolder.setPosition(0.7);
    }
}