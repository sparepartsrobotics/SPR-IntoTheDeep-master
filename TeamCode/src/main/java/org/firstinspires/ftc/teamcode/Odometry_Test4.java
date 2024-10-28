package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Push neutral specimen: Blue")
public class Odometry_Test4 extends LinearOpMode
{
    SampleMecanumDrive srobot;
    @Override public void runOpMode()
    {

        srobot = new SampleMecanumDrive((hardwareMap));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        srobot.boxTilt.setPosition(0.4);
        srobot.boxArm.setPosition(0.25);

//        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
//        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
        Pose2d startPose = new Pose2d(-24, -72, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,-50))
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), -90)
                .lineTo(new Vector2d(0,-60))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), -90)
                .lineToLinearHeading(new Pose2d(-20,-48, Math.toRadians(2)))
                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj2.end(), 2)
//                .lineToLinearHeading(new Pose2d(-58,-63, Math.toRadians(50)))
//                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj3.end(), 50)
//                .lineToLinearHeading(new Pose2d(-58,-48, Math.toRadians(90)))
//                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj4.end(), 90)
//                .lineToLinearHeading(new Pose2d(-58,-63, Math.toRadians(50)))
//                .build();
//
//        Trajectory traj7 = drive.trajectoryBuilder(traj5.end(), 50)
//                .lineToLinearHeading(new Pose2d(-55,-34,Math.toRadians(180)))
//                .build();
//        Trajectory traj8 = drive.trajectoryBuilder(traj6.end(), 180)
//                .lineToLinearHeading(new Pose2d(-58,-63, Math.toRadians(50)))
//                .build();


        //Next two lines are just for FTC OpModes
        waitForStart();

        if(isStopRequested()) return;

        //Robot drives along trajectory
        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.followTrajectory(traj3);
        teleArm();
        intakeWheel();
        //drive.followTrajectory(traj3);

//        sleep(1000);
//        drive.followTrajectory(traj4);
//        sleep(1000);
//        drive.followTrajectory(traj5);
//        sleep(1000);
//        drive.followTrajectory(traj6);
//        sleep(1000);
//        drive.followTrajectory(traj7);
//        sleep(1000);
//        drive.followTrajectory(traj8);

//        sleep(2000);
//        telemetry.addLine("spline 2");
    }
    public void intakeWheel() {
            srobot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            long endtime2 = System.currentTimeMillis() + 2000;
            while (System.currentTimeMillis() < endtime2) {
                srobot.intake.setPower(1.0);
            }
            srobot.intake.setPower(0);
    }
    public void teleArm(){
        srobot.intakeTilt.setDirection(Servo.Direction.FORWARD);
        srobot.intakeTilt.setPosition(0.2);
        srobot.intakeArm.setDirection(Servo.Direction.FORWARD);
        srobot.intakeArm.setPosition(0.075);
    }
}