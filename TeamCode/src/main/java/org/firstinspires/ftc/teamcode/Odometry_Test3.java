package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name = "2 cycle specimen: Red")
public class Odometry_Test3 extends LinearOpMode
{
    SampleMecanumDrive srobot;
    @Override public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        srobot = new SampleMecanumDrive((hardwareMap));
        srobot.specimenHolder.setPosition(0.3);
        srobot.boxTilt.setPosition(0.4);
        srobot.boxArm.setPosition(0.25);

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.telescopicArm.setPosition(0.0);




        srobot.intakeTilt.setPosition(1.0);
        srobot.intakeArm.setPosition(0.8);

        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
        srobot.specimenTilt.setPosition(0.2);
        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(8, -72, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(7,-60))
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), -90)
                .lineTo(new Vector2d(7,-40),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), -90)
                .lineTo(new Vector2d(7,-65))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), -90)
                .lineTo(new Vector2d(30,-65))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), -90)
                .lineTo(new Vector2d(46,-12))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), -90)
                .lineTo(new Vector2d(46,-65))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), -90)
                .lineTo(new Vector2d(40,-12))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), -90)
                .lineTo(new Vector2d(52,-12))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), -90)
                .lineTo(new Vector2d(55,-65))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end(), -90)
                .lineTo(new Vector2d(55,-60))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end(), -90)
                .lineToLinearHeading(new Pose2d(49,-65, Math.toRadians(88)))
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(), 88)
                .lineTo(new Vector2d(48.5,-71))
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end(), 88)
                .lineTo(new Vector2d(49,-68))
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj13.end(), 88)
                .lineToLinearHeading(new Pose2d(0,-60,Math.toRadians(-90)))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end(), -90)
                .lineTo(new Vector2d(0,-41))
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end(), -90)
                .lineTo(new Vector2d(0,-60))
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end(), -90)
                .lineTo(new Vector2d(48,-65))
                .build();
//        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), 90)
//                .lineTo(new Vector2d(0,42),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), 90)
//                .lineTo(new Vector2d(0,65))
//                .build();
//

//        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(), -90)
//                .lineTo(new Vector2d(-49,65))
//                .build();
//        Trajectory traj13 = drive.trajectoryBuilder(traj12.end(), -90)
//                .lineToLinearHeading(new Pose2d(7,60,Math.toRadians(90)))
//                .build();
//        Trajectory traj14 = drive.trajectoryBuilder(traj13.end(), 90)
//                .lineTo(new Vector2d(7,41),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory traj15 = drive.trajectoryBuilder(traj14.end(), 90)
//                .lineTo(new Vector2d(7,65))
//                .build();


        //Next two lines are just for FTC OpModes
        waitForStart();

        if(isStopRequested()) return;

        //Robot drives along trajectory
        srobot.specimenTilt.setPosition(0.5);
        drive.followTrajectory(traj1);
        linSlideHigh();
        specimentTiltUp();
        sleep(500);
        drive.followTrajectory(traj2);
        sleep(500);
        specimentTiltDown();
        drive.followTrajectory(traj3);
        resetLinSlide();
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);
        sleep(500);
        specimenClose();
        specimentTiltUp();
        drive.followTrajectory(traj13);
        drive.followTrajectory(traj14);
        linSlideHigh();
        specimentTiltUp();
        sleep(500);
        drive.followTrajectory(traj15);
        specimentTiltDown();
        sleep(500);
        specimenOpen();
        drive.followTrajectory(traj16);
        drive.followTrajectory(traj17);
//        drive.followTrajectory(traj6);
//        drive.followTrajectory(traj7);
//        linSlideHigh();
//        specimentTiltUp();
//        drive.followTrajectory(traj8);
//        sleep(500);
//        specimentTiltDown();
//        drive.followTrajectory(traj9);
//        resetLinSlide();
//        drive.followTrajectory(traj10);
//        drive.followTrajectory(traj11);
//        sleep(500);
//        specimentTiltUp();
//        drive.followTrajectory(traj12);
//        drive.followTrajectory(traj13);
//        linSlideHigh();
//        specimentTiltUp();
//        sleep(1000);
//        drive.followTrajectory(traj14);
//        sleep(500);
//        specimentTiltDown();
//        drive.followTrajectory(traj15);
//        specimentTiltUp();

//        sleep(2000);
//        telemetry.addLine("spline 2");
    }
    public void linSlideHigh(){
        srobot.intakeTilt.setPosition(.4);
        srobot.intakeArm.setPosition(.7);
        sleep(300);
        srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
        srobot.boxArm.setDirection(Servo.Direction.FORWARD);
        srobot.linearSlide.setTargetPosition(1050);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
        srobot.boxTilt.setPosition(0.4);
        srobot.specimenHolder.setPosition(.3);
    }
    public void linSlideLow(){
        srobot.boxTilt.setDirection(Servo.Direction.FORWARD);
        srobot.boxArm.setDirection(Servo.Direction.FORWARD);
        srobot.linearSlide.setTargetPosition(830);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void specimentTiltUp(){
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.6);

    }
    public void specimentTiltDown(){
        srobot.specimenHolder.setPosition(.28);
        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
        srobot.specimenTilt.setPosition(0.6);

    }
    public void specimenOpen(){
        srobot.specimenHolder.setPosition(0.7);
    }
    public void specimenClose(){
        srobot.specimenHolder.setPosition(.3);
    }
    public void resetLinSlide(){
        srobot.specimenTilt.setPosition(.45);
        srobot.telescopicArm.setPosition(0);
        sleep(300);
        srobot.linearSlide.setTargetPosition(0);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(0.85);
        srobot.specimenHolder.setPosition(0.75);

    }
//    public void extendTeleArm(){
//        srobot.intakeTilt.setDirection(Servo.Direction.FORWARD);
//        srobot.intakeTilt.setPosition(0.19);
//        srobot.intakeArm.setDirection(Servo.Direction.FORWARD);
//        srobot.intakeArm.setPosition(0.07);
//        srobot.telescopicArm.setPosition(.5);
//        long goTime = System.currentTimeMillis()+2000;
//        while (System.currentTimeMillis() < goTime) {
//            srobot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
//            srobot.intake.setPower(1);
//        }
//
//    }
}