package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Specimen Red")
public class SpecimenRed extends LinearOpMode
{
    SampleMecanumDrive srobot;
    @Override public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        srobot = new SampleMecanumDrive((hardwareMap));
        srobot.clawRotate.setPosition(0.5);
        srobot.clawArm.setPosition(0.66);
        srobot.claw.setPosition(1);
        srobot.clawTilt.setPosition(0.2);
        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.ascendArm.setMode(STOP_AND_RESET_ENCODER);
        srobot.ascendArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.telescopicArm.setPosition(0.0);

        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.75);
        srobot.box.setPosition(.7);
        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(16, -72, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(5,-44),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), -90)
                .lineTo(new Vector2d(40,-55))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), -90)
                .splineToConstantHeading(new Vector2d(46,-20), Math.toRadians(-90))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj4.end(), 90)
                .lineTo(new Vector2d(34,-65))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), -90)
                .splineToConstantHeading(new Vector2d(52,-20), Math.toRadians(90))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj7.end(), 90)
                .lineTo(new Vector2d(55,-65))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end(), 90)
                .lineTo(new Vector2d(55,-60))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end(), 90)
                .lineToLinearHeading(new Pose2d(49,-68, Math.toRadians(89)))
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj11.end(), 90)
                .lineTo(new Vector2d(49,-73),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj13.end(), 90)
                .lineToLinearHeading(new Pose2d(0,-60,Math.toRadians(-90)))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end(), 90)
                .lineTo(new Vector2d(-8,-44))
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end(), 90)
                .lineTo(new Vector2d(-8,-60))
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end(), 90)
                .lineToLinearHeading(new Pose2d(49,-68, Math.toRadians(90)))
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(traj17.end(), 90)
                .lineTo(new Vector2d(49,-73),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj19 = drive.trajectoryBuilder(traj18.end(), 90)
                .lineToLinearHeading(new Pose2d(0,-44,Math.toRadians(-90)))
                .build();
        Trajectory traj20 = drive.trajectoryBuilder(traj19.end(), -90)
                .lineToLinearHeading(new Pose2d(49,-68,Math.toRadians(-90)))
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
        specimentTiltUp();
        linSlideHigh();
        drive.followTrajectory(traj2);
        specimentTiltDown();
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        resetLinSlide();
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj9);
        specimenTiltSpec();
        specimenOpen();
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj13);
        specimenClose();
        linSlideHigh2();
        drive.followTrajectory(traj14);
        specimentTiltUp();
        drive.followTrajectory(traj15);
        specimentTiltDown();
        specimenOpen();
        drive.followTrajectory(traj16);
        resetLinSlide();
        specimenTiltSpec();
        specimenOpen();
        drive.followTrajectory(traj17);
        drive.followTrajectory(traj18);
        specimenClose();
        linSlideHigh2();
        drive.followTrajectory(traj19);
        specimentTiltDown();
        resetLinSlide();
        drive.followTrajectory(traj20);
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
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.6);
        srobot.telescopicArm.setPosition(0.3);
        sleep(300);
        srobot.linearSlide.setTargetPosition(1075);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void linSlideHigh2(){
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.6);
        srobot.telescopicArm.setPosition(0.3);
        sleep(300);
        srobot.linearSlide.setTargetPosition(1085);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void linSlideLow(){
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
        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
        srobot.specimenTilt.setPosition(0.6);
        sleep(500);
        srobot.specimenHolder.setPosition(.9);
    }
    public void specimenOpen(){
        srobot.specimenHolder.setPosition(0.85);
    }
    public void specimenTiltSpec(){
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.53);
    }
    public void specimenClose(){
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.6);
    }
    public void resetLinSlide(){
        srobot.telescopicArm.setPosition(0);
        sleep(300);
        srobot.linearSlide.setTargetPosition(0);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(0.85);
        srobot.specimenHolder.setPosition(0.75);
        srobot.specimenTilt.setPosition(0.55);

    }

}