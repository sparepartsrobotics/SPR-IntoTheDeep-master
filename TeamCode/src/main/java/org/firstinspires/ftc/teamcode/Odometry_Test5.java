package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;


@Autonomous(name = "HuskyLens Test", group = "advanced")
@Disabled
public class Odometry_Test5 extends LinearOpMode
{   SampleMecanumDrive srobot;
    private HuskyLens huskyLens;
    @Override public void runOpMode()
    {

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//        srobot = new SampleMecanumDrive((hardwareMap));
//        srobot.specimenHolder.setPosition(0.3);
//        srobot.boxTilt.setPosition(0.4);
//        srobot.boxArm.setPosition(0.25);
//
//        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
//        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        srobot.telescopicArm.setPosition(0.0);
//
//        srobot.specimenTilt.setPosition(.6);
//
//
//        srobot.intakeTilt.setPosition(1.0);
//        srobot.intakeArm.setPosition(0.8);
//
//        srobot.specimenTilt.setDirection(Servo.Direction.REVERSE);
//        srobot.specimenTilt.setPosition(0.2);

        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(0, 55, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        int x_pos = -1;
        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(
                        new Vector2d(-60,55)

                )
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectoryAsync(traj1);
        while(opModeIsActive() && !isStopRequested() && x_pos == -1){
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if(blocks.length >= 1){
                telemetry.addData("Block count", blocks.length);
                for(int i = 0; i < blocks.length; i++){
                    if(blocks[i].id == 1 && blocks[i].width > 10){
                        x_pos = blocks[i].x;
                        if(blocks[i].x > 20 && blocks[i].x < 50){
                            drive.breakFollowing();

                            drive.setDrivePower(new Pose2d());
                        }
                        drive.update();
                        telemetry.addData("x_pos", x_pos);
                    }
                }
            }
        }

    }

}