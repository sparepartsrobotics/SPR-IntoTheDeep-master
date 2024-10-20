package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "auto3")
public class Odometry_Test3 extends LinearOpMode
{
    @Override public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(24, -72, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,-50), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), -90)
                .lineTo(new Vector2d(36,-50), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), -90)
                .lineTo(new Vector2d(45,-12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), -90)
                .lineTo(new Vector2d(45,-65), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), -90)
                .lineTo(new Vector2d(45,-12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), -90)
                .lineTo(new Vector2d(55,-12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), -90)
                .lineTo(new Vector2d(52,-65), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), -90)
                .lineTo(new Vector2d(52,-12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), -90)
                .lineTo(new Vector2d(65,-12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end(), -90)
                .lineTo(new Vector2d(65,-65), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //Next two lines are just for FTC OpModes
        waitForStart();

        if(isStopRequested()) return;

        //Robot drives along trajectory
        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.followTrajectory(traj3);
        sleep(1000);
        drive.followTrajectory(traj4);
        sleep(1000);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        sleep(1000);
        drive.followTrajectory(traj7);
        sleep(1000);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);

//        sleep(2000);
//        telemetry.addLine("spline 2");
    }
}