package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        Pose2d startPose = new Pose2d(0, -72, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,-50))
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), 90)
                .lineToLinearHeading(new Pose2d(36,-50, Math.toRadians(9)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), 0)
                .lineTo(new Vector2d(36,-30))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), 0)
                .lineTo(new Vector2d(48,-64))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), 0)
                .lineTo(new Vector2d(43,-30))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), 0)
                .lineTo(new Vector2d(60,-64))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), 0)
                .lineTo(new Vector2d(52,-28))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), 0)
                .lineTo(new Vector2d(60,-64))
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
        sleep(1000);
        drive.followTrajectory(traj6);
        sleep(1000);
        drive.followTrajectory(traj7);
        sleep(1000);
        drive.followTrajectory(traj8);

//        sleep(2000);
//        telemetry.addLine("spline 2");
    }
}