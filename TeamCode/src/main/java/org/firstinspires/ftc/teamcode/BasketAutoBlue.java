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

@Autonomous(name = "Basket Blue")
public class BasketAutoBlue extends LinearOpMode
{   SampleMecanumDrive srobot;
    public double frMotorPower = 1.0;
    public double flMotorPower = 1.0;
    public double brMotorPower = 1.0;
    public double blMotorPower = 1.0;
    public boolean hasBlock = false;

    //public double incTemp= 0.0;
    //public double decTemp= 1.0;
    public double inc = 0.0005;
    public double telePosition = 0.0;

    public double clawArmUp = 0.05;//0.7;
    public double clawTiltUp = .35;//.3

    public double clawArmDownHigh = 0.32;
    public double clawTiltDown = .94;//.87
    public double clawArmDownLow = 0.45;
    public double clawArmOut = 0.3;

    public double clawOpen = 1.0;

    public double clawClose = 0.825;
    @Override public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        srobot = new SampleMecanumDrive((hardwareMap));

        initializeRobot();

//        srobot.rightReleaseArm.setPosition(0.5);
//        srobot.leftReleaseArm.setPosition(0.5);




        /** Each coordinate is about an inch */
        /** Remember that 0,0 is in the center of the field */
        /** On the red side start pos, right (+) and left (-) are x, and */
        /** forward (+) and backward (-) are y */
        /** Path from right red alliance station to facing red backdrop */
        //Creates starting position
        Pose2d startPose = new Pose2d(16, 72, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Creates the robot's trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(
                        new Vector2d(8,43.5)

                )
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2x = drive.trajectoryBuilder(traj1.end(), 0)
                .lineTo(
                        new Vector2d(10,50)

                )
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2= drive.trajectoryBuilder(traj2x.end(), 0)
                .lineToLinearHeading(new Pose2d(49.5, 56, Math.toRadians(-90)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), -90)
                .lineToLinearHeading(new Pose2d(58, 69, Math.toRadians(-130)))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), -130)
                .lineToLinearHeading(new Pose2d(59, 56.5, Math.toRadians(-90)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), -90)
                .lineToLinearHeading(new Pose2d(58, 69, Math.toRadians(-130)))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), -130)
                .lineTo(new Vector2d(57, 10))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), -130)
                .lineToLinearHeading(
                        new Pose2d(23, 10, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        /*
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), 90)
                .lineTo(new Vector2d(45,12))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), 90)
                .lineTo(new Vector2d(45,70))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), 90)
                .lineTo(new Vector2d(45,12))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), 90)
                .lineTo(new Vector2d(55,12))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), 90)
                .lineTo(new Vector2d(55,64))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end(), 90)
                .lineTo(new Vector2d(63,12))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end(), 90)
                .lineTo(new Vector2d(63,64))
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(), 90)
                .lineToLinearHeading(new Pose2d(20,0, Math.toRadians(0)))
                .build();
*/
        //Next two lines are just for FTC OpModes
        waitForStart();

        if(isStopRequested()) return;

        //Robot drives along trajectory
        specimentTiltUp();
        linSlideHigh();
        drive.followTrajectory(traj1);
        specimentTiltDown();
        //NeutralSample #1
        drive.followTrajectory(traj2x);
        drive.followTrajectory(traj2);
        resetLinSlide();
        bringClawArmDown();
        sleep(800);
        pickUpSample();
        drive.followTrajectory(traj3);
        linSlideHighBasket();
        sleep(1300);
        releaseSample();
        resetLinSlide();
        srobot.clawArm.setPosition(0.05);
        //Neutral Sample #2
        drive.followTrajectory(traj4);
        bringClawArmDown();
        sleep(1000);
        pickUpSample2();
        drive.followTrajectory(traj5);
        linSlideHighBasket();
        sleep(1300);
        releaseSample();
        resetLinSlide();
        drive.followTrajectory(traj6);
        releaseSampleEnd();
        drive.followTrajectory(traj7);

        /*
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);
*/
    }
    public void releaseSampleEnd(){
        srobot.clawArm.setPosition(0.2);
        srobot.box.setPosition(0);
        sleep(500);

    }
    public void linSlideHigh(){
        srobot.linearSlide.setTargetPosition(1000);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void linSlideHighBasket(){
        specimenClose();
        srobot.claw.setPosition(1.0);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(.8);
        sleep(300);
        srobot.linearSlide.setTargetPosition(2300);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
        bringClawArmDown();
    }

    public void releaseSample() {
        srobot.box.setPosition(.2);
        sleep(1000);
    }
    public void linSlideLow(){
        srobot.linearSlide.setTargetPosition(830);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(1);
    }
    public void specimentTiltUp(){
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setDirection(Servo.Direction.FORWARD);
        srobot.specimenTilt.setPosition(0.65);

    }
    public void specimentTiltDown(){
        srobot.specimenHolder.setPosition(.3);
        srobot.specimenTilt.setPosition(0.05);
        sleep(500);
        specimenOpen();
    }
    public void specimenOpen(){
        srobot.specimenHolder.setPosition(0.85);
    }
    public void specimenClose(){
        srobot.specimenHolder.setPosition(.3);
    }
    public void resetLinSlide(){
        srobot.box.setPosition(.7);
        specimentTiltUp();
        srobot.linearSlide.setTargetPosition(0);
        srobot.linearSlide.setMode(RUN_TO_POSITION);
        srobot.linearSlide.setPower(0.85);
        srobot.specimenHolder.setPosition(.5);
        srobot.clawRotate.setPosition(0.5);
        srobot.clawArm.setPosition(0.05);
        srobot.claw.setPosition(0.825);
        srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
        srobot.clawTilt.setPosition(0.2);

    }
    public void bringClawArmDown() {
        srobot.claw.setPosition(1.0);
        srobot.clawArm.setPosition(0.45);
        sleep(200);
        srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
        srobot.clawTilt.setPosition(clawTiltDown);
        sleep(1000);
        srobot.claw.setPosition(0.825);
    }
    public void pickUpSample() {
        srobot.clawTilt.setPosition(0.33);
        srobot.clawArm.setPosition(0.05);
    }
    public void pickUpSample2() {
        srobot.clawArm.setPosition(0.05);
        srobot.clawTilt.setPosition(0.33);
        sleep(1000);
    }
    public void initializeRobot(){
        srobot = new SampleMecanumDrive((hardwareMap));

        srobot.linearSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.ascendArm.setMode(STOP_AND_RESET_ENCODER);
        srobot.ascendArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        srobot.box.setPosition(.7);
        srobot.clawRotate.setPosition(0.5);
        srobot.claw.setPosition(0.825);
        srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
        srobot.clawTilt.setPosition(clawTiltUp);
        srobot.telescopicArm.setPosition(0.0);
        srobot.specimenTilt.setPosition(0.75);
        srobot.specimenHolder.setPosition(0.75);
        srobot.clawArm.setPosition(clawArmUp);
    }
}