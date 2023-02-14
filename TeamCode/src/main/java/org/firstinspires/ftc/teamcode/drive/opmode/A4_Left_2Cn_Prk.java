package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "A4_Left_1Cn_Prk")
public class A4_Left_2Cn_Prk extends LinearOpMode {

    private Servo clampyBoi = null;
    private DcMotor STRAIGHTUPPPP = null;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() {

        clampyBoi = hardwareMap.get(Servo.class, "clampyBoi");
        STRAIGHTUPPPP = hardwareMap.get(DcMotor.class, "STRAIGHTUPPPP");

        clampyBoi.setDirection(Servo.Direction.FORWARD);
        STRAIGHTUPPPP.setDirection(DcMotor.Direction.REVERSE);

        STRAIGHTUPPPP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // get a reference to the distance sensor that shares the same name.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        /*
        *This auto should: *

        1) drive to scan the beacon
        2) scan the beacon and obtain its color (implying location)
        3) push cone out of way
        4) spline to look at the near high junction
        5) raise the lift
        6) position cone over junction
        7) Drop cone
        8) back up
        9) Lower lift
        10) spline to correct parking location based on beacon reading
         */

        Pose2d startPose = new Pose2d(64.5, -36, Math.toRadians(180)); // beginning pose of this auto
        drive.setPoseEstimate(startPose); // make our localizer and path follower agree with each other
/*
        Pose2d scanPos = new Pose2d(36, -36, Math.toRadians(180));
        Pose2d conePushPos = new Pose2d(12, -36, Math.toRadians(180));

        Pose2d lookingAtMediumPreload = new Pose2d(24, -30, Math.toRadians(90));
        Pose2d overMediumPreload = new Pose2d(24, -30, Math.toRadians(90));

        Pose2d lookingAtMedium = new Pose2d(12, -24, Math.toRadians(0));
        Pose2d overMedium = new Pose2d(18, -24, Math.toRadians(0));

        // CHANGE THE FOLLOWING TWO POSES
        Pose2d lookingAtStack = new Pose2d(12, -60, Math.toRadians(270));
        Pose2d overStack = new Pose2d(18, -24, Math.toRadians(0));

        Pose2d blueParkPos = new Pose2d(36, -12, Math.toRadians(90));
        Pose2d greenParkPos = new Pose2d(36, -36, Math.toRadians(90));
        Pose2d redParkPos = new Pose2d(36, -60, Math.toRadians(90));
*/

        Trajectory driveToScan = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence pushConeAndLookAtMediumForPreloadScore = drive.trajectorySequenceBuilder(driveToScan.end())
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(24, -36, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(-90))
                .build();
                // RAISE LIFT TO MEDIUM
        Trajectory positionOverMediumPreload = drive.trajectoryBuilder(pushConeAndLookAtMediumForPreloadScore.end())
                .lineToLinearHeading(new Pose2d(24, -30, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // DROP CONE
        Trajectory retreatFromPreloadScore = drive.trajectoryBuilder(positionOverMediumPreload.end())
                .lineToLinearHeading(new Pose2d(24, -36, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // LOWER LIFT TO 5 STACK HEIGHT
        TrajectorySequence lookAtStackFromPreloadScore = drive.trajectorySequenceBuilder(retreatFromPreloadScore.end())
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence positionOverStack1stCone = drive.trajectorySequenceBuilder(lookAtStackFromPreloadScore.end())
                .lineToLinearHeading(new Pose2d(12, -66, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // PICK CONE UP
        TrajectorySequence lookAtMediumFromStack1stCone = drive.trajectorySequenceBuilder(positionOverStack1stCone.end())
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .build();
                // RAISE LIFT TO MEDIUM
        TrajectorySequence positionOverMedium1stCone = drive.trajectorySequenceBuilder(lookAtMediumFromStack1stCone.end())
                .lineToLinearHeading(new Pose2d(18, -24, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // DROP CONE
        TrajectorySequence retreatFromMedium1stCone = drive.trajectorySequenceBuilder(positionOverMedium1stCone.end())
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // LOWER LIFT TO 4 STACK HEIGHT
        TrajectorySequence positionOverStack2ndCone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(12, -66, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                //PICK UP CONE
        TrajectorySequence lookAtMediumFromStack2ndCone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .build();
                // RAISE LIFT TO MEDIUM
        TrajectorySequence positionOverMediumFromStack2ndCone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .lineToLinearHeading(new Pose2d(18, -24, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // DROP CONE
        TrajectorySequence retreatFromMedium2ndCone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                // LOWER LIFT TO HEIGHT ZERO
        TrajectorySequence driveToGreenZone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(-90))
                .build();
        Trajectory parkBlue = drive.trajectoryBuilder(driveToGreenZone.end())
                .lineToLinearHeading(new Pose2d(60, -36, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory parkRed = drive.trajectoryBuilder(driveToGreenZone.end())
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        clawControl("clamp");
        moveLift("up", 6, .7);
        drive.followTrajectory(driveToScan);
        String coneColor = "green";

        int totalReds = colorSensor.red();
        int totalGreens = colorSensor.green();
        int totalBlues = colorSensor.blue();

        if (totalReds > totalGreens && totalReds > totalBlues) {
            coneColor = "red";
        } else if (totalGreens > totalBlues && totalGreens > totalReds) {
            coneColor = "green";
        } else if (totalBlues > totalGreens && totalBlues > totalReds) {
            coneColor = "blue";
        }

        drive.followTrajectorySequence(pushConeAndRetreat);// back 3, turn right, forward 38, turn left
        drive.followTrajectory(driveToPrepScore);
        /*
        clawControl("clamp");
        moveLift("up", 27, .5);
        drive.followTrajectorySequence(driveToScore);
        clawControl("open");
        drive.followTrajectory(backUpFromScore);
        moveLift("down", 33, .5);

        if (coneColor.equals("red")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectory(parkRed);

        } else if (coneColor.equals("green")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectory(parkGreen);

        } else if (coneColor.equals("blue")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectory(parkBlue);

        } else {
            String noColor = "Color not detected.";
            telemetry.addData("Color: ", noColor);
            telemetry.update();
        }

         */
    }

    public void moveLift(String direction, double height, double power) {
        double revsToInch = (2 * Math.PI);
        double ticks_per_inch = (1120 / revsToInch); //TICKS PER INCH MAY BE INCORRECT
        // THE LIFT WENT HIGHER THAN EXPECTED LAST TIME THIS WAS RUN AND BROKE THE LIFT
        // FIND ACTUAL TICKS PER INCH BEFORE RUNNING
        double ticksNeeded = height * ticks_per_inch;
        double initialPosition = STRAIGHTUPPPP.getCurrentPosition();
        double currentPosition = STRAIGHTUPPPP.getCurrentPosition();
        int directionSign = 1;
        double ticksMoved;

        if (direction.equals("up")) {
            directionSign = 1;
        } else if (direction.equals("down")) {
            directionSign = -1;
        } else {
            directionSign = -1;
        }

        ticksMoved = Math.abs(initialPosition - currentPosition);
        while (ticksNeeded > ticksMoved) {
            currentPosition = STRAIGHTUPPPP.getCurrentPosition();
            ticksMoved = Math.abs(initialPosition - currentPosition);

            STRAIGHTUPPPP.setPower(power * directionSign);
        }
        STRAIGHTUPPPP.setPower(0);
        clawControl("clamp");
    }

    public void clawControl(String state) {
        double location = 0;

        if (state.equals("clamp")) {
            location = .01;
        } else if (state.equals("release")) {
            location = .12;
        }
        clampyBoi.setPosition(location);
    }
}