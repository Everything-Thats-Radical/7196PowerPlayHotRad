package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import androidx.annotation.RequiresApi;

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

@Autonomous(name = "A4_Left_2Cns_Prk")
public class A4_Left_2Cns_Prk extends LinearOpMode {


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
        3) spline to look at the near high junction
        4) raise the lift
        5) position cone over junction
        6) Drop cone
        7) back up
        8) Lower lift
        9) spline to correct parking location based on beacon reading
         */

        Pose2d startPose = new Pose2d(64.5, -36, 180); // beginning pose of this auto
        drive.setPoseEstimate(startPose); // make our localizer and path follower agree with each other

        Vector2d scanPos = new Vector2d(36, -36); // vector (location without heading) of the place where we will scan the cone
        Vector2d conePushPos = new Vector2d(24, -36);
        Vector2d liftToScorePos = new Vector2d(29.3, 29.3); // 135 degrees
        Vector2d scorePos = new Vector2d(28.24, -28.24); // 135 degrees
        Vector2d blueParkPos = new Vector2d(36, -12); // 135 degrees
        Vector2d greenParkPos = new Vector2d(36, -36); // 135 degrees
        Vector2d redParkPos = new Vector2d(36, -60); // 135 degrees


        Trajectory driveToScan = drive.trajectoryBuilder(startPose)
                .splineTo(scanPos, Math.toRadians(180))
                .build();

        Trajectory pushCone = drive.trajectoryBuilder(driveToScan.end())
                .splineTo(conePushPos, Math.toRadians(180))
                .build();

        Trajectory driveToPrepScore = drive.trajectoryBuilder(pushCone.end())
                .splineTo(liftToScorePos, Math.toRadians(135))
                .build();

        TrajectorySequence driveToScore = drive.trajectorySequenceBuilder(driveToPrepScore.end())
                .splineTo(scorePos, Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory backUpFromScore = drive.trajectoryBuilder(driveToScore.end())
                .splineTo(liftToScorePos, Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkBlue = drive.trajectoryBuilder(backUpFromScore.end())
                .splineTo(blueParkPos, Math.toRadians(180))
                .build();

        Trajectory parkGreen = drive.trajectoryBuilder(backUpFromScore.end())
                .splineTo(greenParkPos, Math.toRadians(180))
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(backUpFromScore.end())
                .splineTo(redParkPos, Math.toRadians(180))
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

        drive.followTrajectory(pushCone);// back 3, turn right, forward 38, turn left
        drive.followTrajectory(driveToPrepScore);
        clawControl("clamp");
        moveLift("up", 27, .5);
        drive.followTrajectorySequence(driveToScore);
        clawControl("open");
        drive.followTrajectory(backUpFromScore);
        moveLift("up", 33, .5);

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
            location = 0;
        } else if (state.equals("release")) {
            location = 1;
        }
        clampyBoi.setPosition(location);
    }
}