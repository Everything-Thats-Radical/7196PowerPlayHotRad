package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.signum;

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

@Autonomous(name = "A4_Left_2Cn_Prk")
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

        STRAIGHTUPPPP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        STRAIGHTUPPPP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        //Pose2d startPose = new Pose2d(64.5625, -41.625, Math.toRadians(180)); // beginning of auto
        Pose2d startPose = new Pose2d(64.5, -36, Math.toRadians(180)); // beginning of things after alignment location of this auto
        drive.setPoseEstimate(startPose); // make our localizer and path follower agree with each other


        TrajectorySequence driveToScan = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    clawControl("clamp");
                    setLift(6);
                })
                .forward(27.5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // SCAN CONE
        TrajectorySequence pushConeAndPositionOverLowForPreloadScore = drive.trajectorySequenceBuilder(driveToScan.end())
                .forward(30, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(18, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    setLift(16);
                })
                .turn(Math.toRadians(90))
                .forward(6.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    clawControl("release");
                })
                .build();
        // RAISE LIFT TO MEDIUM
        // DROP CONE
        Trajectory retreatFromPreloadScore = drive.trajectoryBuilder(pushConeAndLookAtLowForPreloadScore.end())
                .back(6.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // LOWER LIFT TO 5 STACK HEIGHT
        TrajectorySequence positionOverStack1stCone = drive.trajectorySequenceBuilder(retreatFromPreloadScore.end())
                .turn(Math.toRadians(-90))
                .forward(12, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .forward(30, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // PICK CONE UP
        TrajectorySequence lookAtMediumFromStack1stCone = drive.trajectorySequenceBuilder(positionOverStack1stCone.end())
                .back(42, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(90)
                .build();
        // RAISE LIFT TO MEDIUM
        TrajectorySequence positionOverMedium1stCone = drive.trajectorySequenceBuilder(lookAtMediumFromStack1stCone.end())
                .forward(6.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // DROP CONE
        TrajectorySequence retreatFromMedium1stCone = drive.trajectorySequenceBuilder(positionOverMedium1stCone.end())
                .back(6.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // LOWER LIFT TO 4 STACK HEIGHT
        // LOWER LIFT TO HEIGHT ZERO
        TrajectorySequence driveToGreenZone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(-90))
                .forward(12, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .build();
/*
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
*/

        waitForStart();

        if(isStopRequested()) return;

        clawControl("clamp");
        drive.followTrajectorySequence(driveToScan);


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

        drive.followTrajectorySequence(pushConeAndLookAtLowForPreloadScore);
        // RAISE LIFT TO MEDIUM
        // DROP CONE
        drive.followTrajectory(retreatFromPreloadScore);
        // LOWER LIFT TO 5 STACK HEIGHT
        drive.followTrajectorySequence(positionOverStack1stCone);
        // PICK CONE UP
        drive.followTrajectorySequence(lookAtMediumFromStack1stCone);
        // RAISE LIFT TO MEDIUM
        drive.followTrajectorySequence(positionOverMedium1stCone);
        // DROP CONE
        drive.followTrajectorySequence(retreatFromMedium1stCone);
        // LOWER LIFT TO 0
        drive.followTrajectorySequence(driveToGreenZone);
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

    public void setLift(double inches) {
        double ticks_per_inch = 180;
        double desiredLiftPosition = inches * ticks_per_inch;
        double currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
        double ticksNeeded = desiredLiftPosition - currentLiftPosition;
        while (Math.abs(ticksNeeded) > 20){
            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
            ticksNeeded = desiredLiftPosition - currentLiftPosition;
            STRAIGHTUPPPP.setPower(1 * signum(ticksNeeded));
            telemetry.addData("desired lift position: ", desiredLiftPosition);
            telemetry.addData("current lift position: ", currentLiftPosition);
            telemetry.addData("ticks needed: ", ticksNeeded);
            telemetry.addData("power: ", 1 * signum(ticksNeeded));
            telemetry.update();
        }
        STRAIGHTUPPPP.setPower(0);
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