package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

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

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;

@Autonomous(name = "A4_Left_4Cn_Prk")
public class A4_Left_4Cn_Prk extends LinearOpMode {

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

        /*
                        .addDisplacementMarker(() -> {
                    setLift(10.3);
                })
         */
        TrajectorySequence driveToSmallPreload = drive.trajectorySequenceBuilder(startPose)
                /*
                .addDisplacementMarker(() -> {
                    setLift(16);
                })

                 */
                .addTemporalMarker(.1, () -> {
                    setLift(16);
                })
                .lineToSplineHeading(new Pose2d(50.4, -27.9, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence returnToStartPose = drive.trajectorySequenceBuilder(driveToSmallPreload.end())
                .lineToSplineHeading(new Pose2d(60, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory driveToScan = drive.trajectoryBuilder(returnToStartPose.end())
                .addTemporalMarker(.1, () -> {
                    setLift(10.5);
                })
                .splineTo(new Vector2d(45, -35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(42, -35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence pushConeAndDriveToStack1stCone = drive.trajectorySequenceBuilder(driveToScan.end())
                .lineToSplineHeading(new Pose2d(9, -35, Math.toRadians(180)),//45.5
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(12, -35, Math.toRadians(180)),//45.5
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    setLift(10);
                })
                .splineTo(new Vector2d(12, 55), Math.toRadians(270),//45.5
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(12, 61.5), Math.toRadians(270),//45.5
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // PICK CONE UP
        TrajectorySequence driveToMedium1stCone = drive.trajectorySequenceBuilder(pushConeAndDriveToStack1stCone.end())
                .addTemporalMarker(.6, () -> {
                    setLift(25);
                })
                .lineToSplineHeading(new Pose2d(12, -12, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20.5, -18, Math.toRadians(315)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence driveToStack2ndCone = drive.trajectorySequenceBuilder(driveToMedium1stCone.end())
                .addTemporalMarker(.5, () -> {
                    setLift(10);
                })
                .back(4, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(13, -55), Math.toRadians(270),//45.5
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(13, -61.5), Math.toRadians(270),//45.5
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence driveToSmall3rdCone = drive.trajectorySequenceBuilder(driveToStack2ndCone.end())
                .addTemporalMarker(.5, () -> {
                    setLift(15);
                })
                .lineToSplineHeading(new Pose2d(12, -55, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(21, -51, Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence alignToParkingZones = drive.trajectorySequenceBuilder(driveToSmall3rdCone.end())
                .lineToSplineHeading(new Pose2d(12, -55, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence driveToGreen = drive.trajectorySequenceBuilder(alignToParkingZones.end())
                .lineToSplineHeading(new Pose2d(14, -36, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence driveToRed = drive.trajectorySequenceBuilder(alignToParkingZones.end())
                .lineToSplineHeading(new Pose2d(16, -12, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        /*
        TrajectorySequence driveToSmall3rdCone = drive.trajectorySequenceBuilder(driveToStack2ndCone.end())
                .addTemporalMarker(1, () -> {
                    setLift(15);
                })
                .lineToSplineHeading(new Pose2d(12, 33, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20, 44, Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
*/


        /*
        TrajectorySequence driveToScan = drive.trajectorySequenceBuilder(startPose)
                .forward(19,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .forward(3.5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//5
                .build();
        // SCAN CONE
        TrajectorySequence pushConeAndLookAtLowForPreloadScore = drive.trajectorySequenceBuilder(driveToScan.end())
                .forward(35, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//40
                .back(18, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//40
                .addDisplacementMarker(() -> {
                    clawControl("clamp");
                    setLift(16);
                })
                .turn(Math.toRadians(-90))
                .build();
        // RAISE LIFT TO MEDIUM
        Trajectory positionOverLowPreload = drive.trajectoryBuilder(pushConeAndLookAtLowForPreloadScore.end())
                .forward(5.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//10
                .build();
        // DROP CONE
        Trajectory retreatFromPreloadScore = drive.trajectoryBuilder(positionOverLowPreload.end())
                .back(5.5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//10
                .build();
        // LOWER LIFT TO 5 STACK HEIGHT
        TrajectorySequence positionOverStack1stCone = drive.trajectorySequenceBuilder(retreatFromPreloadScore.end())
                .addDisplacementMarker(() -> {
                    setLift(10.3);
                })
                .turn(Math.toRadians(90))
                .forward(12, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .turn(Math.toRadians(-90))
                .forward(23, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .forward(4, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//10
                .build();
        // PICK CONE UP
        TrajectorySequence lookAtMediumFromStack1stCone = drive.trajectorySequenceBuilder(positionOverStack1stCone.end())
                .back(39.5, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    setLift(20);
                })
                .build();
        // RAISE LIFT TO MEDIUM
        TrajectorySequence positionOverMedium1stCone = drive.trajectorySequenceBuilder(lookAtMediumFromStack1stCone.end())
                .forward(6, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//10
                .build();
        // DROP CONE
        TrajectorySequence retreatFromMedium1stCone = drive.trajectorySequenceBuilder(positionOverMedium1stCone.end())
                .back(6, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//10
                .build();
        // LOWER LIFT TO 4 STACK HEIGHT
        // LOWER LIFT TO HEIGHT ZERO
        TrajectorySequence driveToGreenZone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(90))
                .forward(11, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence driveToRedZone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(90))
                .back(12, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//50
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence driveToBlueZone = drive.trajectorySequenceBuilder(retreatFromMedium1stCone.end())
                .turn(Math.toRadians(90))
                .forward(32, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//30
                .turn(Math.toRadians(90))
                .build();
*/

        waitForStart();

        if(isStopRequested()) return;

        clawControl("clamp"); // CLAMP INITIAL CONE
        sleep(400);

        drive.followTrajectorySequence(driveToSmallPreload);

        setLift(14);
        clawControl("release"); // DROP CONE
        sleep(500);
        //setLift(16);
        clawControl("clamp");

        drive.followTrajectorySequence(returnToStartPose);

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

        drive.followTrajectorySequence(pushConeAndDriveToStack1stCone);

        clawControl("release"); // PICK CONE UP
        sleep(300);
        setLift(4.9);
        clawControl("clamp");
        sleep(300);
        setLift(10);

        drive.followTrajectorySequence(driveToMedium1stCone);

        setLift(23.5);
        clawControl("release"); // DROP CONE
        sleep(500);
        //setLift(25);
        clawControl("clamp");

        drive.followTrajectorySequence(driveToStack2ndCone);

        clawControl("release"); // PICK CONE UP
        sleep(300);
        setLift(5);
        clawControl("clamp");
        sleep(300);
        setLift(10);

        drive.followTrajectorySequence(driveToSmall3rdCone);
        clawControl("release"); // DROP CONE
        sleep(500);
        setLift(14);
        clawControl("clamp");

        drive.followTrajectorySequence(alignToParkingZones);

        if(coneColor == "red"){
            drive.followTrajectorySequence(driveToRed);
        } else if (coneColor == "green"){
            drive.followTrajectorySequence(driveToGreen);
        }else if (coneColor == "blue"){

        }

        /*
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

        telemetry.addData("reds: ", totalReds);
        telemetry.addData("greens: ", totalGreens);
        telemetry.addData("blues: ", totalBlues);
        telemetry.addData("Cone color", coneColor);
        telemetry.update();


        drive.followTrajectorySequence(pushConeAndLookAtLowForPreloadScore); // LIFT IS SET TO 16 IN DURING THIS TRAJECTORY SEQUENCE

        drive.followTrajectory(positionOverLowPreload);
        setLift(14);
        clawControl("release"); // DROP CONE
        sleep(300);
        setLift(16);
        clawControl("clamp");

        drive.followTrajectory(retreatFromPreloadScore);

        drive.followTrajectorySequence(positionOverStack1stCone); // LIFT IS LOWERED TO 10.3 IN (5 CONE HEIGHT) DURING THIS
        clawControl("release"); // PICK CONE UP
        sleep(300);
        setLift(5);
        clawControl("clamp"); // PICK CONE UP
        sleep(300);
        setLift(10);

        drive.followTrajectorySequence(lookAtMediumFromStack1stCone); // LIFT IS SET TO 20 IN DURING THIS TO SAVE TIME
        setLift(25.5); // RAISE LIFT TO MEDIUM

        drive.followTrajectorySequence(positionOverMedium1stCone);
        setLift(23.5);
        clawControl("release");
        sleep(300); // DROP CONE
        setLift(25.5);
        clawControl("clamp");

        drive.followTrajectorySequence(retreatFromMedium1stCone);
        setLift(0);

        if (coneColor.equals("red")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectorySequence(driveToRedZone);

        } else if (coneColor.equals("green")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectorySequence(driveToGreenZone);

        } else if (coneColor.equals("blue")) {
            telemetry.addData("Color: ", coneColor);
            telemetry.update();
            drive.followTrajectorySequence(driveToBlueZone);

        } else {
            String noColor = "Color not detected.";
            telemetry.addData("Color: ", noColor);
            telemetry.update();
        }
*/
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
        double ticks_per_inch = 134;
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