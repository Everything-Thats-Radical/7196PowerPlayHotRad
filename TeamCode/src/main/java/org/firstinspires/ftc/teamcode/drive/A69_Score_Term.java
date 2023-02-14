

package org.firstinspires.ftc.teamcode.drive;

import android.app.Activity;
import android.os.Build;
import android.view.View;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/* Our 3rd in command, Troy Tomson, has made an amazing piece of art that is this code.
Although janky, we accept it as beautiful, brave, and stunning. If you don't accept it, you are
literally codiphobic and want all the poor starving children in Africa to die.
*/

@Autonomous(name = "A69_Score_Term")
public class A69_Score_Term extends LinearOpMode {

    // create motor and servo objects
    private Servo clampyBoi = null;
    private DcMotor STRAIGHTUPPPP = null;

    //final double ticks_per_inch = (1120 / (2.952 * 2 * Math.PI));

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
        // hsvValues is an array that will hold the hue, saturation, and value information.

        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.

        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.

        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d scanPose = new Pose2d(0, -25, 0);
        drive.setPoseEstimate(startPose);

        // send the info back to driver station using telemetry function.

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        // TRAJECTORY SEQUENCES BUILT
        //----------------------------------

        TrajectorySequence displaceCone = drive.trajectorySequenceBuilder(startPose)
                .forward(24,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(24,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        waitForStart();

        // START OF AUTO MOVEMENT ----------------------------------------------
        if (!isStopRequested()) {
            drive.followTrajectorySequence(displaceCone);
        }

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Troy: The scoreCone trajectory sequence does not score the cone.
        // Isaiah: Why?
        // Troy: *Walks away*

    }
}


