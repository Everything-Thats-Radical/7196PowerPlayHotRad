package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

import android.graphics.Bitmap;
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.ColorDetection;
import org.firstinspires.ftc.teamcode.drive.ColorDetectionNuevo;
import org.firstinspires.ftc.teamcode.drive.ConceptWebcam;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "A5_ColorDetectionTest")
public class A5_ColorDetectionTest extends LinearOpMode {

    private Servo clampyBoi = null;
    private DcMotor STRAIGHTUPPPP = null;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clampyBoi = hardwareMap.get(Servo.class, "clampyBoi");
        STRAIGHTUPPPP = hardwareMap.get(DcMotor.class, "STRAIGHTUPPPP");
        clampyBoi.setDirection(Servo.Direction.FORWARD);
        STRAIGHTUPPPP.setDirection(DcMotor.Direction.REVERSE);
        STRAIGHTUPPPP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        STRAIGHTUPPPP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        STRAIGHTUPPPP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // get a reference to the distance sensor that shares the same name.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        ConceptWebcam cam = new ConceptWebcam();
        ColorDetectionNuevo detect = new ColorDetectionNuevo();

        Bitmap bmp;
        String color = "null";

        cam.callbackHandler = CallbackLooper.getDefault().getHandler();
        cam.cameraManager = ClassFactory.getInstance().getCameraManager();
        cam.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        cam.initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(cam.captureDirectory);


        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


        waitForStart();

        if(isStopRequested()) return;

        try{
            telemetry.addData("Ready", "");
            telemetry.update();
            while (cam.frameQueue.peek() == null){
                cam.openCamera();
                cam.startCamera();
                sleep(500);
            }
            bmp = cam.frameQueue.peek();
            color = detect.getColor(bmp);


        } finally{
            cam.closeCamera();
        }
        telemetry.addData("color: ", color);
        telemetry.update();
        sleep(10000);

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