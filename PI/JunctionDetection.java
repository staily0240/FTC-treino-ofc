package org.firstinspires.ftc.teamcode.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.JunctionDetection_pt2;
//import org.firstinspires.ftc.teamcode.common.powerplay.junctionDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "yellow junction")
@Disabled
public class JunctionDetection extends LinearOpMode {

    private JunctionDetection_pt2 junctionDetection_pt2;
    private OpenCvCamera camera;

    // Nome da Webcam a ser definido na configuração
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        junctionDetection_pt2 = new junctionDetection();
        camera.setPipeline(junctionDetection_pt2);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", junctionDetection_pt2.getPosition());
            //telemetry.addData("X", TOPLEFT_ANCHOR_POINT.X.getPosition());
            //telemetry.addData("Y", TOPLEFT_ANCHOR_POINT.y.getPosition());
            telemetry.update();
        }

        waitForStart();
    }

    private class junctionDetection extends JunctionDetection_pt2 {
    }
}