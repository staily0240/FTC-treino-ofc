package org.firstinspires.ftc.teamcode.PIJuncao;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Autonomo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVMaster {
    public OpenCvWebcam webcam;
    public StickObserverPipeline opencv = null;
    private LinearOpMode op;
//    public Autonomo autonomo;
    public CVMaster(LinearOpMode p_op){
        //you can input  a hardwareMap instead of linearOpMode if you want
        op = p_op;
        //initialize webcam
        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
    }

    public double eixoX_juncao, eixoY_juncao;

    public void observeStick(){
        //create the pipeline
        opencv = new StickObserverPipeline();

        eixoX_juncao = opencv.strictLowS;
        eixoY_juncao = opencv.strictHighS;

//        autonomo = new Autonomo(eixoX_juncao,eixoY_juncao);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(opencv);
                //start streaming the camera
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //if you are using dashboard, update dashboard camera view
                /*FtcDashboard.getInstance().startCameraStream(webcam, 5);*/

            }

            @Override
            public void onError(int errorCode){
            }
        });
    }

    //stop streaming
    public void stopCamera(){
        webcam.stopStreaming();
    }
}
