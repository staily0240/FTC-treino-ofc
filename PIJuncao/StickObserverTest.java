package org.firstinspires.ftc.teamcode.PIJuncao;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;

@TeleOp(name = "StickObserverTest")
@Disabled

public class StickObserverTest extends LinearOpMode {
    @Override
    public void runOpMode() {

//        initialize camera and pipeline
        CVMaster cv = new CVMaster(this);
//      call the function to startStreaming

        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {

        }
//        stopStreaming
        cv.stopCamera();
    }
}

//  edges.release