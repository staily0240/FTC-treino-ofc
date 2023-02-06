//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.lang.*;
//
//@Disabled
////@TeleOp
//public class MainCode extends LinearOpMode {
//
//    HardwarePushbot   robot       = new HardwarePushbot();
//    Locomocao   locomocao;
//    Sistemas   sistemas;
//
//    double speedLoc, speedLin;
//
//    double targetPosition_PRT;
//    double currentPosition_PRT;
//
//    double targetPosition_ATB;
//    double currentPosition_ATB;
//
//    double angle;
//
//    boolean touch_PRT;
//    boolean touch_ATB;
//
//    double powersLoc[], powersSist[];
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
////        robot.resetEncoder(-1);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            angle = robot.getAngle();
//
////            touch_ATB = robot.getTouch(0);
//            touch_PRT = robot.getTouch(3);
//
//            currentPosition_ATB = robot.ATB.getCurrentPosition();
//            currentPosition_PRT = robot.PRT.getCurrentPosition();
//
//            Locomocao();
//
////            Sistemas();
//
//            //  Print's
//
////            Prints();
//
//            telemetry.update();
//
//        }
//    }
//
//    public void Locomocao(){
//
//        speedLoc = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ? 0.25 : speedLoc;
//
//        locomocao = new Locomocao(
//                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, angle, speedLoc, gamepad1.dpad_down
//        );
//
//        // Set Power - Locomoção
//        powersLoc = locomocao.runlocomocao();
//
//        robot.RMF.setPower(powersLoc[0]);
//        robot.RMB.setPower(powersLoc[1]);
//        robot.LMF.setPower(powersLoc[2]);
//        robot.LMB.setPower(powersLoc[3]);
//
//    }
//
//    public void Sistemas(){
//
//        speedLin =
//            gamepad2.b ? 1 :
//            gamepad2.a ? 0.5 :
//            gamepad2.x ? 0.25 :
//            speedLin;
///*
// && !gamepad2.dpad_left && !gamepad2.dpad_right
//  && !gamepad2.dpad_up && !gamepad2.dpad_down
//            (gamepad2.dpad_right && gamepad2.dpad_down) ? -1000:
//            (gamepad2.dpad_right && gamepad2.dpad_up) ? -3000:
//            && !gamepad2.dpad_up && gamepad2.dpad_down
//
//            (gamepad2.dpad_left && gamepad2.dpad_down) ? 1000:
//            (gamepad2.dpad_left && gamepad2.dpad_up) ? 3000:
//             || (targetPosition_PRT == 0)
//
// */
//        targetPosition_PRT =
//            (gamepad2.dpad_down) ? 0 :
//            (gamepad2.dpad_right) ? -2000 :
//            (gamepad2.dpad_left) ? 2000 :
//            targetPosition_PRT;
//
//        targetPosition_PRT =
//            (((targetPosition_PRT > 0) && (gamepad2.dpad_up))) ? 4000 :
//            ((targetPosition_PRT < 0) && (gamepad2.dpad_up)) ? -4000 :
//                    targetPosition_PRT;
//
//        if(touch_PRT){robot.resetEncoder(3);}
//
//        sistemas = new Sistemas(
//                -gamepad2.left_stick_y, -gamepad2.right_stick_y,
//                speedLin,
//                currentPosition_PRT, targetPosition_PRT,
//                currentPosition_ATB, targetPosition_ATB
//        );
//
//        // Set Power - Sistemas
//
//        powersSist = sistemas.runsistemas();
//
//        robot.ATB.setPower(gamepad2.left_stick_y);
//        robot.LIN.setPower(gamepad2.left_stick_y);
//        robot.PRT.setPower(gamepad2.left_stick_y);
//
//        robot.Test1.setPower(gamepad2.left_stick_y);    // Porta TESTE
//    }
//
//    public void Prints(){
//
//        telemetry.addLine("Motor:");//                                     Force
//        telemetry.addData("Antebraço:\t", powersSist[0]);
//        telemetry.addData("Linear:\t", powersSist[1]);
//        telemetry.addData("Prato:\t", powersSist[2]);
//
//        telemetry.addLine("\n-------------------------------------------------\n");
//
//        telemetry.addLine("Sensor:");//                                    Sensor's
//        telemetry.addData("Touch:\t", touch_PRT);
//
//        telemetry.addLine("\n-------------------------------------------------\n");
//
//        telemetry.addLine("Encoder - Antebraço:");//                                    Encoder's
//        telemetry.addData("Atual:\t", currentPosition_ATB);
//        telemetry.addData("Alvo:\t", targetPosition_ATB);
//        telemetry.addData("Erro:\t", powersSist[4]);
//
//        telemetry.addData("\nPID:\t", powersSist[3]);
//        telemetry.addData("Proporcional:\t", powersSist[5]);
//        telemetry.addData("Integral:\t", powersSist[6]);
//        telemetry.addData("Derivada:\t", powersSist[7]);
//
////        telemetry.addData("Atual:\t", currentPosition_PRT);
////        telemetry.addData("Alvo:\t", targetPosition_PRT);
////        telemetry.addData("Erro:\t", powersSist[4]);
////
////        telemetry.addData("\nPID:\t", powersSist[3]);
////        telemetry.addData("Proporcional:\t", powersSist[5]);
////        telemetry.addData("Integral:\t", powersSist[6]);
////        telemetry.addData("Derivada:\t", powersSist[7]);
//
//    }
//
//}
