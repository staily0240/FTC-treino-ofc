/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SemiOficial;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class  HardwarePushbot {  // Classe de definição de motores e servos
    /* local OpMode members. */
    HardwareMap hwMap           =  null;

//==============================CRIAÇÃO DOS MOTORES E SERVOS========================================

//  Motores de Teste:

    public DcMotor Test1; //Porta 1

//  Motores de locomoção:

    public DcMotor  RMF;    //Porta 0 - Motor Mecanum - Direita Frontal
    public DcMotor  RMB;    //Porta 1 - Motor Mecanum - Direita Traseiro
    public DcMotor  LMF;    //Porta 2 - Motor Mecanum - Esquerda Frontal
    public DcMotor  LMB;    //Porta 3 - Motor Mecanum - Esquerda Traseiro

//  Motores de sistema:

    public DcMotor MAT;     //Porta 0 - Motor Core Hex - Antebraço
    public DcMotor MSE;     //Porta 2 - Motor UltraPlanetary - Sistema de Elevação
    public DcMotor MRP;     //Porta 3 - Motor Core Hex - Rotação Prato

//  Servos:

    public Servo servoGarra;    //Porta 0 - Direita
    public Servo servoGarra2;   //Porta 1 - Esquerda

//  Sensor:

    public BNO055IMU imu;
    //    public DigitalChannel touch_MRP;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        setInitMotors();

        setInitSensor();

        setDiretion();

        setStatus();

        setEncoders();

//        resetEncoder(-1);

        setAllPower(0);     //Definindo força 0 para todos os motores

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

//        touch_MRP.setMode(DigitalChannel.Mode.INPUT);
//        touch_ATB.setMode(DigitalChannel.Mode.INPUT);

    }

    public void setInitMotors(){//  INNICIALIZAÇÃO DOS MOTORES E SERVOS
        LMF = hwMap.get(DcMotor.class, "LMF");
        LMB = hwMap.get(DcMotor.class, "LMB");
        RMF = hwMap.get(DcMotor.class, "RMF");
        RMB = hwMap.get(DcMotor.class, "RMB");

        MAT = hwMap.get(DcMotor.class, "MAT");
        MSE = hwMap.get(DcMotor.class, "MSE");
        MRP = hwMap.get(DcMotor.class, "MRP");

        servoGarra = hwMap.get(Servo.class, "servoGarra");
        servoGarra2 = hwMap.get(Servo.class, "servoGarra2");
    }

    public void setInitSensor(){
//        distance = hwMap.get(DistanceSensor.class, "Distance");
//        touch_MRP = hardwareMap.get(DigitalChannel.class,"touch_MRP");
//        touch_ATB = hardwareMap.get(DigitalChannel.class, "touch_ATB");

    }

    public void setDiretion(){//    DIREÇÃO DOS MOTORES E SERVOS
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);      //REVERSE - Direção Reversa
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);      //FORWARD - Direção Normal
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        MAT.setDirection(DcMotorSimple.Direction.FORWARD);
        MSE.setDirection(DcMotorSimple.Direction.REVERSE);
        MRP.setDirection(DcMotorSimple.Direction.FORWARD);

        servoGarra.setDirection(Servo.Direction.FORWARD);
        servoGarra2.setDirection(Servo.Direction.FORWARD);
    }

    public void setStatus(){//      STATUS DOS MOTORES E SERVOS
        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        MAT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MSE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MRP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setEncoders(){//    STATUS DOS ENCODERS
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//     INUTILIZANDO
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        ATB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//       UTILIZANDO
////        PRT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //    Set power to all motors
    public void setAllPower(double p){     //Metodo para definir a mesma força para todos os motores
        setMotorPowerIndiv(p,p,p,p,p,p,p);
    }

    public void setMotorPowerIndiv(double rF, double rB, double lF,  double lB, double atb, double lin, double prt){
        RMF.setPower(rF);
        RMB.setPower(rB);
        LMF.setPower(lF);
        LMB.setPower(lB);
    }

}