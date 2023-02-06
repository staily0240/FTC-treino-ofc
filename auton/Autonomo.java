    /*
     * Copyright (c) 2021 OpenFTC Team
     *
     * Permission is hereby granted, free of charge, to any person obtaining a copy
     * of this software and associated documentation files (the "Software"), to deal
     * in the Software without restriction, including without limitation the rights
     * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
     * copies of the Software, and to permit persons to whom the Software is
     * furnished to do so, subject to the following conditions:
     *
     * The above copyright notice and this permission notice shall be included in all
     * copies or substantial portions of the Software.
     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
     * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
     * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
     * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
     * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
     * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
     * SOFTWARE.
     */

    package org.firstinspires.ftc.teamcode.auton;

    import com.qualcomm.hardware.bosch.BNO055IMU;
    import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.DistanceSensor;
    import com.qualcomm.robotcore.hardware.Servo;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
    import org.firstinspires.ftc.teamcode.PIDGyro.TurnPIDController;
    import org.firstinspires.ftc.teamcode.PIJuncao.ContourPipeline;
    import org.opencv.core.Core;
    import org.opencv.core.Mat;
    import org.opencv.core.MatOfPoint;
    import org.opencv.core.MatOfPoint2f;
    import org.opencv.core.Point;
    import org.opencv.core.Rect;
    import org.opencv.core.Scalar;
    import org.opencv.core.Size;
    import org.opencv.imgproc.Imgproc;
    import org.openftc.apriltag.AprilTagDetection;
    import org.openftc.easyopencv.OpenCvCamera;
    import org.openftc.easyopencv.OpenCvCameraFactory;
    import org.openftc.easyopencv.OpenCvCameraRotation;

    import java.util.ArrayList;
    import java.util.List;

    @TeleOp
    public class Autonomo extends LinearOpMode
    {

        public DcMotor  RMF;    //Porta 0 - Motor Mecanum - Direita Frontal
        public DcMotor  RMB;    //Porta 1 - Motor Mecanum - Direita Traseiro
        public DcMotor  LMF;    //Porta 2 - Motor Mecanum - Esquerda Frontal
        public DcMotor  LMB;    //Porta 3 - Motor Mecanum - Esquerda Traseiro

        public DcMotor MAT;     //Porta 0 - Motor Core Hex - Antebraço
        public DcMotor MSE;     //Porta 2 - Motor UltraPlanetary - Sistema de Elevação
        public DcMotor MRP;     //Porta 3 - Motor Core Hex - Rotação Prato

        public Servo servoGarra;    //Porta 0 - Direita
        public Servo servoGarra2;   //Porta 1 - Esquerda

        BNO055IMU imu;
        //    public DigitalChannel touch_MRP;
        public DistanceSensor distanceSensorF;
        public DistanceSensor distanceSensorB;

    //    CVMaster cv = new CVMaster(this);

    //======================================== Tag =====================================================

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 1, 2, 3 from the 36h11 family
        int ID_TAG_OF_INTEREST = 18;
        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        int id = 0;

        AprilTagDetection tagOfInterest = null;

    //====================================== Junção ====================================================

        private OpenCvCamera webcam;

        private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
        private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

        private double CrLowerUpdate = 160;
        private double CbLowerUpdate = 100;
        private double CrUpperUpdate = 255;
        private double CbUpperUpdate = 255;

        public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
        public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
        public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
        public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

        private double lowerruntime = 0;
        private double upperruntime = 0;

        // Yellow Range
        public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
        public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    //======================================= Robô =====================================================

        double test = 0;
        double currentPosition_MSE, controleEncoder_MSE, targetPosition_MSE = 1823,  controleForce_MSE, max_MSE = -6100;
        double proporcional_MSE, erro_MSE, kP_MSE = 0.0008;


        double PID_loc;
        double proporcional_loc, integral_loc, derivada_loc;
        double kP_loc = 0.02, kI_loc = 0, kD_loc = 0.002;

        double erro_loc, ultErro_loc;

        double distanceF, distanceB;

        double currentPosition_ATB, targetPosition_ATB = 1950, controleEncoder_ATB;

        double PID_ATB;
        double proporcional_ATB, integral_ATB, derivada_ATB;
        double kP_ATB = 0.0008, kI_ATB = 0.00085, kD_ATB = 0;
        double erro_ATB;

        static final double MAX_POS_D     =  0.2;     // Maximum rotational position 0.5
        static final double MIN_POS_D     =  0.0;     // Minimum rotational position    certo

        static final double MAX_POS_E     =  0.1;     // Maximum rotational position
        static final double MIN_POS_E     =  0.4;     // Minimum rotational position 0.5

        double  position_D = MIN_POS_D; // Start at halfway position
        double  position_E = MIN_POS_E; // Start at halfway position

        public double eixoX_juncao, eixoY_juncao;

        double getAbsoluteAngleVariable, firstAngRotation;
        boolean boolCoroar = false , boolAlinhamento = true;
        double ultDist = 0;

        @Override
        public void runOpMode()
        {

            initHardware();

    //        JuncaoInit();

    //        idQRCode();

    //        telemetry.addData("Eixo X:\t", cv.eixoX_juncao);
    //        telemetry.addData("Eixo Y:\t", cv.eixoY_juncao);
    //        telemetry.update();
            controleEncoder_MSE = MSE.getCurrentPosition();
            controleEncoder_ATB = MAT.getCurrentPosition();

            waitForStart();


            while(test < 50){
                getAbsoluteAngleVariable = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //            getAbsoluteAngleVariable = (getAbsoluteAngleVariable < 0) ? (360 - getAbsoluteAngleVariable) : getAbsoluteAngleVariable;
                test ++;
            }

            statusGarra(false);

            ajusteMSE(false);

            runToStation();

            statusGarra(true);

            setAllPower(0.2);
            sleep(500);

            turnPID(90);
    //
            setAllPower(0);
    //
            setPID_ATB();
    //
            ajusteMSE(true);
    //
            runToJuncao();

            searchingJunction();


            statusGarra(false);
    //
            sleep(500);
        }

        private void test() {
            while(1>0){
                double getAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                telemetry.addData("ang:\t", getAngle);
                telemetry.addData("Angulo do robo:\t", getAbsoluteAngle());
                telemetry.addData("Angulo inicial:\t", getAbsoluteAngleVariable);
                telemetry.addData("First do robo:\t", firstAngRotation);
                telemetry.update();
            }
        }

        //Atalhos

        public void ajusteMSE(boolean mode) {

            if(!mode){

                do {

                    distanceF = distanceSensorF.getDistance(DistanceUnit.CM);
                    MSE.setPower(0.2);

                }while (distanceF < 30);

            }else{

                do {
                    currentPosition_MSE = MSE.getCurrentPosition();
                    MSE.setPower(0.4);
                    telemetry.addData("Encoder:\t", currentPosition_MSE);
                    telemetry.update();
                }while (Math.abs(currentPosition_MSE) < Math.abs(max_MSE));

            }

            MSE.setPower(0);

        }

        public void runToStation(){

            do{

                distanceF = distanceSensorF.getDistance(DistanceUnit.CM);

                erro_loc = -(11.5 - distanceF);

                proporcional_loc = erro_loc * kP_loc;

                if(distanceF > 30){
                    proporcional_loc /= 2;
                }

                setAllPower(proporcional_loc);

                telemetry.addData("Proporcional:\t",proporcional_loc);
                telemetry.addData("Distancia Frente:\t",distanceF);
                telemetry.addData("Erro_Loc:\t",erro_loc);
                telemetry.update();

            }while (erro_loc > 0);

            erro_loc = 0;
        }

        public void runToJuncao() {

            setAllPower(-0.2);
            sleep(500);

            final double kP_loc = 0.015;

            do {

                distanceB = distanceSensorB.getDistance(DistanceUnit.CM);

                erro_loc = (60.0 - distanceB);

                proporcional_loc = erro_loc * kP_loc;

                setAllPower(proporcional_loc);

            }while (erro_loc > 0);

        }

        public void statusGarra(boolean status){

            if(status) {
                position_D = MAX_POS_D; // Start at halfway position
                position_E = MAX_POS_E; // Start at halfway position
            }else{
                position_D = MIN_POS_D; // Start at halfway position
                position_E = MIN_POS_E; // Start at halfway position
            }

            servoGarra.setPosition(position_D);              //Direita
            servoGarra2.setPosition(position_E);             //Esquerda
        }

        public void setPID_ATB(){
            double erro, ultPosition = 100;
            do {

                currentPosition_ATB = Math.abs(MAT.getCurrentPosition()) -  Math.abs(controleEncoder_ATB);
                erro = currentPosition_ATB - ultPosition;
                ultPosition = currentPosition_ATB;

                erro_ATB = targetPosition_ATB - currentPosition_ATB;
                proporcional_ATB = erro_ATB * kP_ATB;

                MAT.setPower(proporcional_ATB);
                telemetry.addData("Posicao:\t", currentPosition_ATB);
                telemetry.addData("Erro:\t", erro_ATB);

                telemetry.addData("Erro2:\t", erro);
                telemetry.update();

            }while (Math.abs(erro_ATB) > 150 && Math.abs(erro) != 0);

            MAT.setPower(0.1);
        }

        public void searchingJunction() {
            double motorPower, erro;
            double timer, timer2;

            int contador = 0, alinhamento = 0;
    //        boolean boolCont = false;

    //        resetRuntime();
            timer = getRuntime(); // 1000
            do {

                distanceF = distanceSensorF.getDistance(DistanceUnit.CM);

                if(distanceF < 40 ){
                    contador++;
                }

                motorPower = 0.4;
                motorPower = (contador != 0) ? 0.2 : motorPower;
                motorPower = (contador%2 != 0) ? -motorPower : motorPower;

                turnToPID(firstAngRotation);
                setMotorPowerIndiv(-motorPower, motorPower, motorPower, -motorPower);

                telemetry.addData("Contador:\t", contador);
                telemetry.addData("Erro:\t", Math.abs(firstAngRotation - getAbsoluteAngle()));
                telemetry.addData("Distancia Frente:\t",distanceF);
                telemetry.addData("First Ang Rotation:\t",firstAngRotation);
                telemetry.update();

                if(boolAlinhamento){
                    ultDist = distanceF;
                    boolAlinhamento = false;
                }
    //            if(-(8 - distanceF) < 1){
    //                boolCont = false;
    //            }
                if(contador >= 10){
                    coroar();
                }

            }while (boolCoroar == false);

        }

        public void coroar() {

            do{

                boolCoroar = false;
                distanceF = distanceSensorF.getDistance(DistanceUnit.CM);

                erro_loc = -(8 - distanceF);

    //            proporcional_loc = erro_loc * kP_loc;
    //
    //            if(distanceF > 30){
    //                proporcional_loc /= 2;
    //            }

                setAllPower(0.2);

                telemetry.addData("Proporcional:\t",proporcional_loc);
                telemetry.addData("Distancia Frente:\t",distanceF);
                telemetry.addData("Erro_Loc:\t",erro_loc);
                telemetry.update();

                if(Math.abs(distanceF) > 40){  //
                    if(erro_loc > 3){searchingJunction();}
                }
    
            }while (erro_loc > 3);

            boolCoroar = true;
            setAllPower(0);
        }

    //    Gyro
        public double getAbsoluteAngle() {
            double getAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            return getAngle;
        }

        public void turnPID(double degrees) {
            double targetAngle = degrees + getAbsoluteAngleVariable;

            firstAngRotation = targetAngle;

            turnToPID(targetAngle);
        }

        void turnToPID(double targetAngle) {
            TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
            telemetry.setMsTransmissionInterval(50);
            // Checking lastSlope to make sure that it's not oscillating when it quits
            while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {

                double motorPower = pid.update(getAbsoluteAngle());
                setMotorPowerIndiv(motorPower, motorPower, -motorPower, -motorPower);

                telemetry.addData("Current Angle", getAbsoluteAngle());
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Slope", pid.getLastSlope());
                telemetry.addData("Power", motorPower);
                telemetry.update();

                if(motorPower < 0.006) break;

            }
            setAllPower(0);
        }
        //Hardware Pushbot

        public void initHardware() {

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
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

    //        touch_MRP.setMode(DigitalChannel.Mode.INPUT);
    //        touch_ATB.setMode(DigitalChannel.Mode.INPUT);

        }

        public void setInitMotors(){//  INNICIALIZAÇÃO DOS MOTORES E SERVOS
            LMF = hardwareMap.get(DcMotor.class, "LMF");
            LMB = hardwareMap.get(DcMotor.class, "LMB");
            RMF = hardwareMap.get(DcMotor.class, "RMF");
            RMB = hardwareMap.get(DcMotor.class, "RMB");

            MAT = hardwareMap.get(DcMotor.class, "MAT");
            MSE = hardwareMap.get(DcMotor.class, "MSE");
            MRP = hardwareMap.get(DcMotor.class, "MRP");

            servoGarra = hardwareMap.get(Servo.class, "servoGarra");
            servoGarra2 = hardwareMap.get(Servo.class, "servoGarra2");
        }

        public void setInitSensor(){
            distanceSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");
            distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
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

        public void setAllPower(double p){     //Metodo para definir a mesma força para todos os motores
            setMotorPowerIndiv(p,p,p,p);
        }

    //    public void giroEixo(double p){
    //
    //        if(p >= 0){
    //            setMotorPowerIndiv(-p,-p,p,p);
    //        }else{
    //            setMotorPowerIndiv(p,p,-p,-p);
    //        }
    //
    //    }

        public void setMotorPowerIndiv(double rF, double rB, double lF,  double lB){
            RMF.setPower(rF);
            RMB.setPower(rB);
            LMF.setPower(lF);
            LMB.setPower(lB);
        }

    //Processamento de Imagem:

    //-  Cone:
        public void idQRCode(){

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

            telemetry.setMsTransmissionInterval(50);

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested())
            {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0)
                {
                    boolean tagFound = false;

                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if(tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
    //            sleep(20);
            }

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if (tagOfInterest == null || tagOfInterest.id == LEFT){
                //trajetoria
            }else if (tagOfInterest.id == MIDDLE){
                //trajetoria
            }else{
                //trajetoria
            }


        }

        void tagToTelemetry(AprilTagDetection detection)
        {
            id = detection.id;
    //        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    //        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
    //        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
    //        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    //        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
    //        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    //        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }

        public void JuncaoInit(){
            // OpenCV webcam
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            //OpenCV Pipeline
            ContourPipeline myPipeline;
            webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
            // Configuration of Pipeline
            myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
            myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
            // Webcam Streaming
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                }
            });


            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);

            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            testing(myPipeline);

            telemetry.update();


        }

        public void testing(ContourPipeline myPipeline){
            if(lowerruntime + 0.05 < getRuntime()){
                CrLowerUpdate += -gamepad1.left_stick_y;
                CbLowerUpdate += gamepad1.left_stick_x;
                lowerruntime = getRuntime();
            }
            if(upperruntime + 0.05 < getRuntime()){
                CrUpperUpdate += -gamepad1.right_stick_y;
                CbUpperUpdate += gamepad1.right_stick_x;
                upperruntime = getRuntime();
            }

            CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
            CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
            CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
            CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

            myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
            myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

    //        telemetry.addData("Eixo_x ", myPipeline.getRectMidpointXY().x);
    //        telemetry.addData("Eixo_y ", myPipeline.getRectMidpointXY().y);

            eixoX_juncao = myPipeline.getRectMidpointXY().x;
            eixoY_juncao = myPipeline.getRectMidpointXY().y;
        }

        public Double inValues(double value, double min, double max){
            if(value < min){ value = min; }
            if(value > max){ value = max; }
            return value;
        }

    }