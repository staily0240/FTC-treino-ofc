package org.firstinspires.ftc.teamcode.PI;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class JunctionDetection_pt2 extends OpenCvPipeline {
    /*
    AMARELO = Estacionamento Esquerdo
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Ponto de ancoragem TOPLEFT para a caixa delimitadora
    private static Point TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Largura e altura da caixa delimitadora
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // definições de cores
    private final Scalar
            YELLOW = new Scalar(255, 255, 0);

    // definições do ponto de ancoragem
    Point junction_pointA = new Point(
            TOPLEFT_ANCHOR_POINT.x,
            TOPLEFT_ANCHOR_POINT.y);
    Point junction_pointB = new Point(
            TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Variável de execução que armazena a posição de estacionamento
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Obtém o submat frame e então soma todos os valores
        Mat areaMat = input.submat(new Rect(junction_pointA, junction_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        // Obtém o valor RGB mínimo de cada canal
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        // Altere a cor da caixa delimitadora com base na cor da capa
//
//              position = ParkingPosition.CENTER;
////                 Imgproc.rectangle(
////                         input,
////                         sleeve_pointA,
////                         sleeve_pointB,
////                         CYAN,
////                         2
////                 );
////             } else if (sumColors.val[1] == minColor) {
////                 position = ParkingPosition.RIGHT;
////                 Imgproc.rectangle(
////                         input,
////                         sleeve_pointA,
////                         sleeve_pointB,
////                         MAGENTA,
////                         2
////                 );
////             } else {
////                 position = ParkingPosition.LEFT;
////                 Imgproc.rectangle(
////                         input,
////                         sleeve_pointA,
////                         sleeve_pointB,
////                         YELLOW,
////                         2
////                 );
////             }
//
//            position = ParkingPosition.LEFT;
//            Imgproc.rectangle(
//                input,
//                sleeve_pointA,
//                sleeve_pointB,
//                YELLOW,
//                2
//            );
             // Libera e retorna a entrada
             areaMat.release();
             return input;
         }

        // Retorna um enum sendo a posição atual onde o robô irá estacionar
        public ParkingPosition getPosition() {
            return position;
        }
    }