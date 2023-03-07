package frc.robot.Constants;

import org.opencv.core.Mat.Tuple2;

public class SwerveConstants {
    public static final int MODULE_COUNT = 4;

    public static final double xOffsetFrontRight = 0.381; //example value
    public static final double yOffsetFrontRight = 0.381; //example value

    public static final double xOffsetFrontLeft = -0.381; //example value
    public static final double yOffsetFrontLeft = 0.381; //example value


    public static final Tuple2 tmp = new Tuple2<Double>(xOffsetFrontRight, yOffsetFrontRight);
    public static final Tuple2 tmp2 = new Tuple2<Double>(xOffsetFrontLeft, yOffsetFrontLeft);

    
}