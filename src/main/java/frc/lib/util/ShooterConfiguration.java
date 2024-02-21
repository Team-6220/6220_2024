package frc.lib.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class ShooterConfiguration {
    private Pair<Double, Double> velocities = Pair.of(0.0, 0.0);
    private double armAngle;
    private double headingOffset;
    private static HashMap<Pair<Integer, Integer>, ShooterConfiguration> shooterConfigurations = new HashMap<Pair<Integer, Integer>, ShooterConfiguration>();

    private ShooterConfiguration(Pair<Double, Double> velocities, double armAngle, double headingOffset){
        this.velocities = velocities;
        this.armAngle = armAngle;
        this.headingOffset = headingOffset;
    }

    public Pair<Double, Double> getVelocities(){
        return velocities;
    }

    public double getArmAngle(){
        return armAngle;
    }

    public double getHeadingOffset(){
        return headingOffset;
    }

    public void setVelocities(Pair<Double, Double> velocities){
        this.velocities = velocities;
    }

    public static void setupConfigurations(){
        //FIXME - Add actual values - refer to desmos graph to see corresponding angles
        shooterConfigurations.put(Pair.of(1, 0), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(1, 1), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(1, 2), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(1, 3), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(1, 4), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 0), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 1), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 2), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 3), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 4), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 5), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(2, 6), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 0), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 1), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 2), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 3), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 4), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 5), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 6), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 7), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(3, 8), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 0), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 1), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 2), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 3), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 4), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 5), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 6), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 7), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 8), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 9), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(4, 10), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 0), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 1), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 2), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 3), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 4), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 5), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 6), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 7), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 8), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 9), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 10), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 11), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
        shooterConfigurations.put(Pair.of(5, 12), new ShooterConfiguration(Pair.of(0d,0d), 0d, 0d));
    }

    private static Pair<Double, Double> polarToCartesian(int r, int num){
        double angle = ((8 * Math.PI / 9) * (num / (r + 1) * 2) - (4 * Math.PI / 9));
        return Pair.of(r * Math.cos(angle), r * Math.sin(angle));
    }

    private static Pair<Double, Double> cartesianToPolar(Pose2d pose){
        Pose2d speakerPos = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;
        double yval = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED.getY() - pose.getY() : pose.getY() - VisionConstants.SPEAKER_POSE2D_BLUE.getY();
        double angle = Math.atan2(yval, Math.abs(speakerPos.getX() - pose.getX()));
        double r = Math.hypot(Math.abs(pose.getX() - speakerPos.getX()), Math.abs(pose.getY() - speakerPos.getY()));
        return Pair.of(r, angle);
    }

    private static double columnToAngle(int col, int numcol){
        return ((8 * Math.PI / 9) * (col / (numcol) - (4 * Math.PI / 9)));
    }

    private static List<Pair<Integer, Integer>> getNearestShooterConfigurations(Pose2d robotPose) throws IllegalPositionException{
        ArrayList<Pair<Integer, Integer>> nearestConfigurations = new ArrayList<Pair<Integer, Integer>>();
        Pair<Double, Double> polar = cartesianToPolar(robotPose);
        int innerRow = (int) Math.floor(polar.getFirst());
        int outerRow = (int) Math.ceil(polar.getFirst());
        if(polar.getSecond() <= -4 * Math.PI / 9 || polar.getSecond() >= 4 * Math.PI / 9){
            throw new IllegalPositionException("Robot is too far to the side");
        }
        else if(polar.getFirst() < 1 || polar.getFirst() > 5){
            throw new IllegalPositionException("Robot is too far to the side");
        }

        ArrayList<Pair<Integer, Integer>> points = new ArrayList<Pair<Integer, Integer>>();
        for(int i = 0; i < (innerRow + 1) * 2; i++){
            if(polar.getSecond() >= columnToAngle(i, (innerRow + 1) * 2) && polar.getSecond() <= columnToAngle(i, (innerRow + 1) * 2)){
                nearestConfigurations.add(Pair.of(innerRow, i));
                nearestConfigurations.add(Pair.of(innerRow, i + 1));
                points.add(Pair.of(innerRow, i));
                points.add(Pair.of(innerRow, i + 1));
                break;
            }
        }
        for(int i = 0; i < (outerRow + 1) * 2; i++){
            if(polar.getSecond() >= columnToAngle(i, (outerRow + 1) * 2) && polar.getSecond() <= columnToAngle(i, (outerRow + 1) * 2)){
                nearestConfigurations.add(Pair.of(outerRow, i));
                nearestConfigurations.add(Pair.of(outerRow, i + 1));
                points.add(Pair.of(outerRow, i));
                points.add(Pair.of(outerRow, i + 1));
                break;
            }
        }
        Pair<Integer, Integer> furthest = null;
        double max = 0;
        for (Pair<Integer, Integer> point : points){
            Pair<Double, Double> pointCartesian = polarToCartesian(point.getFirst(), point.getSecond());
            if(Math.hypot(pointCartesian.getFirst() - robotPose.getX(), pointCartesian.getSecond() - robotPose.getY()) > max){
                max = Math.hypot(pointCartesian.getFirst() - robotPose.getX(), pointCartesian.getSecond() - robotPose.getY());
                furthest = point;
            }
        }

        nearestConfigurations.remove(furthest);

        return nearestConfigurations;
    }

    private static double[] get_equation_plane(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){
        double a1 = x2 - x1;
        double b1 = y2 - y1;
        double c1 = z2 - z1;
        double a2 = x3 - x1;
        double b2 = y3 - y1;
        double c2 = z3 - z1;
        double a = b1 * c2 - b2 * c1;
        double b = a2 * c1 - a1 * c2;
        double c = a1 * b2 - b1 * a2;
        double d = (- a * x1 - b * y1 - c * z1);
        return new double[]{a, b, c, d};
    }

    private static double solve_equation_plane(double[] coeffs, double x, double y){
        return (-coeffs[3] - coeffs[0] * x - coeffs[1] * y) / coeffs[2];
    }

    public static ShooterConfiguration getShooterConfiguration(Pose2d robotPose) throws IllegalPositionException{
        List<Pair<Integer, Integer>> configs = getNearestShooterConfigurations(robotPose);
        if(configs.size() == 2){
            Pair<Double, Double> polar = cartesianToPolar(robotPose);
            Pair<Double, Double> vels = Pair.of(
                shooterConfigurations.get(configs.get(0)).getVelocities().getFirst() + (shooterConfigurations.get(configs.get(1)).getVelocities().getFirst() - shooterConfigurations.get(configs.get(0)).getVelocities().getFirst()) * (polar.getFirst() - configs.get(0).getFirst()) / (configs.get(1).getFirst() - configs.get(0).getFirst()),
                shooterConfigurations.get(configs.get(0)).getVelocities().getSecond() + (shooterConfigurations.get(configs.get(1)).getVelocities().getSecond() - shooterConfigurations.get(configs.get(0)).getVelocities().getSecond()) * (polar.getFirst() - configs.get(0).getFirst()) / (configs.get(1).getFirst() - configs.get(0).getFirst())
            );
            double arm = shooterConfigurations.get(configs.get(0)).getArmAngle() + (shooterConfigurations.get(configs.get(1)).getArmAngle() - shooterConfigurations.get(configs.get(0)).getArmAngle()) * (polar.getFirst() - configs.get(0).getFirst()) / (configs.get(1).getFirst() - configs.get(0).getFirst());
            double heading = shooterConfigurations.get(configs.get(0)).getHeadingOffset() + (shooterConfigurations.get(configs.get(1)).getHeadingOffset() - shooterConfigurations.get(configs.get(0)).getHeadingOffset()) * (polar.getFirst() - configs.get(0).getFirst()) / (configs.get(1).getFirst() - configs.get(0).getFirst());
            return new ShooterConfiguration(vels, arm, heading);
        }
        Pose2d speakerPos = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;
        double yval = Constants.isRed ? VisionConstants.SPEAKER_POSE2D_RED.getY() - robotPose.getY() : robotPose.getY() - VisionConstants.SPEAKER_POSE2D_BLUE.getY();
        double xval = Math.abs(robotPose.getX() - speakerPos.getX());
        Pair<Double, Double> a = polarToCartesian(configs.get(0).getFirst(), configs.get(0).getSecond());
        Pair<Double, Double> b = polarToCartesian(configs.get(1).getFirst(), configs.get(1).getSecond());
        Pair<Double, Double> c = polarToCartesian(configs.get(2).getFirst(), configs.get(2).getSecond());
        double[] v1coeffs = get_equation_plane(a.getFirst(), a.getSecond(), shooterConfigurations.get(configs.get(0)).getVelocities().getFirst(), b.getFirst(), b.getSecond(), shooterConfigurations.get(configs.get(1)).getVelocities().getFirst(), c.getFirst(), c.getSecond(), shooterConfigurations.get(configs.get(2)).getVelocities().getFirst());
        double[] v2coeffs = get_equation_plane(a.getFirst(), a.getSecond(), shooterConfigurations.get(configs.get(0)).getVelocities().getSecond(), b.getFirst(), b.getSecond(), shooterConfigurations.get(configs.get(1)).getVelocities().getSecond(), c.getFirst(), c.getSecond(), shooterConfigurations.get(configs.get(2)).getVelocities().getSecond());
        double[] armcoeffs = get_equation_plane(a.getFirst(), a.getSecond(), shooterConfigurations.get(configs.get(0)).getArmAngle(), b.getFirst(), b.getSecond(), shooterConfigurations.get(configs.get(1)).getArmAngle(), c.getFirst(), c.getSecond(), shooterConfigurations.get(configs.get(2)).getArmAngle());
        double[] headingcoeffs = get_equation_plane(a.getFirst(), a.getSecond(), shooterConfigurations.get(configs.get(0)).getHeadingOffset(), b.getFirst(), b.getSecond(), shooterConfigurations.get(configs.get(1)).getHeadingOffset(), c.getFirst(), c.getSecond(), shooterConfigurations.get(configs.get(2)).getHeadingOffset());
        Pair<Double, Double> vels = Pair.of(solve_equation_plane(v1coeffs, xval, yval), solve_equation_plane(v2coeffs, xval, yval));
        double arm = solve_equation_plane(armcoeffs, xval, yval);
        double heading = solve_equation_plane(headingcoeffs, xval, yval);
        return new ShooterConfiguration(vels, arm, heading);
    }
}
