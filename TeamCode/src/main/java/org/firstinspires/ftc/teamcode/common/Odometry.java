package org.firstinspires.ftc.teamcode.common;

public class Odometry {
    private double netTheta;

    private double lo;
    private double ro;

    private final double tl;
    private final double tr;

    public Odometry(double tl, double tr, double netTheta) {
        this.netTheta = netTheta;

        this.tl = tl;
        this.tr = tr;

        lo = 0;
        ro = 0;
    }

    public void update(double l, double r) {
        double deltaL = l - lo;
        double deltaR = r - ro;

        double deltaTheta = (deltaL - deltaR) / (tl + tr);
        netTheta += deltaTheta;

        lo = l;
        ro = r;
    }

    public double getHeading() {
        return netTheta;
    }
}
