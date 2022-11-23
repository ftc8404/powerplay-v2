package org.quixilver8404.powerplaycode.util;



public class OdometryEMA {
    Vector3 m_hat;
    final Vector3 R;
    Vector3 W;
    final double W_n;

    public OdometryEMA(final Vector3 R, final double W_n) {
        m_hat = new Vector3(1, 1, 1);
        this.R = R;
        W = new Vector3(1,1,1);
        this.W_n = W_n;
    }

    public void update(final Vector3 d_abs, final Vector3 d_odo) {

//        System.out.println("Inside Odometry ema");
//        System.out.println("m_hat: " + m_hat.toString());
//        System.out.println("d_abs: " + d_abs.toString());
//        System.out.println("d_odo: " + d_odo.toString());

        if (Math.abs(d_odo.x()) >= 1e-2) {
            final double w_x = Math.sqrt(R.x()*R.x() - d_odo.x()*d_odo.x()) - R.x();
            final double m_x = d_abs.x()/d_odo.x();
//            System.out.println("m_x: " + m_x);
//            System.out.println("w_x: " + w_x);
            W.setX(W.x()*(W_n-1)/W_n + w_x*30/W_n);
            m_hat.setX(
                    (1 - w_x/W.x())*m_hat.x() + (w_x/W.x())*m_x
            );
        }

        if (Math.abs(d_odo.y()) >= 1e-2) {
            final double w_y = Math.sqrt(R.y()*R.y() - d_odo.y()*d_odo.y()) - R.y();
            final double m_y = d_abs.y()/d_odo.y();
//            System.out.println("m_y: " + m_y);
//            System.out.println("w_y: " + w_y);
            W.setY(W.y()*(W_n-1)/W_n + w_y*30/W_n);
            m_hat.setY(
                    (1 - w_y/W.y())*m_hat.y() + (w_y/W.y())*m_y
            );
        }

        if (Math.abs(d_odo.theta()) >= 1e-2) {
            final double w_theta = Math.sqrt(R.theta()*R.theta() - d_odo.theta()*d_odo.theta()) - R.theta();
            final double m_theta = d_abs.theta()/d_odo.theta();
//            System.out.println("m_theta: " + m_theta);
//            System.out.println("w_theta: " + w_theta);
            W.setTheta(W.theta()*(W_n-1)/W_n + w_theta*30/W_n);
            m_hat.setTheta(
                    (1 - w_theta/W.theta())*m_hat.theta() + (w_theta/W.theta())*m_theta
            );
        }


//        final Vector3 w = Vector3.SubtractVector(Vector3.AddVector(R.square(), d_odo.square()).sqrt(), R).div(d_odo.abs());
//        final Vector3 m = d_abs.div(d_odo);
//
//        W = W.scalarMultiply((W_n-1)/W_n).addVector(w.scalarMultiply(30 * 1/W_n));
//
//        if (Double.isFinite(m.x()) && Double.isFinite(w.x()) && W.x() != 0) { // For when odometry registers a movement of 0. That's makes our equations numerically unstable, and so we discard the measurement
//            m_hat.setX(
//                    (1 - w.x()/W.x())*m_hat.x() + (w.x()/W.x())*m.x()
//            );
//        }
//        if (Double.isFinite(m.y()) && Double.isFinite(w.y()) && W.y() != 0) {
//            m_hat.setY(
//                    (1 - w.y()/W.y())*m_hat.y() + w.y()*m.y()
//            );
//        }
//        if (Double.isFinite(m.theta()) && Double.isFinite(w.theta()) && W.theta() != 0) {
//            m_hat.setTheta(
//                    (1 - w.theta()/W.theta())*m_hat.theta() + w.theta()*m.theta()
//            );
//        }
//        System.out.println("W: " + W.toString() + ", w: " + w.toString());
//        System.out.println("m_hat: " + m_hat.toString() + ", m: " + m.toString());
    }

    public Vector3 getMhat() {
        return m_hat;
    }
}
