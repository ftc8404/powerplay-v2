package org.quixilver8404.powerplaycode.util;

public class SensorEMA3D {

    protected Vector3 sigma_hat_squared;
    protected Vector3 sigma_desired_squared;
    protected double ema_sigma_n;
    protected double delta_t_hat;
    protected double ema_delta_t_n;
    protected Vector3 last_pos;

    public SensorEMA3D(final Vector3 startPos, final Vector3 sigma_desired, final double n_sigma, final double n_dt, final double expected_dt) {
        sigma_hat_squared = new Vector3();
        sigma_desired_squared = new Vector3(sigma_desired.x()*sigma_desired.x(), sigma_desired.y()*sigma_desired.y(), sigma_desired.theta()*sigma_desired.theta());
        ema_sigma_n = n_sigma;
        delta_t_hat = expected_dt; // Start it off at something vaguely reasonable that's not 0
        ema_delta_t_n = n_dt;
        last_pos = startPos;
    }

    public final Vector3 updateAndCalcTau(final Vector3 dPos, final double dt) {
//        System.out.println("sigma_hat_squared before: " + sigma_desired_squared.toString());
        sigma_hat_squared = new Vector3(
                sigma_hat_squared.x() * (ema_sigma_n - 1) / ema_sigma_n + dPos.x() * dPos.x() / ema_sigma_n,
                sigma_hat_squared.y() * (ema_sigma_n - 1) / ema_sigma_n + dPos.y() * dPos.y() / ema_sigma_n,
                sigma_hat_squared.theta() * (ema_sigma_n - 1) / ema_sigma_n + dPos.theta() * dPos.theta() / ema_sigma_n
        );
//        System.out.println("sigma_hat_squared after: " + sigma_desired_squared.toString());
//        System.out.println("dt_hat before: " + delta_t_hat);
        delta_t_hat = delta_t_hat * (ema_delta_t_n - 1) / (ema_delta_t_n) + dt / (ema_delta_t_n);
//        System.out.println("dt_hat after: " + delta_t_hat);

        final double tau_x = delta_t_hat * sigma_hat_squared.x() / sigma_desired_squared.x();
        final double tau_y = delta_t_hat * sigma_hat_squared.y() / sigma_desired_squared.y();
        final double tau_alpha = delta_t_hat * sigma_hat_squared.theta() / sigma_desired_squared.theta();

        final Vector3 tau_vec = new Vector3(tau_x, tau_y, tau_alpha);

        return tau_vec;
    }
}
