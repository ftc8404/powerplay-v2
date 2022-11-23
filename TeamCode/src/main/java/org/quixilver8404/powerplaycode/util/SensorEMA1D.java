package org.quixilver8404.powerplaycode.util;

public class SensorEMA1D {

    protected double sigma_hat_squared;
    protected double sigma_desired_squared;
    protected double ema_sigma_n;
    protected double delta_t_hat;
    protected double ema_delta_t_n;
    protected double last_pos;

    public SensorEMA1D(final double startPos, final double sigma_desired, final double n_sigma, final double n_dt, final double expected_dt) {
        sigma_hat_squared = 0;
        sigma_desired_squared = sigma_desired*sigma_desired;
        ema_sigma_n = n_sigma;
        delta_t_hat = expected_dt; // Start it off at something vaguely reasonable that's not 0
        ema_delta_t_n = n_dt;
        last_pos = startPos;
    }

    public final double updateAndCalcTau(final double dPos, final double dt) {

        sigma_hat_squared = sigma_hat_squared * (ema_sigma_n - 1) / ema_sigma_n + dPos * dPos / ema_sigma_n;
        delta_t_hat = delta_t_hat * (ema_delta_t_n - 1) / (ema_delta_t_n) + dt / (ema_delta_t_n);

        final double tau = delta_t_hat * sigma_hat_squared / sigma_desired_squared;

        return tau;
    }
}
