#include "Vehicle.h"

enum
{
    VEHICLE_UPDATE_HZ = 50
};

static double clamp(double value, double min_value, double max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

static double abs_double(double value)
{
    return value < 0.0 ? -value : value;
}

static double sign_double(double value)
{
    if (value > 0.0)
        return 1.0;

    if (value < 0.0)
        return -1.0;

    return 0.0;
}

void Vehicle_init(Vehicle* vehicle, const VehicleParams* params)
{
    vehicle->params = *params;
    vehicle->commanded_torque_nm = 0.0;
    vehicle->speed_m_per_s = 0.0;
    vehicle->distance_m = 0.0;
}

void Vehicle_set_torque(Vehicle* vehicle, double torque_nm)
{
    const double max_torque = vehicle->params.motor_max_torque_nm;
    vehicle->commanded_torque_nm = clamp(torque_nm, -max_torque, max_torque);
}

void Vehicle_step_50hz(Vehicle* vehicle)
{
    const double dt = 1.0 / (double) VEHICLE_UPDATE_HZ;
    const double air_density_kg_per_m3 = 1.225;
    const double drag_coefficient = 1.0;
    const double gravity_m_per_s2 = 9.81;
    const double min_speed_epsilon = 1e-6;

    const VehicleParams* params = &vehicle->params;
    const double old_speed = vehicle->speed_m_per_s;
    const double wheel_angular_speed = params->wheel_radius_m > 0.0 ? old_speed / params->wheel_radius_m : 0.0;
    const double motor_angular_speed = wheel_angular_speed * params->gear_ratio;

    double motor_torque = vehicle->commanded_torque_nm;
    if (abs_double(motor_angular_speed) >= params->motor_max_speed_rad_per_sec)
        motor_torque = 0.0;

    const double wheel_torque = motor_torque * params->gear_ratio;
    const double tractive_force = params->wheel_radius_m > 0.0 ? wheel_torque / params->wheel_radius_m : 0.0;

    const double rolling_force_magnitude =
        params->rolling_resistance_coefficient * params->mass_kg * gravity_m_per_s2;
    const double drag_force = 0.5 * air_density_kg_per_m3 * drag_coefficient * params->frontal_area_m2 * old_speed * abs_double(old_speed);

    double opposing_force = drag_force;
    if (abs_double(old_speed) > min_speed_epsilon)
        opposing_force += rolling_force_magnitude * sign_double(old_speed);
    else if (abs_double(tractive_force) <= rolling_force_magnitude)
        opposing_force = tractive_force;
    else
        opposing_force += rolling_force_magnitude * sign_double(tractive_force);

    const double net_force = tractive_force - opposing_force;
    const double acceleration = params->mass_kg > 0.0 ? net_force / params->mass_kg : 0.0;

    double new_speed = old_speed + acceleration * dt;
    if (old_speed > 0.0 && new_speed < 0.0)
        new_speed = 0.0;
    else if (old_speed < 0.0 && new_speed > 0.0)
        new_speed = 0.0;

    vehicle->speed_m_per_s = new_speed;
    vehicle->distance_m += 0.5 * (old_speed + new_speed) * dt;
}

double Vehicle_get_speed(const Vehicle* vehicle)
{
    return vehicle->speed_m_per_s;
}
