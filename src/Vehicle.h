#ifndef VEHICLE_H
#define VEHICLE_H

typedef struct
{
    double mass_kg;
    double frontal_area_m2;
    double rolling_resistance_coefficient;
    double motor_max_speed_rad_per_sec;
    double motor_max_torque_nm;
    double gear_ratio;
    double wheel_radius_m;
} VehicleParams;

typedef struct
{
    VehicleParams params;
    double commanded_torque_nm;
    double speed_m_per_s;
    double distance_m;
} Vehicle;

void Vehicle_init(Vehicle* vehicle, const VehicleParams* params);
void Vehicle_set_torque(Vehicle* vehicle, double torque_nm);
void Vehicle_step_50hz(Vehicle* vehicle);
double Vehicle_get_speed(const Vehicle* vehicle);

#endif // VEHICLE_H
