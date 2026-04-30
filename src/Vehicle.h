#ifndef VEHICLE_H
#define VEHICLE_H

typedef struct
{
    /* Параметри, які задають фізичні властивості електромобіля. */
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
    /* params не змінюються під час руху, а стан нижче оновлюється на кожному кроці симуляції. */
    VehicleParams params;
    double commanded_torque_nm;
    double speed_m_per_s;
    double distance_m;
} Vehicle;

/* Ініціалізує модель електромобіля початковими параметрами і нульовим станом.
 * vehicle - об'єкт, який треба заповнити.
 * params - набір фізичних параметрів електромобіля.
 */
void Vehicle_init(Vehicle* vehicle, const VehicleParams* params);
/* Задає бажаний момент двигуна з урахуванням допустимих меж.
 * vehicle - модель електромобіля, для якої задається команда.
 * torque_nm - бажаний момент у ньютон-метрах.
 */
void Vehicle_set_torque(Vehicle* vehicle, double torque_nm);
/* Виконує один крок симуляції тривалістю 1/50 секунди.
 * vehicle - модель електромобіля, стан якої потрібно оновити.
 */
void Vehicle_step_50hz(Vehicle* vehicle);
/* Повертає поточну лінійну швидкість електромобіля.
 * vehicle - модель електромобіля, з якої читається швидкість.
 */
double Vehicle_get_speed(const Vehicle* vehicle);

#endif // VEHICLE_H
