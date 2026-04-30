#include "Vehicle.h"

enum
{
    /* Симуляція оновлюється 50 разів за секунду. */
    VEHICLE_UPDATE_HZ = 50
};

/* Обмежує число знизу і зверху.
 * value - значення, яке треба перевірити.
 * min_value - найменше допустиме значення.
 * max_value - найбільше допустиме значення.
 */
static double clamp(double value, double min_value, double max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

/* Повертає модуль числа типу double.
 * value - число, для якого обчислюється модуль.
 */
static double abs_double(double value)
{
    return value < 0.0 ? -value : value;
}

/* Повертає знак числа: -1, 0 або 1.
 * value - число, знак якого треба визначити.
 */
static double sign_double(double value)
{
    if (value > 0.0)
        return 1.0;

    if (value < 0.0)
        return -1.0;

    return 0.0;
}

/* Ініціалізує модель електромобіля початковими параметрами і нульовим станом.
 * vehicle - об'єкт, який треба заповнити.
 * params - набір фізичних параметрів електромобіля.
 */
void Vehicle_init(Vehicle* vehicle, const VehicleParams* params)
{
    /* Копіюємо параметри один раз, а поточний стан обнуляємо. */
    vehicle->params = *params;
    vehicle->commanded_torque_nm = 0.0;
    vehicle->speed_m_per_s = 0.0;
    vehicle->distance_m = 0.0;
}

/* Задає бажаний момент двигуна з урахуванням допустимих меж.
 * vehicle - модель електромобіля, для якої задається команда.
 * torque_nm - бажаний момент у ньютон-метрах.
 */
void Vehicle_set_torque(Vehicle* vehicle, double torque_nm)
{
    /* Команда моменту не може виходити за межі можливостей двигуна. */
    const double max_torque = vehicle->params.motor_max_torque_nm;
    vehicle->commanded_torque_nm = clamp(torque_nm, -max_torque, max_torque);
}

/* Виконує один крок симуляції тривалістю 1/50 секунди.
 * vehicle - модель електромобіля, стан якої потрібно оновити.
 */
void Vehicle_step_50hz(Vehicle* vehicle)
{
    /* dt — це тривалість одного кроку інтегрування. */
    const double dt = 1.0 / (double) VEHICLE_UPDATE_HZ;
    const double air_density_kg_per_m3 = 1.225;
    const double drag_coefficient = 1.0;
    const double gravity_m_per_s2 = 9.81;
    const double min_speed_epsilon = 1e-6;

    const VehicleParams* params = &vehicle->params;
    const double old_speed = vehicle->speed_m_per_s;
    /* Швидкість обертання колеса і двигуна отримуємо зі швидкості електромобіля. */
    const double wheel_angular_speed = params->wheel_radius_m > 0.0 ? old_speed / params->wheel_radius_m : 0.0;
    const double motor_angular_speed = wheel_angular_speed * params->gear_ratio;

    double motor_torque = vehicle->commanded_torque_nm;
    /* Якщо двигун вже досяг граничної швидкості, далі момент не додаємо. */
    if (abs_double(motor_angular_speed) >= params->motor_max_speed_rad_per_sec)
        motor_torque = 0.0;

    /* Перетворюємо момент двигуна на тягову силу в плямі контакту колеса. */
    const double wheel_torque = motor_torque * params->gear_ratio;
    const double tractive_force = params->wheel_radius_m > 0.0 ? wheel_torque / params->wheel_radius_m : 0.0;

    const double rolling_force_magnitude =
        params->rolling_resistance_coefficient * params->mass_kg * gravity_m_per_s2;
    const double drag_force = 0.5 * air_density_kg_per_m3 * drag_coefficient * params->frontal_area_m2 * old_speed * abs_double(old_speed);

    double opposing_force = drag_force;
    /* На нульовій швидкості імітуємо "залипання": слабка тяга не зрушить авто з місця. */
    if (abs_double(old_speed) > min_speed_epsilon)
        opposing_force += rolling_force_magnitude * sign_double(old_speed);
    else if (abs_double(tractive_force) <= rolling_force_magnitude)
        opposing_force = tractive_force;
    else
        opposing_force += rolling_force_magnitude * sign_double(tractive_force);

    /* Другий закон Ньютона: F = m * a, звідси a = F / m. */
    const double net_force = tractive_force - opposing_force;
    const double acceleration = params->mass_kg > 0.0 ? net_force / params->mass_kg : 0.0;

    double new_speed = old_speed + acceleration * dt;
    /* Не дозволяємо швидкості "перестрибнути" через нуль за один крок. */
    if (old_speed > 0.0 && new_speed < 0.0)
        new_speed = 0.0;
    else if (old_speed < 0.0 && new_speed > 0.0)
        new_speed = 0.0;

    vehicle->speed_m_per_s = new_speed;
    /* Пройдений шлях оновлюємо за формулою площі трапеції. */
    vehicle->distance_m += 0.5 * (old_speed + new_speed) * dt;
}

/* Повертає поточну лінійну швидкість електромобіля.
 * vehicle - модель електромобіля, з якої читається швидкість.
 */
double Vehicle_get_speed(const Vehicle* vehicle)
{
    return vehicle->speed_m_per_s;
}
