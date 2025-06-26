import brian.motors as motors
import math


### Šikovné konstanty ###
LARGE_MOTOR_MAX_SPEED = 1050  # stupňů za sekundu
MEDIUM_MOTOR_MAX_SPEED = 1560  # stupňů za sekundu


### Třída pro řízení pohybu robota ###
class DriveBase:
    def __init__(
        self,
        left_motor: motors.Motor,
        right_motor: motors.Motor,
        wheel_diameter=56,
        axle_width=100,
    ):
        """
        Vytvoří DriveBase s *left_motor* a *right_motor*.

        Argumenty:
            left_motor (motors.Motor): Levý motor.
            right_motor (motors.Motor): Pravý motor.
            wheel_diameter (float, volitelné): Průměr kola v milimetrech. Výchozí hodnota je 56.
            axle_width (float, volitelné): Vzdálenost mezi koly v milimetrech. Výchozí hodnota je 100.
        """
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter
        self.axle_width = axle_width

        self.speed = 100  # mm/s

    def straight(self, distance: float):
        """
        Ujede rovně *distance* milimetrů a zastaví se.
        """
        self.left_motor.rotate_by_angle(distance / (math.pi * self.wheel_diameter) * 360, self.speed, 0)
        self.right_motor.rotate_by_angle(distance / (math.pi * self.wheel_diameter) * 360, self.speed)

    def turn(self, angle: float):
        """
        Otočí se na místě o *angle* stupňů (kladné = ve směru hodinových ručiček).
        """
        distance = angle / 360 * math.pi * self.axle_width
        left_angle = distance / (math.pi * self.wheel_diameter) * 360
        right_angle = -left_angle

        self.left_motor.rotate_by_angle(left_angle, self.speed, 0)
        self.right_motor.rotate_by_angle(right_angle, self.speed)

    def drive(self, drive_speed: float, turn_rate: float):
        """
        Jede donekonečna rychlostí *drive_speed* (mm/s) a zatačí *turn_rate* (°/s). Lze zastavit pomocí `stop()`.
        """
        self.left_motor.run_at_speed(drive_speed + turn_rate * self.axle_width / 2)
        self.right_motor.run_at_speed(drive_speed - turn_rate * self.axle_width / 2)

    def hold(self):
        """
        Drží motory zabržděné.
        """
        self.left_motor.hold()
        self.right_motor.hold()

    def stop(self):
        """
        Zastaví motory.
        """
        self.left_motor.brake()
        self.right_motor.brake()

### Příklad použití třídy DriveBase ###
from time import sleep

base = DriveBase(motors.EV3LargeMotor(motors.MotorPort.B), motors.EV3LargeMotor(motors.MotorPort.C), 56,  100) # Nastavení DriveBase s levým a pravým motorem, průměrem kola 56 mm a vzdáleností mezi koly 100 mm

base.straight(1000)  # Jede 1000 mm rovně
base.turn(360)  # Otočí se o 360 stupňů
sleep(1)  # Počká 1 sekundu
base.turn(-360)  # Otočí se o -360 stupňů protisměru hodinových ručiček
sleep(1)

base.drive(1000, 10) # Jede rychlostí 1000 mm/s a zatáčí 10°/s
sleep(5)
base.stop() # Zastaví motory
sleep(1)
