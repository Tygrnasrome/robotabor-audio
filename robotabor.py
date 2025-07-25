import brian.motors as motors
import math

#########################
### Šikovné konstanty ###
#########################
LARGE_MOTOR_MAX_SPEED = 1050  # stupňů za sekundu
MEDIUM_MOTOR_MAX_SPEED = 1560  # stupňů za sekundu


######################################
### Třída pro řízení pohybu robota ###
######################################
class Pilot:
    def __init__(
        self,
        left_motor: motors.Motor,
        right_motor: motors.Motor,
        wheel_diameter: float = 56,
        axle_width: float = 100,
        reverse: bool = False,
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
        self.reverse = bool(reverse)

        self.speed = 400  # deg/s
        self._angle_offset = (
            self.left_motor.current_angle() - self.right_motor.current_angle()
        )

    def _mm_to_deg(self, mm: float) -> float:
        """
        Převede vzdálenost v mm na úhel motoru ve stupních.

        Argumenty:
            mm (float): Vzdálenost v milimetrech.

        Návratová hodnota:
            float: Vzdálenost převedená na úhel v stupních.
        """
        return round(mm * 360 / (math.pi * self.wheel_diameter))

    def deg_to_mm(self, deg: float) -> float:
        """
        Převede úhel motoru ve stupních na vzdálenost v mm.

        Argumenty:
            deg (float): Úhel v stupních.

        Návratová hodnota:
            float: Úhel převedený na vzdálenost v milimetrech.
        """
        return deg * (math.pi * self.wheel_diameter) / 360

    def _run_motor_at_speeds(self, left_speed: float, right_speed: float):
        """Nastaví okamžité rychlosti motorů (deg/s, vč. znaménka)."""
        self.left_motor.run_at_speed(left_speed)
        self.right_motor.run_at_speed(right_speed)

    ################
    ### MOVEMENT ###
    ################
    def forward(self):
        """Jede vpřed rychlostí *speed* (deg/s) dokud se nezavolá `stop()`."""
        direction = -1 if self.reverse else 1
        self._run_motor_at_speeds(direction * self.speed, direction * self.speed)

    def backward(self):
        """Jede vzad rychlostí *speed* (deg/s) dokud se nezavolá `stop()`."""
        direction = 1 if self.reverse else -1
        self._run_motor_at_speeds(direction * self.speed, direction * self.speed)

    def travel(self, distance: float, wait_until_done: bool = True):
        """
        Jede danou vzdálenost v milimetrech.

        Argumenty:
            distance (float): Vzdálenost v milimetrech, kterou má robot ujet.
            wait_until_done (bool, volitelné): Pokud je True, počká, dokud robot nedojede. Výchozí hodnota je True.
        """
        degrees = self._mm_to_deg(distance)

        if self.reverse:
            degrees = -degrees
        self.left_motor.rotate_by_angle(round(degrees), round(self.speed), 0)
        self.right_motor.rotate_by_angle(round(degrees), round(self.speed), 0)

        if wait_until_done:
            self.left_motor.wait_for_movement()
            self.right_motor.wait_for_movement()

    def rotate(self, angle: float, wait_until_done: bool = True):
        """
        Otočí robota na místě o daný úhel v stupních.

        Argumenty:
            angle (float): Úhel otočení v stupních.
            wait_until_done (bool, volitelné): Pokud je True, počká, dokud robot nedokončí otáčení. Výchozí hodnota je True.
        """
        arc_mm = (angle / 360) * math.pi * self.axle_width
        degrees = self._mm_to_deg(arc_mm)

        left_speed = self.speed if not self.reverse else -self.speed
        right_speed = -left_speed

        self.left_motor.rotate_by_angle(round(degrees), round(left_speed), 0)
        self.right_motor.rotate_by_angle(-round(degrees), round(right_speed), 0)

        if wait_until_done:
            self.left_motor.wait_for_movement()
            self.right_motor.wait_for_movement()

    def steer(
        self, turn_rate: float, angle: float = None, wait_until_done: bool = True
    ):
        """
        Řízená jízda po kružnici.

        • *turn_rate* … −200 … 200
          0 = rovně, ±100 = vnitřní kolo stojí, ±200 = otáčení na místě
          záporné → zatáčí doprava, kladné → doleva

        • *angle*
          - `None`  → jede nekonečně dlouho, zastaví `stop()`
          - číslo   → ukončí, jakmile se čelo robota natočí o daný počet stupňů

        • *wait_until_done*
          `False`  → func vrátí hned, motory jedou dál
          `True` → po dokončení (nebo při nekonečné jízdě nevrací)
        """
        turn_rate = max(-200, min(200, turn_rate))

        if turn_rate == 0:
            if angle is not None:
                raise ValueError("U jizdy rovne neni uhel steer(0)")
            self.forward()
            return

        abs_tr = abs(turn_rate)
        if abs_tr <= 100:
            ratio = 1 - abs_tr / 100
        else:
            ratio = -(abs_tr - 100) / 100

        if turn_rate > 0:
            left_speed = self.speed * ratio
            right_speed = self.speed
        else:
            left_speed = self.speed
            right_speed = self.speed * ratio

        if self.reverse:
            left_speed = -left_speed
            right_speed = -right_speed

        if angle is None:
            self._run_motor_at_speeds(left_speed, right_speed)
            return

        mm_per_deg = (math.pi * self.wheel_diameter) / 360
        v_l_mm = left_speed * mm_per_deg
        v_r_mm = right_speed * mm_per_deg

        omega = (v_r_mm - v_l_mm) / self.axle_width

        t = math.radians(angle) / abs(omega)

        s_l_mm = v_l_mm * t
        s_r_mm = v_r_mm * t

        deg_l = self._mm_to_deg(s_l_mm)
        deg_r = self._mm_to_deg(s_r_mm)

        self.left_motor.rotate_by_angle(deg_l, abs(left_speed), 0)
        self.right_motor.rotate_by_angle(deg_r, abs(right_speed), 0)

        if wait_until_done:
            self.left_motor.wait_for_movement()
            self.right_motor.wait_for_movement()

    def stop(self):
        """
        Zastaví oba motory robota.
        """
        self.left_motor.brake()
        self.right_motor.brake()

    ######################
    ### INFO FUNCTIONS ###
    ######################
    def get_angle(self) -> float:
        """
        Vrátí aktuální úhel natočení robota (stupně).
        Kladné hodnoty = zatočení doprava (po směru hodin).
        """
        left_motor_rotation = self.left_motor.current_angle()
        right_motor_rotation = self.right_motor.current_angle()

        rotation_difference = (
            left_motor_rotation - right_motor_rotation - self._angle_offset
        )

        return rotation_difference / (math.pi * self.axle_width * 4) * 360

    def reset_angle(self):
        """
        Resetuje aktuální úhel natočení robota na 0 stupňů.
        """
        self._angle_offset = (
            self.left_motor.current_angle() - self.right_motor.current_angle()
        )

    def is_moving(self) -> bool:
        """Vrátí True když se minimálně jeden motor točí."""
        return (
            abs(self.left_motor.current_speed()) > 0
            or abs(self.right_motor.current_speed()) > 0
        )

    def get_acceleration(self):
        """
        Vrátí aktuální nastavení akcelerace motorů.
        """
        return {
            "left_motor": self.left_motor.limits.acceleration,
            "right_motor": self.right_motor.limits.acceleration,
        }

    def set_acceleration(self, acceleration: float):
        """
        Nastaví akceleraci motorů.

        Argumenty:
            acceleration (float): Akcelerace v stupních za sekundu na druhou. Defaultní hodnota je 6000 (maximální zrychlení).
        """
        self.left_motor.limits.acceleration = acceleration
        self.right_motor.limits.acceleration = acceleration

    def set_speed(self, speed: float):
        """Nastaví základní rychlost *speed* (deg/s) otáčení motorů.

        Argumenty:
            speed (float): Rychlost v stupních za sekundu. Výchozí hodnota je 400.
        """
        self.speed = speed
