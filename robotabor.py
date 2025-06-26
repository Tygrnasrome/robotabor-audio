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
        wheel_diameter=56,
        axle_width=100,
        reverse=False
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
        self.angle_offset = self.left_motor.current_angle() - self.right_motor.current_angle()
        
    def _mm_to_deg(self, mm: float) -> float:
        """
        Převede vzdálenost v milimetrech na úhel v stupních.
        
        Argumenty:
            mm (float): Vzdálenost v milimetrech.
        
        Návratová hodnota:
            float: Vzdálenost převedená na úhel v stupních.
        """
        return mm * 360 / (math.pi * self.wheel_diameter)
    
    def deg_to_mm(self, deg: float) -> float:
        """
        Převede úhel v stupních na vzdálenost v milimetrech.
        
        Argumenty:
            deg (float): Úhel v stupních.
        
        Návratová hodnota:
            float: Úhel převedený na vzdálenost v milimetrech.
        """
        return deg * (math.pi * self.wheel_diameter) / 360
    
    def _run_motor_at_speeds(self, left_speed: float, right_speed: float):
        """Nastaví rychlosti otáčení motorů."""
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
        """
        Jede zpět rychlostí *speed* (deg/s) dokud se nezavolá `stop()`.
        """
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
        speed = self.speed if not self.reverse else -self.speed
        
        self.left_motor.rotate_by_angle(degrees, speed, 0)
        self.right_motor.rotate_by_angle(degrees, speed, 0)
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
        degrees = self._mm_to_deg((angle/360) * math.pi * self.axle_width)
        left_speed = self.speed if not self.reverse else -self.speed
        right_speed = -left_speed
        
        print(f"Rotating {angle} degrees ({degrees} degrees) with speeds: left={left_speed}, right={right_speed}")
        self.left_motor.rotate_by_angle(degrees, left_speed, 0)
        self.right_motor.rotate_by_angle(-degrees, right_speed, 0)
        
        if wait_until_done:
            self.left_motor.wait_for_movement()
            self.right_motor.wait_for_movement()
    
    
    def steer(self, turn_rate: float, angle:float = 0, wait_until_done: bool = True):
        """
        Otočí robota o daný úhel v milimetrech. Pokud je nějaký *angle*, zastaví se když je dosaženo tohoto úhlu.
        
        Argumenty:
            turn_rate (float): Rychlost otáčení v milimetrech za sekundu.
            angle (float, volitelné): Úhel otočení v stupních. Výchozí hodnota je 0.
        """
        # TODO IMPLEMENT
        

    def stop(self):
        """
        Zastaví robota.
        """
        self.left_motor.brake()
        self.right_motor.brake()
    
    ######################
    ### INFO FUNCTIONS ###
    ######################

    def get_angle(self):
        """
        Vrátí aktuální úhel natočení robota.
        
        Návratová hodnota:
            float: Aktuální úhel natočení robota v stupních.
        """
        return math.degrees(math.atan((self.left_motor.current_angle() - self.right_motor.current_angle() - self.angle_offset) * math.pi * self.wheel_diameter / 360 / self.axle_width))

    def reset_angle(self):
        """
        Resetuje aktuální úhel natočení robota na 0 stupňů.
        """
        self.angle_offset = self.left_motor.current_angle() - self.right_motor.current_angle()

    def get_travel(self):
        """
        Vrátí aktuální vzdálenost, kterou robot ujel.

        Návratová hodnota:
            float: Aktuální vzdálenost, kterou robot ujel v milimetrech.
        """
        return (self.left_motor.current_angle() + self.right_motor.current_angle()) / 2

    def is_moving(self):
        """
        Zjistí, zda se robot momentálně pohybuje.

        Návratová hodnota:
            bool: True, pokud se robot pohybuje, jinak False.
        """
        return abs(self.left_motor.current_speed) > 0 or abs(self.right_motor.current_speed) > 0

    def get_acceleration(self):
        """
        Vrátí aktuální nastavení akcelerace motorů.
        """
        return {
            "left_motor": self.left_motor.limits.acceleration,
            "right_motor": self.right_motor.limits.acceleration
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
        """
        Nastaví rychlost otáčení motorů.
        
        Argumenty:
            speed (float): Rychlost v stupních za sekundu. Výchozí hodnota je 400.
        """
        self.speed = speed

###############################
### TESTING THE PILOT CLASS ###
###############################
from time import sleep
pilot = Pilot(
    motors.EV3LargeMotor(motors.MotorPort.B),
    motors.EV3LargeMotor(motors.MotorPort.C),
    wheel_diameter=56,
    axle_width=100,
    reverse=False
)

pilot.print_limits()
pilot.set_acceleration(2000)
pilot.forward()
sleep(1)
pilot.stop()
pilot.backward()
sleep(1)
pilot.stop()
sleep(1)
pilot.rotate(90)
sleep(1)
pilot.rotate(-90)
sleep(1)
pilot.stop()
sleep(5)
