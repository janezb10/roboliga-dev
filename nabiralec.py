
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# """ 
# Program za vodenje robota EV3 po seznamu tock na poligonu.
# [Robo liga FRI 2020: Cebelnjak]
# """

__author__ = "Laboratory for adaptive systems and parallel processing"
__copyright__ = "Copyright 2020, UL FRI - LASPP"
__credits__ = ["Laboratory for adaptive systems and parallel processing"]
__license__ = "GPL"
__version__ = "0.1"
__maintainer__ = "Nejc Ilc"
__email__ = "nejc.ilc@fri.uni-lj.si"
__status__ = "Active"


# in uprorabljati "code auto-completition":
# pip install python-ev3dev
from ev3dev.ev3 import TouchSensor, Button, LargeMotor, Sound
# Na EV3 robotu je potrebno namestiti paketa ujson in pycurl:
# sudo apt-get update
# sudo apt-get install python3-pycurl
# sudo apt-get install python3-ujson
import pycurl
import ujson
import sys
import math
from io import BytesIO
from time import time, sleep
from enum import Enum
from collections import deque

# Nastavitev najpomembnjsih parametrov
# ID robota. Spremenite, da ustreza stevilki oznacbe, ki je dolocena vasi ekipi.
ROBOT_ID = 25
# Naslov IP igralnega streznika.
SERVER_IP = "192.168.1.130:8088/game/"
# Datoteka na igralnem strezniku s podatki o tekmi.
GAME_ID = "5a72"

# Priklop motorjev na izhode.
MOTOR_LEFT_PORT = 'outB'
MOTOR_RIGHT_PORT = 'outC'

# Najvisja dovoljena hitrost motorjev (teoreticno je to 1000).
SPEED_MAX = 900
# Najvisja dovoljena nazivna hitrost motorjev pri voznji naravnost.
# Naj bo manjsa kot SPEED_MAX, da ima robot se moznost zavijati.
SPEED_BASE_MAX = 800

# Parametri za PID
# Obracanje na mestu in zavijanje med voznjo naravnost
PID_TURN_KP = 3.0
PID_TURN_KI = 0.5
PID_TURN_KD = 0.0
PID_TURN_INT_MAX = 100
# Nazivna hitrost pri voznji naravnost.
PID_STRAIGHT_KP = 2.0
PID_STRAIGHT_KI = 0.5
PID_STRAIGHT_KD = 0.01
PID_STRAIGHT_INT_MAX = 100

# Dolzina FIFO vrste za hranjenje meritev (oddaljenost in kot do cilja).
HIST_QUEUE_LENGTH = 3

# Razdalje - tolerance
# Dovoljena napaka v oddaljenosti do cilja [mm].
DIST_EPS = 20
# Dovoljena napaka pri obracanju [stopinje].
DIR_EPS = 5
# Blizina cilja [mm].
DIST_NEAR = 100
# Koliko sekund je robot lahko stanju voznje naravnost v blizini cilja
# (oddaljen manj kot DIST_NEAR), preden sprozimo varnostni mehanizem
# in ga damo v stanje obracanja na mestu.
TIMER_NEAR_TARGET = 3


class State(Enum):
    """
    Stanja robota.
    """

    def __str__(self):
        return str(self.name)
    IDLE = 0
    TURN = 1
    DRIVE_STRAIGHT = 2
    LOAD_NEXT_TARGET = 3


class Connection():
    """
    Objekt za vzpostavljanje povezave s streznikom.
    """

    def __init__(self, url: str):
        """
        Inicializacija nove povezave.
        Argumenti:
        url: pot do datoteke na strezniku (URL)
        """
        self._url = url
        self._buffer = BytesIO()
        self._pycurlObj = pycurl.Curl()
        self._pycurlObj.setopt(self._pycurlObj.URL, self._url)
        self._pycurlObj.setopt(self._pycurlObj.CONNECTTIMEOUT, 10)
        self._pycurlObj.setopt(self._pycurlObj.WRITEDATA, self._buffer)

    def request(self, debug=False):
        """
        Nalaganje podatkov s streznika.
        """
        # Pocistimo pomnilnik za shranjevanje sporocila
        self._buffer.seek(0, 0)
        self._buffer.truncate()
        # Posljemo zahtevek na streznik
        self._pycurlObj.perform()
        # Dekodiramo sporocilo
        msg = self._buffer.getvalue().decode()
        # Izluscimo podatke iz JSON
        try:
            return ujson.loads(msg)
        except ValueError as err:
            if debug:
                print('Napaka pri razclenjevanju datoteke JSON: ' + str(err))
                print('Sporocilo streznika:')
                print(msg)
            return -1

    def test_delay(self, num_iters: int = 10):
        """
        Merjenje zakasnitve pri pridobivanju podatkov o tekmi s streznika. 
        Zgolj informativno.
        """
        sum_time = 0
        for _ in range(num_iters):
            start_time = time()
            if self.request(True) == -1:
                robot_die()
            elapsed_time = time() - start_time
            sum_time += elapsed_time
        return sum_time / num_iters


class PID():
    """
    Implementacija algoritma za regulacijo PID.
    Nekaj virov za razjasnitev osnovnega nacela delovanja:
        - https://en.wikipedia.org/wiki/PID_controller
        - https://www.csimn.com/CSI_pages/PIDforDummies.html
        - https://blog.opticontrols.com/archives/344
        - https://www.youtube.com/watch?v=d2AWIA6j0NU
    """

    def __init__(
            self,
            setpoint: float,
            Kp: float,
            Ki: float = None,
            Kd: float = None,
            integral_limit: float = None):
        """
        Ustvarimo nov regulator PID s pripadajocimi parametri.
        Argumenti:
        setpoint: ciljna vrednost regulirane spremenljivke
        Kp: ojacitev proporcionalnega dela regulatorja.
            Visoke vrednosti pomenijo hitrejsi odziv sistema,
            vendar previsoke vrednosti povzrocijo oscilacije in nestabilnost.
        Ki: ojacitev integralnega clena regulatorja.
            Iznici napako v ustaljenem stanju. Zmanjsa odzivnost.
        Kd: ojacitev odvoda napake.
            Zmanjsa cas umirjanja in poveca odzivnost.
        integral_limit: najvisja vrednost integrala
        """
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._integral_limit = integral_limit
        self._error = None
        self._time = None
        self._integral = None
        self._value = None

    def reset(
            self,
            setpoint: float = None,
            Kp: float = None,
            Ki: float = None,
            Kd: float = None,
            integral_limit: float = None):
        """
        Ponastavitev regulatorja. 
        Lahko mu tudi spremenimo katero od vrednosti parametrov.
        Napaka, integral napake in cas se ponastavijo.
        """
        if setpoint is not None:
            self._setpoint = setpoint
        if Kp is not None:
            self._Kp = Kp
        if Ki is not None:
            self._Ki = Ki
        if Kd is not None:
            self._Kd = Kd
        if integral_limit is not None:
            self._integral_limit = integral_limit
        self._error = None
        self._time = None
        self._integral = None
        self._value = None

    def update(self, measurement: float) -> float:
        """
        Izracunamo vrednost izhoda regulatorja (regulirna velicina) 
        glede na izmerjeno vrednost regulirane velicine (measurement) 
        in ciljno vrednost (setpoint).
        Argumenti:
        measurement: s tipali izmerjena vrednost regulirane velicine
        Izhodna vrednost:
        regulirna velicina, s katero zelimo popraviti delovanje sistema 
        (regulirano velicino), da bo dosegel ciljno vrednost
        """
        if self._value is None:
            # Na zacetku se nimamo zgodovine meritev, zato inicializiramo
            # integral in vrnemo samo proporcionalni clen.
            self._value = measurement
            # Zapomnimo si zacetni cas.
            self._time = time()
            # Ponastavimo integral napake.
            self._integral = 0
            # Napaka = ciljna vrednost - izmerjena vrednost regulirane velicine.
            self._error = self._setpoint - measurement
            return self._Kp * self._error
        else:
            # Sprememba casa
            time_now = time()
            delta_time = time_now - self._time
            self._time = time_now
            # Izmerjena vrednost regulirane velicine.
            self._value = measurement
            # Napaka = ciljna vrednost - izmerjena vrednost regulirane velicine.
            error = self._setpoint - self._value

            # Proporcionalni del
            P = self._Kp * error

            # Integralni in odvodni clen sta opcijska.
            if self._Ki is None:
                I = 0
            else:
                # Integral se poveca za (sprememba napake) / (sprememba casa).
                self._integral += error * delta_time
                # Ojacitev integralnega dela.
                I = self._Ki * self._integral
                if self._integral_limit is not None:
                    # Omejimo integralni del.
                    I = max(min(I, self._integral_limit),
                            (-1)*(self._integral_limit))

            if self._Kd is None:
                D = 0
            else:
                # Odvod napake z ojacitvijo.
                D = self._Kd * (error - self._error) / delta_time
            # Posodobimo napako.
            self._error = error
            # Vrnemo regulirno velicino, sestavljeno iz proporcionalnega,
            # integralnega in odvodnega clena.
            return P + I + D


class Point():
    """
    Tocka na poligonu.
    """

    def __init__(self, position):
        self.x = position['x']
        self.y = position['y']

    def __str__(self):
        return '('+str(self.x)+', '+str(self.y)+')'


def get_angle(p1, a1, p2) -> float:
    """
    Izracunaj kot, za katerega se mora zavrteti robot, da bo obrnjen proti tocki p2.
    Robot se nahaja v tocki p1 in ima smer (kot) a1.
    """
    a = math.degrees(math.atan2(p2.y-p1.y, p2.x - p1.x))
    a_rel = a - a1
    if abs(a_rel) > 180:
        if a_rel > 0:
            a_rel = a_rel - 360
        else:
            a_rel = a_rel + 360

    return a_rel


def get_distance(p1: Point, p2: Point) -> float:
    """
    Evklidska razdalja med dvema tockama na poligonu.
    """
    return math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)


def init_large_motor(port: str) -> LargeMotor:
    """
    Preveri, ali je motor priklopljen na izhod `port`.
    Vrne objekt za motor (LargeMotor).
    """
    motor = LargeMotor(port)
    while not motor.connected:
        print('\nPriklopi motor na izhod ' + port +
              ' in pritisni ter spusti gumb DOL.')
        wait_for_button('down')
        motor = LargeMotor(port)
    return motor


def init_sensor_touch() -> TouchSensor:
    """
    Preveri, ali je tipalo za dotik priklopljeno na katerikoli vhod. 
    Vrne objekt za tipalo.
    """
    sensor = TouchSensor()
    while not sensor.connected:
        print('\nPriklopi tipalo za dotik in pritisni ter spusti gumb DOL.')
        wait_for_button('down')
        sensor = TouchSensor()
    return sensor


def wait_for_button(btn_name: str = 'down'):
    """
    cakaj v zanki dokler ni gumb z imenom `btn_name` pritisnjen in nato sproscen.
    """
    while not getattr(btn, btn_name):
        pass
    flag = False
    while getattr(btn, btn_name):
        if not flag:
            flag = True


def beep(duration=1000, freq=440):
    """
    Potrobi s frekvenco `freq` za cas `duration`. Klic ne blokira.
    """
    Sound.tone(freq, duration)
    # ce zelimo, da blokira, dokler se pisk ne konca.
    #Sound.tone(freq, duration).wait()


def robot_die():
    """
    Koncaj s programom na robotu. Ustavi motorje.
    """
    print('KONEC')
    motor_left.stop(stop_action='brake')
    motor_right.stop(stop_action='brake')
    Sound.play_song((
        ('D4', 'e'),
        ('C4', 'e'),
        ('A3', 'h')))
    sys.exit(0)


# -----------------------------------------------------------------------------
# NASTAVITVE TIPAL, MOTORJEV IN POVEZAVE S STREzNIKOM
# -----------------------------------------------------------------------------
# Nastavimo tipala in gumbe.
print('Priprava tipal ... ', end='', flush=True)
btn = Button()
#sensor_touch = init_sensor_touch()
print('OK!')

# Nastavimo velika motorja. Priklopljena naj bosta na izhoda A in D.
print('Priprava motorjev ... ', end='')
motor_left = init_large_motor(MOTOR_LEFT_PORT)
motor_right = init_large_motor(MOTOR_RIGHT_PORT)
print('OK!')

# Nastavimo povezavo s streznikom.
url = SERVER_IP + GAME_ID
print('Vspostavljanje povezave z naslovom ' + url + ' ... ', end='', flush=True)
conn = Connection(url)
print('OK!')

# Informativno izmerimo zakasnitev pri pridobivanju podatkov (povprecje num_iters meritev)
print('Zakasnitev v komunikaciji s streznikom ... ', end='', flush=True)
print('%.4f s' % (conn.test_delay(num_iters=10)))


# -----------------------------------------------------------------------------
# PRIPRAVA NA TEKMO
# -----------------------------------------------------------------------------
# Pridobimo podatke o tekmi.
game_state = conn.request()
# Ali nas robot sploh tekmuje? ce tekmuje, ali je team1 ali team2?
if ROBOT_ID == game_state['teams']['team1']['id']:
    team_my_tag = 'team1'
    team_op_tag = 'team2'
elif ROBOT_ID == game_state['teams']['team2']['id']:
    team_my_tag = 'team2'
    team_op_tag = 'team1'
else:
    print('Robot ne tekmuje.')
    robot_die()
print('Robot tekmuje in ima interno oznako "' + team_my_tag + '"')

# Doloci cilj za robota (seznam tock na poligonu).
# Nasem primeru se bo vozil po notranjih kotih obeh kosar.
targets_list = [
    Point(game_state['fields']['baskets']['team1']['bottomRight']),
    Point(game_state['fields']['baskets']['team1']['topRight']),
    Point(game_state['fields']['baskets']['team2']['topLeft']),
    Point(game_state['fields']['baskets']['team2']['bottomLeft']),
]
print('Seznam ciljnih tock:')
for trgt in targets_list:
    print('\t' + str(trgt))


# -----------------------------------------------------------------------------
# GLAVNA ZANKA
# -----------------------------------------------------------------------------
print('Izvajam glavno zanko. Prekini jo s pritiskon na tipko DOL.')
print('Cakam na zacetek tekme ...')

# Zacetno stanje.
state = State.IDLE
# Prejsnje stanje.
state_old = -1
# Indeks trenutne ciljne lokacije.
target_idx = 0

# Regulator PID za obracanje na mestu.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
# Nasa regulirana velicina je torej kar napaka kota, ki mora biti 0.
# To velja tudi za regulacijo voznje naravnost.
PID_turn = PID(
    setpoint=0,
    Kp=PID_TURN_KP,
    Ki=PID_TURN_KI,
    Kd=PID_TURN_KD,
    integral_limit=PID_TURN_INT_MAX)

# PID za voznjo naravnost - regulira nazivno hitrost za oba motorja,
# ki je odvisna od oddaljenosti od cilja.
# setpoint=0 pomeni, da mora biti razdalja med robotom in ciljem enaka 0.
PID_frwd_base = PID(
    setpoint=0,
    Kp=PID_STRAIGHT_KP,
    Ki=PID_STRAIGHT_KI,
    Kd=PID_STRAIGHT_KD,
    integral_limit=PID_STRAIGHT_INT_MAX)

# PID za obracanje med voznjo naravnost.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
PID_frwd_turn = PID(
    setpoint=0,
    Kp=PID_TURN_KP,
    Ki=PID_TURN_KI,
    Kd=PID_TURN_KD,
    integral_limit=PID_TURN_INT_MAX)

# Hitrost na obeh motorjih.
speed_right = 0
speed_left = 0

# Zgodovina (okno) zadnjih nekaj vrednosti meritev kota in razdalje.
# Implementirana je kot vrsta FIFO.
robot_dir_hist = deque([180.0] * HIST_QUEUE_LENGTH)
robot_dist_hist = deque([math.inf] * HIST_QUEUE_LENGTH)

# Merimo cas obhoda zanke. Za visoko odzivnost robota je zelo pomembno,
# da je ta cas cim krajsi.
t_old = time()

do_main_loop = True
while do_main_loop and not btn.down:

    time_now = time()
    loop_time = time_now - t_old
    t_old = time_now

    # Zaznaj spremembo stanja.
    if state != state_old:
        state_changed = True
    else:
        state_changed = False
    state_old = state

    # Iz seznama ciljev izberi naslednjega.
    target = targets_list[target_idx]

    # Osvezi stanje tekme.
    game_state = conn.request()
    if game_state == -1:
        print('Napaka v paketu, ponovni poskus ...')
    else:
        game_on = game_state['gameOn']
        time_left = game_state['timeLeft']

        # Pridobi pozicijo in orientacijo svojega robota;
        # najprej pa ga poisci v tabeli vseh robotov na poligonu.
        robot_pos = None
        robot_dir = None
        for robot_data in game_state['objects']['robots'].values():
            if robot_data['id'] == ROBOT_ID:
                robot_pos = Point(robot_data['position'])
                robot_dir = robot_data['dir']
        # Ali so podatki o robotu veljavni? ce niso, je zelo verjetno,
        # da sistem ne zazna oznake na robotu.
        robot_alive = (robot_pos is not None) and (robot_dir is not None)

        # ce tekma poteka in je oznaka robota vidna na kameri,
        # potem izracunamo novo hitrost na motorjih.
        # Sicer motorje ustavimo.
        if game_on and robot_alive:
            # Razdalja med robotom in ciljem.
            target_dist = get_distance(robot_pos, target)
            # Kot med robotom in ciljem.
            target_angle = get_angle(robot_pos, robot_dir, target)

            # Spremljaj zgodovino meritev kota in oddaljenosti.
            # Odstrani najstarejsi element in dodaj novega - princip FIFO.
            robot_dir_hist.popleft()
            robot_dir_hist.append(target_angle)
            robot_dist_hist.popleft()
            robot_dist_hist.append(target_dist)

            if state == State.IDLE:
                # Stanje mirovanja - tu se odlocamo, kaj bo robot sedaj pocel.
                speed_right = 0
                speed_left = 0
                # Preverimo, ali je robot na ciljni tocki.
                # ce ni, ga tja posljemo.
                if target_dist > DIST_EPS:
                    state = State.TURN
                    robot_near_target_old = False
                else:
                    state = State.LOAD_NEXT_TARGET

            elif state == State.LOAD_NEXT_TARGET:
                # Nalozimo naslednjo ciljno tocko iz seznama.
                target_idx = target_idx + 1
                # ce smo prisli do konca seznama, gremo spet od zacetka
                if target_idx >= len(targets_list):
                    target_idx = 0
                state = State.IDLE

            elif state == State.TURN:
                # Obracanje robota na mestu, da bo obrnjen proti cilju.
                if state_changed:
                    # ce smo ravno prisli v to stanje, najprej ponastavimo PID.
                    PID_turn.reset()

                # Ali smo ze dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjsa od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0:
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.DRIVE_STRAIGHT
                else:
                    # Reguliramo obracanje.
                    # Ker se v regulatorju trenutna napaka izracuna kot:
                    #   error = setpoint - measurement,
                    # dobimo negativno vrednost, ko se moramo zavrteti
                    # v pozitivno smer.
                    # Primer:
                    #   Robot ima smer 90 stopinj (obrnjen je proti "severu").
                    #   Cilj se nahaja na njegovi levi in da ga doseze,
                    #   se mora obrniti za 90 stopinj.
                    #       setpoint=0
                    #       target_angle = measurement = 90
                    #       error = setpoint - measurement = -90
                    #       u = funkcija, odvisna od error in parametrov PID.
                    #   ce imamo denimo Kp = 1, Ki = Kd = 0, potem bo u = -90.
                    #   Robot se mora zavrteti v pozitivno smer,
                    #   torej z desnim kolesom naprej in levim nazaj.
                    #   Zato:
                    #   speed_right = -u
                    #   speed_left = u
                    #   Lahko bi tudi naredili droben trik in bi rekli:
                    #       measurement= -target_angle.
                    #   V tem primeru bi bolj intuitivno nastavili
                    #   speed_right = u in speed_left = -u.
                    u = PID_turn.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.DRIVE_STRAIGHT:
                # Voznja robota naravnost proti ciljni tocki.
                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    PID_frwd_base.reset()
                    PID_frwd_turn.reset()
                    timer_near_target = TIMER_NEAR_TARGET

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v blizino cilja.
                    # Zacnimo odstevati varnostno budilko.
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo ze na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjsa ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0:
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.LOAD_NEXT_TARGET
                elif timer_near_target < 0:
                    # Smo morda blizu cilja in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.TURN
                else:
                    u_turn = PID_frwd_turn.update(measurement=target_angle)
                    # Ker je napaka izracunana kot setpoint - measurement in
                    # smo nastavili setpoint na 0, bomo v primeru u_base dobili
                    # negativne vrednosti takrat, ko se bo robot moral premikati
                    # naprej. Zato dodamo minus pri izracunu hitrosti motorjev.
                    u_base = PID_frwd_base.update(measurement=target_dist)
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo se manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = -u_base - u_turn
                    speed_left = -u_base + u_turn

            # Omejimo vrednosti za hitrosti na motorjih.
            speed_right = round(
                            min(
                                max(speed_right, -SPEED_MAX),
                                SPEED_MAX)
                            )
            speed_left = round(
                            min(
                                max(speed_left, -SPEED_MAX),
                                SPEED_MAX)
                            )

            # Izracunane hitrosti zapisemo na motorje.
            motor_right.run_forever(speed_sp=speed_right)
            motor_left.run_forever(speed_sp=speed_left)

        else:
            # Robot bodisi ni viden na kameri bodisi tekma ne tece, 
            # zato ustavimo motorje.
            motor_left.stop(stop_action='brake')
            motor_right.stop(stop_action='brake')

# Konec programa
robot_die()