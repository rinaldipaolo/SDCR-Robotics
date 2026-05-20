"""
╔══════════════════════════════════════════════════════════════════════════════╗
║          SDCR ROBOTICS – SOFTWARE DI CONTROLLO DRONE VTOL                  ║
║          Bozza tecnica commentata – Ipotesi architetturali e codice         ║
╚══════════════════════════════════════════════════════════════════════════════╝

SOMMARIO DELLE IPOTESI SOFTWARE
================================

OPZIONE A ── pymavlink (Python + MAVLink)          [CONSIGLIATA]
   Protocollo MAVLink direttamente via Python.
   Ideale per: controllo personalizzato, telemetria LoRa, scripting missioni.
   Compatibile con: Pixhawk 4 / Cube Orange / Holybro Durandal + ArduPilot o PX4.

OPZIONE B ── MAVSDK-Python
   Wrapper Python di alto livello su MAVLink. Sintassi async/await moderna.
   Ideale per: sviluppo rapido, missioni waypoint, task autonomi.

OPZIONE C ── ROS 2 + MAVROS
   Framework robotico completo. Molto potente ma complesso.
   Ideale per: integrazione sensori avanzata, computer vision, espansione futura.

OPZIONE D ── QGroundControl / Mission Planner (GCS software)
   Software GCS già pronti (no codice richiesto).
   Ideale per: testing, configurazione FC, missioni semplici via GUI.

──────────────────────────────────────────────────────────────────────────────
STACK TECNOLOGICO RACCOMANDATO PER SDCR ROBOTICS
──────────────────────────────────────────────────────────────────────────────

  Flight Controller:  Pixhawk 4 / Cube Orange
  Firmware FC:        ArduPilot (ArduPlane VTOL)  ←  gestisce modalità VTOL nativa
  Link di controllo:  RF 2.4/5.8 GHz (MAVLink via UART/UDP)
  Link telemetria:    LoRa (MAVLink su seriale LoRa)
  Linguaggio GCS:     Python 3.10+
  Librerie:           pymavlink, mavsdk, asyncio, pyserial

  Installazione dipendenze:
    pip install pymavlink mavsdk pyserial asyncio

──────────────────────────────────────────────────────────────────────────────
"""

# ==============================================================================
# SEZIONE 1 – CONNESSIONE E CONFIGURAZIONE BASE (pymavlink)
# ==============================================================================
# pymavlink è la libreria Python ufficiale per comunicare col protocollo MAVLink.
# MAVLink è il protocollo standard usato da Pixhawk, ArduPilot, PX4.

from pymavlink import mavutil
import time
import threading

# ── Parametri di connessione ──────────────────────────────────────────────────
# Modifica CONNECTION_STRING in base al tuo setup:
#   Via USB/seriale:   'COM3' su Windows  |  '/dev/ttyUSB0' su Linux
#   Via UDP (WiFi/RF): 'udpin:0.0.0.0:14550'
#   Via TCP:           'tcp:192.168.1.100:5760'
#   Via LoRa (seriale): '/dev/ttyS1' a 57600 baud (tipico per moduli LoRa MAVLink)

CONNECTION_STRING = 'udpin:0.0.0.0:14550'   # Default: GCS riceve su porta UDP
BAUD_RATE         = 57600                    # Standard MAVLink su telemetria LoRa
SYSTEM_ID         = 255                      # ID del GCS (255 = ground control)
COMPONENT_ID      = 0                        # 0 = generico


def connetti_drone(connection_string: str, baud: int = 57600):
    """
    Crea la connessione MAVLink con il flight controller.

    Parametri:
        connection_string: stringa di connessione (seriale, UDP, TCP)
        baud: baud rate per connessioni seriali

    Ritorna:
        master: oggetto MAVLink connection
    """
    print(f"[SDCR] Connessione al drone: {connection_string}")
    master = mavutil.mavlink_connection(
        connection_string,
        baud=baud,
        source_system=SYSTEM_ID
    )

    # Attesa heartbeat: il drone invia un messaggio HEARTBEAT ogni secondo.
    # Questo conferma che il flight controller è attivo e risponde.
    print("[SDCR] Attesa HEARTBEAT dal flight controller...")
    master.wait_heartbeat(timeout=10)
    print(f"[SDCR] ✓ Drone connesso! System ID: {master.target_system}, "
          f"Component: {master.target_component}")

    return master


# ==============================================================================
# SEZIONE 2 – TELEMETRIA (lettura dati dal drone)
# ==============================================================================
# Il drone trasmette continuamente messaggi MAVLink con GPS, batteria,
# altitudine, stato motori, ecc. Qui leggiamo i più importanti.

def leggi_telemetria(master) -> dict:
    """
    Legge un ciclo completo di telemetria dal drone.

    Messaggi MAVLink utilizzati:
        GLOBAL_POSITION_INT  → GPS (lat, lon, alt, velocità)
        BATTERY_STATUS       → voltaggio, corrente, percentuale carica
        ATTITUDE             → rollio, beccheggio, imbardata
        VFR_HUD              → airspeed, groundspeed, heading, throttle
        SYS_STATUS           → stato generale del sistema
        HEARTBEAT            → modalità di volo e stato armamento

    Ritorna:
        dict con tutti i valori letti
    """
    telemetria = {
        'gps_lat': None, 'gps_lon': None, 'gps_alt': None,
        'batteria_volt': None, 'batteria_pct': None,
        'roll': None, 'pitch': None, 'yaw': None,
        'velocita': None, 'heading': None,
        'armato': None, 'modalita': None
    }

    # ── GPS ──────────────────────────────────────────────────────────────────
    msg_gps = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg_gps:
        telemetria['gps_lat'] = msg_gps.lat / 1e7         # in gradi decimali
        telemetria['gps_lon'] = msg_gps.lon / 1e7
        telemetria['gps_alt'] = msg_gps.relative_alt / 1000  # in metri (AGL)

    # ── Batteria ─────────────────────────────────────────────────────────────
    msg_batt = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=2)
    if msg_batt:
        # voltages[0] in millivolt → conversione in Volt
        if msg_batt.voltages[0] != 65535:  # 65535 = valore non disponibile
            telemetria['batteria_volt'] = msg_batt.voltages[0] / 1000.0
        telemetria['batteria_pct'] = msg_batt.battery_remaining  # 0-100%

    # ── Assetto (IMU) ─────────────────────────────────────────────────────────
    msg_att = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if msg_att:
        import math
        telemetria['roll']  = math.degrees(msg_att.roll)
        telemetria['pitch'] = math.degrees(msg_att.pitch)
        telemetria['yaw']   = math.degrees(msg_att.yaw)

    # ── Velocità e Heading ────────────────────────────────────────────────────
    msg_hud = master.recv_match(type='VFR_HUD', blocking=True, timeout=2)
    if msg_hud:
        telemetria['velocita'] = msg_hud.groundspeed   # m/s
        telemetria['heading']  = msg_hud.heading        # 0-360°

    # ── Stato sistema (armato / modalità) ────────────────────────────────────
    msg_hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg_hb:
        telemetria['armato']   = bool(msg_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        # Decodifica modalità: in ArduPilot VTOL le modalità chiave sono:
        # 0=MANUAL, 3=AUTO, 5=GUIDED, 6=LOITER, 11=QHOVER, 12=QLOITER,
        # 13=QLAND, 17=QRTL, 20=QAUTOTUNE
        telemetria['modalita'] = msg_hb.custom_mode

    return telemetria


def monitor_telemetria_continuo(master, intervallo_sec: float = 1.0):
    """
    Thread separato per monitoraggio continuo della telemetria.
    Stampa i dati a schermo; in produzione, inviarli all'app mobile.

    Nota: ArduPilot invia telemetria automaticamente a ~1-4 Hz di default.
    Si può aumentare la frequenza con MAVLink_RATE_STREAM.
    """
    print("[SDCR] Avvio monitor telemetria...")

    while True:
        t = leggi_telemetria(master)
        print(
            f"\r[TEL] GPS: {t['gps_lat']:.5f},{t['gps_lon']:.5f} "
            f"Alt:{t['gps_alt']:.1f}m | "
            f"Batt:{t['batteria_volt']:.1f}V ({t['batteria_pct']}%) | "
            f"Roll:{t['roll']:.1f}° Pitch:{t['pitch']:.1f}° | "
            f"V:{t['velocita']:.1f}m/s | Armato:{t['armato']}",
            end='', flush=True
        )
        time.sleep(intervallo_sec)


# ==============================================================================
# SEZIONE 3 – COMANDI DI VOLO BASE
# ==============================================================================

def arma_drone(master, conferma_timeout: int = 5) -> bool:
    """
    Arma i motori del drone.
    ATTENZIONE: i motori inizieranno a girare dopo l'armamento!
    Assicurarsi che il drone sia in area sicura e in modalità corretta.

    In ArduPilot VTOL, per armare:
    - La modalità deve essere GUIDED, LOITER o QHOVER
    - Il pre-arm check deve essere superato (GPS lock, IMU calibrato, ecc.)
    """
    print("[SDCR] Invio comando ARM...")
    master.arducopter_arm()  # Funzione helper di pymavlink per ArduCopter/ArduPlane

    # Attesa conferma armamento tramite HEARTBEAT
    for _ in range(conferma_timeout * 10):
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[SDCR] ✓ Drone ARMATO!")
            return True
        time.sleep(0.1)

    print("[SDCR] ✗ Armamento fallito o timeout")
    return False


def disarma_drone(master):
    """
    Disarma i motori. Usare solo a terra!
    """
    print("[SDCR] Invio comando DISARM...")
    master.arducopter_disarm()
    print("[SDCR] ✓ Drone DISARMATO")


def imposta_modalita(master, modalita: str) -> bool:
    """
    Cambia la modalità di volo del drone.

    Modalità chiave per SDCR VTOL (ArduPlane con QuadPlane):
        'QHOVER'   → Hover VTOL stabile (buono per test iniziali)
        'QLOITER'  → Loiter VTOL con GPS (mantiene posizione)
        'GUIDED'   → Controllo via MAVLink (per waypoint da script)
        'AUTO'     → Missione autonoma caricata nel FC
        'QRTL'     → VTOL Return-To-Launch (fail-safe)
        'QLAND'    → Atterraggio verticale controllato
        'FBWA'     → Fly-By-Wire A (ala fissa, stabilizzato)
        'CRUISE'   → Crociera automatica (ala fissa)
        'LOITER'   → Loiter ala fissa (cerchio attorno a punto GPS)
    """
    modalita_map = master.mode_mapping()

    if modalita not in modalita_map:
        print(f"[SDCR] ✗ Modalità '{modalita}' non disponibile.")
        print(f"[SDCR] Modalità disponibili: {list(modalita_map.keys())}")
        return False

    mode_id = modalita_map[modalita]
    master.set_mode(mode_id)
    print(f"[SDCR] ✓ Modalità impostata a: {modalita} (ID: {mode_id})")
    return True


# ==============================================================================
# SEZIONE 4 – MISSIONE AUTONOMA CON WAYPOINT
# ==============================================================================
# ArduPilot supporta il caricamento di missioni con waypoint.
# La missione viene eseguita automaticamente in modalità AUTO.

def crea_missione_ricognizione(master, waypoints: list):
    """
    Carica una missione di ricognizione nel flight controller.

    Parametri:
        waypoints: lista di tuple (lat, lon, alt_m, velocita_ms)
                   es: [(45.0703, 7.6869, 100, 18), ...]
                   (Torino come esempio: lat=45.07, lon=7.69)

    Struttura di una missione VTOL tipica SDCR:
        Item 0: TAKEOFF VTOL (decollo verticale)
        Item 1-N: WAYPOINT (punti di navigazione)
        Item N+1: LAND VTOL (atterraggio verticale a casa)

    IMPORTANTE: ArduPlane con QuadPlane usa comandi specifici:
        MAV_CMD_NAV_VTOL_TAKEOFF  (84)  → decollo verticale
        MAV_CMD_NAV_WAYPOINT       (16)  → waypoint standard
        MAV_CMD_NAV_VTOL_LAND     (85)  → atterraggio verticale
        MAV_CMD_DO_CHANGE_SPEED   (178) → cambio velocità
        MAV_CMD_DO_SET_CAM_TRIGG_DIST → trigger camera ogni X metri
    """
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2

    wp_list = []

    # ── Item 0: Home position ─────────────────────────────────────────────────
    # Il punto home viene impostato automaticamente all'armamento tramite GPS.
    home = master.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
    home_lat = home.latitude  / 1e7 if home else waypoints[0][0]
    home_lon = home.longitude / 1e7 if home else waypoints[0][1]

    wp_home = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system, master.target_component,
        0,                                              # seq: numero item
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame: altitudine relativa
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        1, 1,                                           # current=1 (home), autocontinue=1
        0, 0, 0, 0,                                     # param1-4 (non usati per home)
        int(home_lat * 1e7),
        int(home_lon * 1e7),
        0                                               # altitudine home = 0m
    )
    wp_list.append(wp_home)

    # ── Item 1: VTOL Takeoff ──────────────────────────────────────────────────
    alt_decollo = 30  # metri AGL – quota sicura per iniziare la transizione

    wp_takeoff = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system, master.target_component,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        84,   # MAV_CMD_NAV_VTOL_TAKEOFF
        0, 1,
        0, 0, 0, 0,
        int(home_lat * 1e7),
        int(home_lon * 1e7),
        alt_decollo
    )
    wp_list.append(wp_takeoff)

    # ── Item 2...N: Waypoint di ricognizione ──────────────────────────────────
    for i, (lat, lon, alt, vel) in enumerate(waypoints):

        # Comando cambio velocità prima di ogni waypoint (opzionale)
        wp_speed = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system, master.target_component,
            len(wp_list),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            178,  # MAV_CMD_DO_CHANGE_SPEED
            0, 1,
            1,      # tipo: 1 = groundspeed
            vel,    # velocità in m/s
            -1,     # -1 = nessun cambio throttle
            0, 0, 0, 0
        )
        wp_list.append(wp_speed)

        # Waypoint di navigazione
        wp = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system, master.target_component,
            len(wp_list),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0,           # hold time (secondi di sosta sul waypoint, 0 = nessuna)
            2.0,         # acceptance radius in metri
            0, 0,
            int(lat * 1e7),
            int(lon * 1e7),
            alt
        )
        wp_list.append(wp)

    # ── Item finale: VTOL Land ────────────────────────────────────────────────
    wp_land = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system, master.target_component,
        len(wp_list),
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        85,   # MAV_CMD_NAV_VTOL_LAND
        0, 1,
        0, 0, 0, 0,
        int(home_lat * 1e7),
        int(home_lon * 1e7),
        0
    )
    wp_list.append(wp_land)

    # ── Invio missione al FC ──────────────────────────────────────────────────
    print(f"[SDCR] Caricamento missione: {len(wp_list)} item (inclusi home, takeoff, land)")

    # Step 1: invia MISSION_COUNT (numero totale di item)
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(wp_list),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    # Step 2: rispondi alle richieste MISSION_REQUEST_INT del FC
    for i, wp in enumerate(wp_list):
        msg = master.recv_match(type='MISSION_REQUEST_INT', blocking=True, timeout=5)
        if not msg:
            print(f"[SDCR] ✗ Timeout richiesta waypoint {i}")
            return False
        master.mav.send(wp)
        print(f"[SDCR]   → Waypoint {i+1}/{len(wp_list)} inviato")

    # Step 3: attesa ACK finale di conferma
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("[SDCR] ✓ Missione caricata con successo!")
        return True
    else:
        print(f"[SDCR] ✗ Errore caricamento missione: {ack.type if ack else 'timeout'}")
        return False


def avvia_missione(master):
    """
    Avvia la missione precedentemente caricata.
    Il drone deve essere armato e in modalità AUTO.
    """
    imposta_modalita(master, 'AUTO')
    # In modalità AUTO con la missione caricata, il drone esegue automaticamente
    # la sequenza: VTOL Takeoff → Waypoint 1 → ... → VTOL Land
    print("[SDCR] ✓ Missione avviata in modalità AUTO")


# ==============================================================================
# SEZIONE 5 – GESTIONE LoRa
# ==============================================================================
# LoRa è il canale di telemetria a lungo raggio.
# In ArduPilot, un modulo LoRa (es. RFD900, SiK, o LoRa con MAVLink bridge)
# appare come una porta seriale e trasmette automaticamente messaggi MAVLink.
#
# Due approcci:
#   A) LoRa con MAVLink nativo:  collegato direttamente come telemetria 2 del FC
#   B) LoRa custom (LoRaWAN o SX1276): bridge personalizzato via Python

import serial

class GestoreLoRa:
    """
    Gestisce la comunicazione LoRa per telemetria a lungo raggio.

    Questo esempio usa un modulo LoRa collegato via UART (es. Heltec LoRa 32,
    TTGO LoRa, o modulo SX1276) configurato come bridge MAVLink.

    Crittografia: AES-128 su ogni pacchetto (come da STEP 7 del progetto).
    """

    def __init__(self, porta: str = '/dev/ttyS1', baud: int = 9600):
        """
        porta: porta seriale del modulo LoRa (es. '/dev/ttyS1' o 'COM4')
        baud: baud rate (LoRa tipicamente 9600-57600)
        """
        self.porta     = porta
        self.baud      = baud
        self.serial    = None
        self.attivo    = False
        self._buffer   = b''

    def connetti(self) -> bool:
        """Apre la connessione seriale con il modulo LoRa."""
        try:
            self.serial = serial.Serial(
                port     = self.porta,
                baudrate = self.baud,
                timeout  = 1.0,
                bytesize = serial.EIGHTBITS,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE
            )
            self.attivo = True
            print(f"[LoRa] ✓ Connesso su {self.porta} a {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[LoRa] ✗ Errore connessione: {e}")
            return False

    def invia_pacchetto_telemetria(self, dati: dict) -> bool:
        """
        Serializza e invia un pacchetto di telemetria via LoRa.

        Formato pacchetto (JSON compresso):
            {t: timestamp, lat: ..., lon: ..., alt: ..., batt: ..., mode: ...}

        In produzione: aggiungere crittografia AES-128 come da STEP 7.
        """
        import json, struct

        if not self.attivo or not self.serial:
            return False

        # Costruzione pacchetto compatto (minimizza byte trasmessi su LoRa)
        pacchetto = {
            't':    int(time.time()),
            'lat':  round(dati.get('gps_lat', 0), 6),
            'lon':  round(dati.get('gps_lon', 0), 6),
            'alt':  round(dati.get('gps_alt', 0), 1),
            'bat':  dati.get('batteria_pct', 0),
            'v':    round(dati.get('velocita', 0), 1),
            'hdg':  dati.get('heading', 0),
            'arm':  1 if dati.get('armato') else 0,
            'mod':  dati.get('modalita', 0)
        }

        # ── Crittografia AES-128 (placeholder – da implementare con pycryptodome) ──
        # from Crypto.Cipher import AES
        # cipher = AES.new(AES_KEY_128BIT, AES.MODE_CBC)
        # dati_cifrati = cipher.encrypt(pad(json_bytes, AES.block_size))
        # ────────────────────────────────────────────────────────────────────────

        payload = json.dumps(pacchetto).encode('utf-8')

        # Header: 2 byte magic (0xAA 0xBB) + 2 byte lunghezza
        header = struct.pack('>BBH', 0xAA, 0xBB, len(payload))

        try:
            self.serial.write(header + payload)
            return True
        except serial.SerialException:
            return False

    def ricevi_comandi(self) -> dict | None:
        """
        Riceve comandi inviati via LoRa dal GCS (Ground Control Station).

        Comandi supportati:
            RTL  → Return To Launch (fail-safe)
            LAND → Atterraggio immediato
            MODE → Cambio modalità
            WP   → Nuovo waypoint di destinazione
        """
        if not self.serial or not self.serial.in_waiting:
            return None

        self._buffer += self.serial.read(self.serial.in_waiting)

        # Cerca il magic header nel buffer
        while len(self._buffer) >= 4:
            if self._buffer[0] == 0xAA and self._buffer[1] == 0xBB:
                import struct, json
                lunghezza = struct.unpack('>H', self._buffer[2:4])[0]
                if len(self._buffer) >= 4 + lunghezza:
                    payload = self._buffer[4:4+lunghezza]
                    self._buffer = self._buffer[4+lunghezza:]
                    try:
                        return json.loads(payload.decode('utf-8'))
                    except:
                        pass
            else:
                self._buffer = self._buffer[1:]  # scarta byte non valido

        return None

    def disconnetti(self):
        """Chiude la connessione LoRa."""
        if self.serial:
            self.serial.close()
        self.attivo = False
        print("[LoRa] Connessione chiusa")


# ==============================================================================
# SEZIONE 6 – SISTEMA FAIL-SAFE
# ==============================================================================
# Il fail-safe è critico per operazioni VTOL a medio raggio.
# ArduPilot ha fail-safe integrati, ma questo layer aggiunge logica applicativa.

class GestoreFailSafe:
    """
    Layer applicativo di fail-safe per SDCR Robotics.

    Integra i fail-safe di ArduPilot con controlli addizionali:
        1. Perdita segnale RF → RTL (Return To Launch) automatico
        2. Batteria bassa → QLAND forzato
        3. Perdita GPS → hover VTOL e attesa
        4. Timeout telemetria LoRa → avviso operatore
    """

    SOGLIA_BATTERIA_CRITICA = 15  # % – atterraggio forzato
    SOGLIA_BATTERIA_BASSA   = 25  # % – avviso operatore
    TIMEOUT_HEARTBEAT_SEC   = 5   # secondi senza heartbeat → emergenza

    def __init__(self, master, lora: GestoreLoRa | None = None):
        self.master   = master
        self.lora     = lora
        self._running = False
        self._ultimo_heartbeat = time.time()
        self._thread  = None

    def avvia(self):
        """Avvia il thread di monitoraggio fail-safe."""
        self._running = True
        self._thread = threading.Thread(
            target=self._loop_monitoraggio,
            daemon=True,
            name="FailSafe-Monitor"
        )
        self._thread.start()
        print("[FailSafe] ✓ Monitor attivo")

    def ferma(self):
        """Ferma il monitoraggio."""
        self._running = False
        print("[FailSafe] Monitor fermato")

    def _loop_monitoraggio(self):
        """Loop continuo di monitoraggio (eseguito in thread separato)."""
        while self._running:
            telemetria = leggi_telemetria(self.master)
            self._controlla_batteria(telemetria)
            self._controlla_heartbeat()
            time.sleep(0.5)  # check ogni 500ms

    def _controlla_batteria(self, tel: dict):
        """Gestisce i livelli critici di batteria."""
        pct = tel.get('batteria_pct')
        volt = tel.get('batteria_volt')

        if pct is None:
            return

        if pct <= self.SOGLIA_BATTERIA_CRITICA:
            print(f"[FailSafe] ⚠ CRITICO: Batteria al {pct}% ({volt}V) → ATTERRAGGIO FORZATO")
            self._esegui_atterraggio_emergenza()

        elif pct <= self.SOGLIA_BATTERIA_BASSA:
            print(f"[FailSafe] ⚠ AVVISO: Batteria bassa {pct}% → valutare RTL")
            # Notifica via LoRa
            if self.lora:
                self.lora.invia_pacchetto_telemetria(
                    {**tel, 'allarme': 'BATT_LOW'}
                )

        # Controllo anche per batteria LiPo 6S: soglia minima per cella
        # 6S LiPo: 22.2V nominale, 21.0V = vuota, 25.2V = piena
        if volt and volt < 21.0:
            print(f"[FailSafe] ⚠ CRITICO: Tensione LiPo sotto soglia ({volt}V < 21.0V)")
            self._esegui_atterraggio_emergenza()

    def _controlla_heartbeat(self):
        """Verifica che il link con il FC sia ancora attivo."""
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self._ultimo_heartbeat = time.time()
        elif time.time() - self._ultimo_heartbeat > self.TIMEOUT_HEARTBEAT_SEC:
            print("[FailSafe] ✗ TIMEOUT HEARTBEAT → link RF perso! RTL attivato da FC")
            # ArduPilot gestisce automaticamente il failsafe RF tramite
            # RADIO_FAILSAFE configurato nella parameter list del FC

    def _esegui_atterraggio_emergenza(self):
        """
        Attiva la procedura di atterraggio di emergenza VTOL.
        In ArduPilot QuadPlane: QLAND forza atterraggio verticale immediato.
        """
        print("[FailSafe] 🔴 ATTERRAGGIO DI EMERGENZA ATTIVATO")
        imposta_modalita(self.master, 'QLAND')


# ==============================================================================
# SEZIONE 7 – CONTROLLO GIMBAL E PAYLOAD
# ==============================================================================
# Il gimbal è controllato tramite MAVLink con comandi MOUNT_CONTROL
# o il protocollo MAVLink Camera Protocol (per sistemi più nuovi).

def controlla_gimbal(master, pitch_deg: float, roll_deg: float = 0, yaw_deg: float = 0):
    """
    Controlla il puntamento del gimbal stabilizzato.

    Parametri:
        pitch_deg: angolo di beccheggio (-90° = giù, 0° = orizzontale)
        roll_deg:  rollio (tipicamente 0°)
        yaw_deg:   imbardata relativa al drone (0° = avanti)

    Uso tipico per ricognizione:
        controlla_gimbal(master, -45)   # Camera puntata a 45° verso il basso
        controlla_gimbal(master, -90)   # Camera verticale (nadir)
        controlla_gimbal(master, 0)     # Camera orizzontale (sorveglianza)
    """
    master.mav.mount_control_send(
        master.target_system,
        master.target_component,
        int(pitch_deg * 100),   # MAVLink usa centidegrees
        int(roll_deg  * 100),
        int(yaw_deg   * 100),
        0                       # save position: 0=no, 1=yes
    )
    print(f"[Gimbal] Puntamento: pitch={pitch_deg}° roll={roll_deg}° yaw={yaw_deg}°")


def avvia_streaming_video(indirizzo_gcs: str = '192.168.1.100', porta: int = 5600):
    """
    Avvia lo streaming video verso il GCS.

    In produzione, questo è gestito da un link RF separato (2.4/5.8 GHz)
    con latenza < 100ms come specificato nel progetto.

    Per testing in laboratorio: GStreamer via UDP è lo standard ArduPilot.
    Il Raspberry Pi (o companion computer) esegue:

        gst-launch-1.0 \\
          v4l2src device=/dev/video0 \\
          ! video/x-h264,width=1920,height=1080,framerate=30/1 \\
          ! h264parse ! rtph264pay \\
          ! udpsink host={indirizzo_gcs} port={porta}

    Sul GCS (ricezione in QGroundControl o VLC):
        udp://@:{porta}
    """
    gst_cmd = (
        f"gst-launch-1.0 "
        f"v4l2src device=/dev/video0 "
        f"! video/x-h264,width=1280,height=720,framerate=30/1 "
        f"! h264parse ! rtph264pay config-interval=1 "
        f"! udpsink host={indirizzo_gcs} port={porta}"
    )
    print(f"[Video] Comando GStreamer:\n  {gst_cmd}")
    print(f"[Video] Per ricevere: apri VLC → udp://@:{porta}")

    # In Python (companion computer):
    # import subprocess
    # subprocess.Popen(gst_cmd.split())


# ==============================================================================
# SEZIONE 8 – ESEMPIO OPZIONE B: MAVSDK-Python (async/await)
# ==============================================================================
# MAVSDK è un'alternativa moderna a pymavlink.
# Vantaggi: sintassi pulita, async/await, più semplice da usare.
# Svantaggi: meno controllo di basso livello rispetto a pymavlink.
#
# Installazione: pip install mavsdk

"""
ESEMPIO MAVSDK (da eseguire separatamente):

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def missione_sdcr_mavsdk():
    # Connessione
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Attesa connessione e GPS lock
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connesso!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("GPS pronto")
            break

    # Crea waypoint per missione VTOL
    mission_items = [
        MissionItem(
            latitude_deg=45.0703,   # Latitudine Torino (esempio)
            longitude_deg=7.6869,   # Longitudine Torino
            relative_altitude_m=100,
            speed_m_s=18,
            is_fly_through=True,
            gimbal_pitch_deg=-45,   # Camera puntata verso il basso
            gimbal_yaw_deg=0,
            camera_action=MissionItem.CameraAction.NONE,
        ),
        # ... altri waypoint ...
    ]

    mission_plan = MissionPlan(mission_items)

    # Upload e avvio missione
    await drone.mission.upload_mission(mission_plan)
    await drone.action.arm()
    await drone.mission.start_mission()

    # Monitor progresso
    async for progress in drone.mission.mission_progress():
        print(f"Waypoint {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Missione completata!")
            break

    # RTL automatico
    await drone.action.return_to_launch()

asyncio.run(missione_sdcr_mavsdk())
"""


# ==============================================================================
# SEZIONE 9 – MAIN: FLUSSO OPERATIVO COMPLETO
# ==============================================================================

def main():
    """
    Flusso operativo completo per una missione di ricognizione SDCR.

    Sequenza:
        1. Connessione al FC via RF/UDP
        2. Connessione LoRa per telemetria ridondante
        3. Avvio monitor fail-safe
        4. Caricamento missione
        5. Armamento e decollo VTOL
        6. Esecuzione missione autonoma
        7. Monitoraggio continuo
        8. Atterraggio VTOL e disarmamento

    Per test senza hardware fisico: usare ArduPilot SITL (Software In The Loop)
        ./ArduPlane -S --quadplane --model quadplane
        (simula un VTOL QuadPlane in ambiente virtuale)
    """
    print("=" * 60)
    print("  SDCR ROBOTICS – Sistema di Controllo GCS")
    print("=" * 60)

    # ── 1. Connessione drone ──────────────────────────────────────────────────
    master = connetti_drone(CONNECTION_STRING, BAUD_RATE)

    # ── 2. Connessione LoRa ───────────────────────────────────────────────────
    lora = GestoreLoRa(porta='/dev/ttyS1', baud=9600)
    lora_ok = lora.connetti()  # Prosegui anche senza LoRa (canale ridondante)

    # ── 3. Avvio fail-safe ────────────────────────────────────────────────────
    failsafe = GestoreFailSafe(master, lora if lora_ok else None)
    failsafe.avvia()

    # ── 4. Avvio streaming video ──────────────────────────────────────────────
    avvia_streaming_video(indirizzo_gcs='192.168.1.100', porta=5600)

    # ── 5. Puntamento gimbal ──────────────────────────────────────────────────
    controlla_gimbal(master, pitch_deg=-45)  # 45° verso il basso (ricognizione)

    # ── 6. Definizione waypoint missione ─────────────────────────────────────
    # Esempio: pattugliamento area periurbana (coordinate fittizie)
    # Format: (latitudine, longitudine, altitudine_m, velocità_ms)
    waypoints_missione = [
        (45.07100, 7.69000, 100, 18),   # WP1: zona nord, quota 100m, 18 m/s
        (45.07200, 7.69500, 150, 18),   # WP2: sorvolo edificio, quota 150m
        (45.07000, 7.69800, 100, 15),   # WP3: zona est, rallentamento
        (45.06800, 7.69300,  80, 15),   # WP4: zona sud, quota bassa
        (45.07000, 7.69000, 100, 18),   # WP5: rientro verso home
    ]

    # ── 7. Caricamento missione ───────────────────────────────────────────────
    if not crea_missione_ricognizione(master, waypoints_missione):
        print("[Main] ✗ Caricamento missione fallito. Abort.")
        return

    # ── 8. Armamento in modalità QHOVER (hover VTOL) ─────────────────────────
    imposta_modalita(master, 'QHOVER')
    time.sleep(1)

    if not arma_drone(master):
        print("[Main] ✗ Armamento fallito. Abort.")
        return

    # ── 9. Avvio missione in AUTO ─────────────────────────────────────────────
    time.sleep(2)
    avvia_missione(master)  # Imposta AUTO → il drone esegue VTOL takeoff → WP → VTOL land

    # ── 10. Monitor durante la missione ──────────────────────────────────────
    print("[Main] Missione in esecuzione. Premi Ctrl+C per interrompere.")
    print("[Main] Telemetria in tempo reale:\n")

    try:
        while True:
            tel = leggi_telemetria(master)

            # Trasmissione telemetria via LoRa ogni ciclo
            if lora_ok:
                lora.invia_pacchetto_telemetria(tel)

                # Controlla eventuali comandi in arrivo via LoRa
                cmd = lora.ricevi_comandi()
                if cmd:
                    print(f"\n[Main] Comando ricevuto via LoRa: {cmd}")
                    if cmd.get('tipo') == 'RTL':
                        imposta_modalita(master, 'QRTL')
                    elif cmd.get('tipo') == 'MODE':
                        imposta_modalita(master, cmd.get('valore', 'QLOITER'))

            # Stampa telemetria
            print(
                f"[TEL] GPS:{tel['gps_lat']:.5f},{tel['gps_lon']:.5f} "
                f"Alt:{tel['gps_alt']:.0f}m | "
                f"Batt:{tel['batteria_pct']}% | "
                f"V:{tel['velocita']:.1f}m/s | "
                f"Hdg:{tel['heading']}° | "
                f"Modo:{tel['modalita']}"
            )
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[Main] Interruzione operatore → RTL attivato")
        imposta_modalita(master, 'QRTL')

    finally:
        # ── Cleanup ──────────────────────────────────────────────────────────
        failsafe.ferma()
        if lora_ok:
            lora.disconnetti()
        print("[Main] Sistema GCS terminato")


# ==============================================================================
# DIPENDENZE E NOTE DI INSTALLAZIONE
# ==============================================================================
"""
INSTALLAZIONE AMBIENTE DI SVILUPPO
====================================

  # Crea ambiente virtuale Python
  python3 -m venv venv_sdcr
  source venv_sdcr/bin/activate   # Linux/Mac
  venv_sdcr\\Scripts\\activate     # Windows

  # Installa dipendenze
  pip install pymavlink mavsdk pyserial

  # Per crittografia LoRa AES-128
  pip install pycryptodome

  # Per streaming video (opzionale, via GStreamer Python bindings)
  pip install PyGObject  # solo Linux


TEST SENZA HARDWARE (ArduPilot SITL)
======================================

  # Installa ArduPilot SITL
  git clone https://github.com/ArduPilot/ardupilot
  cd ardupilot && git submodule update --init --recursive

  # Avvia simulatore QuadPlane VTOL
  cd ArduPlane
  ../Tools/autotest/sim_vehicle.py -v ArduPlane --quadplane --console --map

  # Collegati con questo script (default UDP 14550)
  python sdcr_drone_control.py


PARAMETRI ArduPilot VTOL CONSIGLIATI
=======================================

  Q_ENABLE     = 1          # Abilita modalità QuadPlane
  Q_FRAME_TYPE = 1          # Configurazione X (4 motori)
  ARSPD_FBW_MIN = 12        # Velocità minima ala fissa (m/s)
  ARSPD_FBW_MAX = 22        # Velocità massima ala fissa (m/s)
  Q_TRANSITION_MS = 8000    # Tempo transizione VTOL→crociera (ms)
  BATT_FS_LOW_ACTION = 2    # Fail-safe batteria bassa → RTL
  BATT_FS_CRT_ACTION = 1    # Fail-safe batteria critica → LAND
  FS_LONG_ACTN = 2          # Fail-safe perdita RC → RTL
"""

if __name__ == '__main__':
    main()
