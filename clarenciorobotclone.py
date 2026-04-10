# ================= IMPORTS =================
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.tools import wait

# ================= DEFINIÇÕES =================
hub = PrimeHub()

ME = Motor(Port.A, Direction.CLOCKWISE)   # Esquerdo
MD = Motor(Port.B, Direction.COUNTERCLOCKWISE)  # Direito

SE = ColorSensor(Port.D)  # Esquerdo
SD = ColorSensor(Port.E)  # Direito

# ================= CONSTANTES =================
GYRO_TOL = 1.5      # Mais preciso
GYRO_MAX = 85
GYRO_MIN = 25

# ================= VARIÁVEIS GLOBAIS =================
erro = 0
correcao = 0
integral = 0
derivada = 0
last_error = 0
qnt_pretos = 0
estado = "seguir"  # "seguir", "giro", "realinhar"

def is_verde(hsv):
    h, s, v = hsv
    return 100 <= h <= 199 and 30 <= s <= 70 and 30 <= v <= 80

# ================= PID ADAPTATIVO =================
def PID_adaptativo(erro):
    global integral, derivada, correcao, last_error
    
    integral += erro
    derivada = erro - last_error

    kp_base = 290
    kp_max = 330
    kp = kp_base + (kp_max - kp_base) * (abs(erro) / (1 + abs(erro)))**0.3

    kd_base = 9
    kd_max = 31
    kd = kd_base + (kd_max - kd_base) * (abs(erro) / (1 + abs(erro)))**0.5

    ki = 0
    correcao = (erro * kp) + (integral * ki) + (derivada * kd)
    last_error = erro
    integral = max(min(integral, 30), -30)

# ================= GYRO TURN FORTE =================
def gyro_turn(graus):
    global erro, correcao, estado
    
    print("INICIANDO GIRO...")
    
    # FREIO TOTAL ANTES DO GIRO
    ME.brake()
    MD.brake()
    wait(300)
    
    # Reset PID
    integral = 0
    last_error = 0
    
    kp = 3  # Mais agressivo
    ki = 0.001
    kd = 0.20
    
    hub.imu.reset_heading(0)
    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    # GIRO PRINCIPAL
    while abs(erro_angular(alvo, hub.imu.heading())) > GYRO_TOL:
        erro = erro_angular(alvo, hub.imu.heading())
        integral += erro
        derivada = erro - last_error
        correcao = (erro * kp) + (integral * ki) + (derivada * kd)
        last_error = erro
        
        potencia = max(GYRO_MIN, min(GYRO_MAX, abs(correcao)))
        
        if erro > 0:  # Direita
            ME.dc(potencia)
            MD.dc(-potencia)
        else:  # Esquerda
            ME.dc(-potencia)
            MD.dc(potencia)
        
        wait(8)

    # FREIO FORTE
    ME.brake()
    MD.brake()
    wait(500)

    # CORREÇÃO FINA
    erro_residual = erro_angular(alvo, hub.imu.heading())
    if abs(erro_residual) > 0.2:
        if erro_residual > 0:
            ME.dc(12)
            MD.dc(-12)
        else:
            ME.dc(-12)
            MD.dc(12)
        wait(100)
        ME.brake()
        MD.brake()
    
    print("GIRO CONCLUÍDO! Realinhando...")
    wait(300)

# ================= REALINHAMENTO FORTE =================
def realinhar_linha():
    global estado
    
    print("REALINHANDO...")
    
    # FREIO
    ME.brake()
    MD.brake()
    wait(200)
    
    # PROCURA LINHA DEVAGAR (1 segundo)
    for i in range(100):
        ref_esq = SE.reflection()
        ref_dir = SD.reflection()
        
        erro_linha = ref_dir - ref_esq
        correcao_linha = erro_linha * 25
        
        vel_esq = 20 + correcao_linha
        vel_dir = 20 - correcao_linha
        
        vel_esq = max(min(vel_esq, 40), 0)  # SÓ para frente
        vel_dir = max(min(vel_dir, 40), 0)
        
        ME.dc(vel_esq)
        MD.dc(vel_dir)
        wait(10)
    
    # FREIO FINAL
    ME.brake()
    MD.brake()
    wait(400)
    
    integral = 0  # Reset PID
    print("REALINHADO! Continuando...")
    estado = "seguir"

# ================= SEGUIDOR ADAPTATIVO =================
def seguidor_adaptativo(velocidade):
    global qnt_pretos, erro, correcao, integral, estado
    
    qnt_pretos = 0
    integral = 0
    estado = "seguir"
    
    while qnt_pretos < 10000000:
        ref_esq = SE.reflection()
        ref_dir = SD.reflection()
        hsv_dir = SD.hsv()
        hsv_esq = SE.hsv()

        # ✅ VERDE DETECTADO
        if is_verde(hsv_dir) or is_verde(hsv_esq):
            print("VERDE! PARANDO...")
            estado = "giro"
            gyro_turn(90)      # GIRO 90°
            realinhar_linha()  # REALINHA
            continue

        # SÓ SEGUE SE ESTIVER NO ESTADO "seguir"
        if estado != "seguir":
            ME.brake()
            MD.brake()
            wait(10)
            continue

        # CURVAS ACENTUADAS
        if ref_esq < 20 and ref_dir > 65:
            ME.dc(-50)
            MD.dc(velocidade)
            wait(5)
            continue
        elif ref_dir < 20 and ref_esq > 65:
            ME.dc(velocidade)
            MD.dc(-50)
            wait(5)
            continue

        # PID NORMAL
        if (ref_esq + ref_dir) == 0:
            erro = 0
        else:
            erro = (ref_dir - ref_esq) / (ref_esq + ref_dir)

        PID_adaptativo(erro)

        vel_esq = max(min(velocidade - correcao, 90), -90)
        vel_dir = max(min(velocidade + correcao, 90), -90)
        
        ME.dc(vel_esq)
        MD.dc(vel_dir)
        
        if ref_esq < 25 or ref_dir < 25:
            qnt_pretos += 1

        wait(8)

# ================= MAIN =================
def main():
    gyro_turn(90)

main()