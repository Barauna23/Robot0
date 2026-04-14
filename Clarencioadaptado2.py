from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ================= DEFINIÇÕES =================
hub = PrimeHub()

ME = Motor(Port.A, Direction.CLOCKWISE)   # Esquerdo
MD = Motor(Port.B, Direction.COUNTERCLOCKWISE)  # Direito
garra = Motor(Port.C) # garra
SE = ColorSensor(Port.D)  # Esquerdo
SD = ColorSensor(Port.E)  # Direito
SF = ColorSensor(Port.F) # Frente

# ================= CONSTANTES =================
GYRO_TOL = 0.5      
GYRO_MAX = 80
GYRO_MIN = 50 # Aumentei de 20 ou 25 (não lembro) pra 50
fator_calibracao = 1.0

# ================= VARIÁVEIS GLOBAIS =================
erro = 0
correcao = 0
integral = 0
derivada = 0
last_error = 0
qnt_pretos = 0
qnt_preto_ambos = 0 # Adicionei para fazer a linha de chegada e o final do resgate
qnt_verde = 0
estado = "seguir"

def is_verde(hsv):
    h, s, v = hsv
    return 100 <= h <= 199 and 30 <= s <= 70 and 30 <= v <= 80

def is_prata(hsv): # melhorar o def prata, ta cagadão
    h, s, v = hsv
    return 200 <= h <= 215 and 20 <= s <= 30 and 50 <= v <= 70

def is_preto(hsv): # adicionei um def preto, pra linha de chegada e pro treco da arena
    h, s, v = hsv
    return 200 <= h <= 260 and 10 <= s <= 20 and 5 <= v <= 15

def is_branco(hsv): # talvez eu use esse para resolver o problema dos verdes
    h, s, v = hsv
    return ? <= h <= ? and ? <= s <= ? and ? <= v <= ?

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
    global erro, correcao
    
    print("Giro")

    integral = 0
    last_error = 0
    
    kp = 3.65 # alterei o kp de 3 para 3.35, vê se precisa aumentar o resto
    ki = 0.001
    kd = 0.20
    
    hub.imu.reset_heading(0)
    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    while abs(erro_angular(alvo, hub.imu.heading())) > GYRO_TOL:
        erro = erro_angular(alvo, hub.imu.heading())
        integral += erro
        derivada = erro - last_error
        correcao = (erro * kp) + (integral * ki) + (derivada * kd)
        last_error = erro
        
        potencia = max(GYRO_MIN, min(GYRO_MAX, abs(correcao)))
        
        if erro > 0:
            ME.dc(potencia)
            MD.dc(-potencia)
        else:
            ME.dc(-potencia)
            MD.dc(potencia)
        
        wait(8)

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
    
    print("Giro concluído")
    wait(200)

# ================= REALINHAMENTO FORTE =================
def realinhar_linha(): # perguntar pra algum veterano como funciona essa parte para que eu possa alterar e deixar mais realinhado
    global estado, integral
    
    # Procura linha
    for i in range(120):  # Aumentado para melhor procura
        ref_esq = SE.reflection()
        ref_dir = SD.reflection()
        
        erro_linha = ref_dir - ref_esq
        correcao_linha = erro_linha * 25
        
        vel_esq = 25 + correcao_linha
        vel_dir = 25 - correcao_linha
        
        vel_esq = max(min(vel_esq, 45), 0)
        vel_dir = max(min(vel_dir, 45), 0)
        
        ME.dc(vel_esq)
        MD.dc(vel_dir)
        wait(10)
    
    ME.brake()
    MD.brake()
    wait(300)
    
    integral = 0
    estado = "seguir"  # Voltando a linha do seguidor
    print("Realinhando")

# ==================== GYRO MOVE (GM) ====================
def GM(rotacoes, velocidade_final, angle, reverso=False):
    global erro, correcao
    
    integral = 0
    last_error = 0
    ME.reset_angle(0)
    MD.reset_angle(0)
    wait(50)
    
    alvo_heading = angle
    rotacoes_corrigidas = rotacoes * fator_calibracao
    velocidade_final = abs(velocidade_final)
    velocidade_atual = 40
    sinal = -1 if reverso else 1
    corr_sinal = -1 if reverso else 1
    
    ME.dc(sinal * (velocidade_atual - corr_sinal * correcao))
    MD.dc(sinal * (velocidade_atual + corr_sinal * correcao))
    
    while True:
        rot_esq = abs(ME.angle() / 360)
        rot_dir = abs(MD.angle() / 360)
        rot_media = (rot_esq + rot_dir) / 2
        
        if rot_media >= abs(rotacoes_corrigidas):
            break
            
        erro = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180
        correcao = erro * 8.9 + (erro - last_error) * 0.05
        last_error = erro
        
        if rot_media < 0.3 * abs(rotacoes_corrigidas):
            if velocidade_atual < velocidade_final:
                velocidade_atual += 2
        
        if reverso:
            pot_esq = sinal * (velocidade_atual - correcao)
            pot_dir = sinal * (velocidade_atual + correcao)
        else:
            pot_esq = sinal * (velocidade_atual + correcao)
            pot_dir = sinal * (velocidade_atual - correcao)
            
        ME.dc(pot_esq)
        MD.dc(pot_dir)
        wait(20)
    
    ME.brake()
    MD.brake()
    wait(100)
    print("GM concluído")

# ================= SEGUIDOR ADAPTATIVO =================
def seguidor_adaptativo(velocidade):
    global qnt_pretos, erro, correcao, integral, estado
    
    qnt_pretos = 0
    integral = 0
    estado = "seguir"
    
    print("Seguidor:", velocidade)
    
    while qnt_pretos < 10000000:
        ref_esq = SE.reflection()
        ref_dir = SD.reflection()
        hsv_dir = SD.hsv()
        hsv_esq = SE.hsv()

       # Eu fiz essa ´parte por que o Matheus disse que ele pode mudar as possições do verde a qualquer momento
        if is_verde(hsv_dir): 
            GM(0.6, 30, 0, reverso=True) # indo para trás para verificar
            if is_branco(hsv_dir) and is_branco(hsv_esq): # verificando se é branco ou preto
                estado = "giro"
                GM(0.6, 30, 0, reverso=False) # voltando a possição inical (precisa?)
                gyro_turn(90)
                realinhar_linha()
                continue
            elif is_preto(hsv_dir) and is_preto(hsv_esq):
                estado = "seguir"
                realinhar_linha()
                continue


        elif is_verde(hsv_esq): 
            GM(0.6, 30, 0, reverso=True) # indo para trás para verificar
            if is_branco(hsv_dir) and is_branco(hsv_esq): # verificando se é branco ou preto
                estado = "giro"
                GM(0.6, 30, 0, reverso=False) # voltando a possição inical (precisa?)
                gyro_turn(-90)
                realinhar_linha()
                continue
            elif is_preto(hsv_dir) and is_preto(hsv_esq):
                estado = "seguir"
                realinhar_linha()
                continue


        elif is_verde(hsv_dir) and is_verde(hsv_esq):
            estado = "giro"
            gyro_turn(180)
            realinhar_linha()
            continue

        elif is_prata(hsv_dir) or is_prata(hsv_esq):
            print("Prata")
            GM(0.65, velocidade, 0, reverso=False)
            ME.brake()
            MD.brake()
            # break

        elif is_preto(hsv_dir) and is_preto(hsv_esq):
            qnt_preto_ambos += 1

            if qnt_preto_ambos == 1:
                print("1 vez preto")
                realinhar_linha() # Ver se funciona, socorro KKKKKK
                continue

            elif qnt_preto_ambos == 2:
                print("2 vez preto")
                continue

            elif qnt_preto_ambos == 3:
                print("3 vez preto")
                ME.break()
                MD.break()

        # Só para quando o estado estiver no seguir
        if estado != "seguir":
            ME.brake()
            MD.brake()
            wait(10)
            continue

        # Curvas acentuadas
        if ref_esq < 20 and ref_dir > 65:
            ME.dc(-velocidade)
            MD.dc(velocidade)
            wait(5)
            continue
        elif ref_dir < 20 and ref_esq > 65:
            ME.dc(velocidade)
            MD.dc(-velocidade)
            wait(5)
            continue

        # PID SEGUINDOR (SEMPRE EXECUTA)
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
    seguidor_adaptativo(70)

main()