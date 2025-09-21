import serial
import time
import random

SERIAL_PORT = "/dev/tty.usbmodem11301"
BAUD_RATE = 115200

NUM_EIXOS = 4  # (intake_pressure, intake_temperature, rpm, speed)
NUM_AMOSTRAS = 1 

TOTAL_VALORES_ESPERADOS = NUM_EIXOS * NUM_AMOSTRAS


def gerar_dados_amostra(total_valores):
    """
    Gera uma lista de floats de exemplo no formato esperado.
    Formato: [p1, t1, r1, s1, p2, t2, r2, s2, ...]
    """
    print(
        f"Gerando {total_valores} valores de amostra ({NUM_AMOSTRAS} amostras de {NUM_EIXOS} eixos)..."
    )
    lista_de_dados = []
    for _ in range(NUM_AMOSTRAS):
        # [intake_pressure, intake_temperature, rpm, speed]
        pressure = round(random.uniform(50.0, 110.0), 2)
        temp = round(random.uniform(20.0, 70.0), 2)
        rpm = round(random.uniform(1000.0, 3000.0), 2)
        speed = round(random.uniform(40.0, 100.0), 2)
        lista_de_dados.extend([pressure, temp, rpm, speed])

    return lista_de_dados


def formatar_e_enviar(ser, dados):
    """
    Formata a lista de dados como string, envia e lê a resposta.
    """

    data_string = ",".join(map(str, dados))
    data_string += "\n"

    print(f"\nEnviando {len(dados)} valores...")

    ser.write(data_string.encode("utf-8"))

    try:
        resposta = ser.read_until(b"\r\n\r\n")
        print("--- Resposta do Arduino ---")
        print(resposta.decode("utf-8", errors="ignore"))
        print("---------------------------")

        msg_aguardando = ser.read_until(
            b"Aguardando dados na porta serial...\r\n"
        )
        print(msg_aguardando.decode("utf-8", errors="ignore").strip())

    except serial.SerialTimeoutException:
        print("Timeout! O Arduino não respondeu a tempo (espera global de 10s).")


def main():
    print("--- Script de Envio Serial para Edge Impulse ---")
    print(f"Este script tentará se conectar a {SERIAL_PORT} em 3 segundos...")
    print("------------------------------------------------------")
    time.sleep(3)

    try:
        # Abrir a porta serial
        # ATUALIZAÇÃO: Timeout global aumentado para 10 segundos.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
        print(f"Conectado a {SERIAL_PORT} com sucesso.")

        print("Aguardando Arduino inicializar (2s)...")
        time.sleep(2)

        ser.flushInput()

        while True:
            dados = gerar_dados_amostra(TOTAL_VALORES_ESPERADOS)
            print(f"Dados gerados: {dados[:8]}... (total {len(dados)} valores)")

            # Enviar dados e ler resposta
            formatar_e_enviar(ser, dados)

            # Esperar antes de enviar a próxima inferência
            print("Aguardando 5 segundos antes de enviar novos dados...")
            time.sleep(5)

    except serial.SerialException as e:
        print(f"\n[ERRO] Não foi possível abrir a porta serial {SERIAL_PORT}.")
        print(f"Detalhe: {e}")
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário. Fechando porta serial.")
    except Exception as e:
        print(f"Ocorreu um erro inesperado: {e}")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Porta serial fechada.")


if __name__ == "__main__":
    main()