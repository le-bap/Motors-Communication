#include <iostream>
#include "dynamixel_sdk/dynamixel_sdk.h"

// Configurações básicas (ajuste para seu motor/porta)
#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 1.0
#define ADDR_GOAL_POSITION 30   // endereço do Goal Position (ex.: AX-12)

// Função simples para mover o motor
bool SetPosition(int id, int pos,
                 dynamixel::PortHandler* port,
                 dynamixel::PacketHandler* packet)
{
    int result = packet->write2ByteTxRx(port, id, ADDR_GOAL_POSITION, pos);
    return (result == COMM_SUCCESS);
}

int main()
{
    // Inicializa comunicação
    auto portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    int id, pos;
    std::cout << "Digite o ID do motor: ";
    std::cin >> id;
    std::cout << "Digite a posição desejada: ";
    std::cin >> pos;

    if (SetPosition(id, pos, portHandler, packetHandler)) {
        std::cout << "Comando enviado com sucesso!" << std::endl;
    } else {
        std::cout << "Erro ao enviar comando." << std::endl;
    }

    portHandler->closePort();
    return 0;
}
