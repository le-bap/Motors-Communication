#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#define DEVICENAME "/dev/ttyUSB2"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 2.0
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define TORQUE_ENABLE 1

class MotorsCommunication : public rclcpp::Node
{
public:
    MotorsCommunication() : Node("motors_communication")
    {
        // Inicializa comunicação
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a porta!");
            return;
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao configurar baudrate!");
            return;
        }

        int id, pos;
        std::cout << "Digite o ID do motor: ";
        std::cin >> id;
        std::cout << "Digite a posição desejada: ";
        std::cin >> pos;

        if (setPosition(id, pos)) {
            std::cout << "Comando enviado com sucesso!" << std::endl;
        } else {
            std::cout << "Erro ao enviar comando." << std::endl;
        }

        portHandler->closePort();
    }

private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;

    bool setPosition(int id, int pos)
    {
        uint8_t error;
        int result;

        // Ativar torque
        result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Erro ativando torque: %s", packetHandler->getTxRxResult(result));
            return false;
        } else if (error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erro no pacote ao ativar torque: %s", packetHandler->getRxPacketError(error));
            return false;
        }

        // Enviar posição
        result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pos, &error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Erro de comunicação ao enviar posição: %s", packetHandler->getTxRxResult(result));
            return false;
        } else if (error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erro no pacote ao enviar posição: %s", packetHandler->getRxPacketError(error));
            return false;
        }

        return true;
    }

};

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsCommunication>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
