#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Definições para comunicação entre motor 
#define DEVICENAME "/dev/ttyUSB2"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 2.0
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define TORQUE_ENABLE 1

// Cria a classe 
class MotorsCommunication : public rclcpp::Node
{
public: // Cria o nó motors_communication
    MotorsCommunication() : Node("motors_communication")
    {
        // Inicializa comunicação
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Tenta abrir a porta
        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a porta");
            return;
        }

        // Tenta configurar o baudrate
        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao configurar baudrate");
            return;
        }

        // Recebe os valores do usuário
        int id, pos;
        std::cout << "Digite o ID do motor: ";
        std::cin >> id;
        std::cout << "Digite a posição desejada: ";
        std::cin >> pos;

        // Chama a função setPosition com os valores recebidos
        if (setPosition(id, pos)) {
            RCLCPP_INFO(this->get_logger(), "Comando enviado com sucesso!!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Erro ao enviar o comando");
        }

        portHandler->closePort();
    }

private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;

    // Função que fará a mudança na posição do motors
    bool setPosition(int id, int pos)
    {
        uint8_t error;
        int result;

       // Ativa o torque do motor 
        result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &error);
        if (result != COMM_SUCCESS || error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao ativar torque.");
            return false;
        }

        // Envia a posição desejada para o motor
        result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pos, &error);
        if (result != COMM_SUCCESS || error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao enviar posição.");
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
