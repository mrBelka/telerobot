#pragma once
#include <string>
#include <asio.hpp>
#include <iostream>
#include <optional>

namespace robot::io
{

    class SerialConnector {
    public:
        SerialConnector(const std::string& port, size_t baudRate);
        ~SerialConnector() noexcept;
        void Open();
        bool IsOpen();
        void Close();

        template<typename T, typename = std::enable_if_t<sizeof(T) == sizeof(char)>>
        size_t Send(const T *buf, size_t size)
        {
            try {
                return m_serial_port->write_some(asio::buffer(buf, size));
            }
            catch (std::exception &e) {
                std::cerr << e.what() << " " << __FILE__ << __LINE__ << std::endl;
            }
        }

        template<typename T, typename = std::enable_if_t<sizeof(T) == sizeof(char)>>
        size_t Receive(T *data, int count)
        {
            try
            {
                size_t len = m_serial_port->read_some(asio::buffer(data, count));
                return len;
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << " " << __FILE__ << __LINE__ << std::endl;
            }
        }

    private:
        std::string m_port;
        size_t m_baudRate;
        asio::io_service m_io_service;
        asio::serial_port* m_serial_port;
    };
}

