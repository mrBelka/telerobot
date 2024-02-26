#include <SerialConnector.hpp>
#include <exception>
#include <iostream>

namespace robot::io
{

    SerialConnector::SerialConnector(const std::string& port, size_t baudRate)
            : m_port(port), m_baudRate(baudRate)
    {
        m_serial_port = new asio::serial_port(m_io_service);
    }

    SerialConnector::~SerialConnector() noexcept
    {
        if (m_serial_port == nullptr)
            return;
        Close();
        delete m_serial_port;
    }

    void SerialConnector::Open()
    {
        try
        {
            if (m_serial_port->is_open())
                m_serial_port->close();

            m_serial_port->open(m_port);
            if (!m_serial_port->is_open())
                throw std::runtime_error("port wasn`t opened");

            m_serial_port->set_option(asio::serial_port_base::baud_rate(m_baudRate));
            m_serial_port->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
            m_serial_port->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            m_serial_port->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            m_serial_port->set_option(asio::serial_port_base::character_size(8));
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << std::endl;
        }
    }

    bool SerialConnector::IsOpen()
    {
        return m_serial_port->is_open();
    }

    void SerialConnector::Close()
    {
        try {
            m_serial_port->close();
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << std::endl;
        }
    }

}
