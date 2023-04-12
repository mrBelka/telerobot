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
		return 0;
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
	    	return 0;
	    }
        }

        template<typename T, typename = std::enable_if_t<sizeof(T) == sizeof(char)>>
        [[maybe_unused]] size_t ReciveTimeouted(T *data, int count,std::chrono::milliseconds expiryTime)
        {
            asio::steady_timer timer(m_io_service);
            std::optional<asio::error_code> timerResult = std::nullopt;
            timer.expires_from_now(expiryTime);

            timer.async_wait([&timerResult](const asio::error_code& error)
                             {
                                 timerResult = error;
                             }
            );

            size_t bytesReaded = 0;
            std::optional<asio::error_code> readResult;

            m_serial_port->async_read_some(
                    asio::buffer(data,count),
                    [&readResult,&bytesReaded](const asio::error_code& error, size_t readed)
                    {
                        readResult = error;
                        bytesReaded = readed;
                    }
            );

            m_io_service.run_one();


            if(readResult)
            {
                timer.cancel();
                return bytesReaded;
            }

            if(timerResult)
                m_serial_port->cancel();

            std::cerr << "Read Timeout" << std::endl;
            //throw sv::Exception("Read Timed out",__FILE__,__LINE__);
        }



    private:
        std::string m_port;
        size_t m_baudRate;
        asio::io_service m_io_service;
        asio::serial_port* m_serial_port;
    };
}

