#include <ModbusMaster.hpp>

namespace robot::protocol
{

    ModbusMaster::ModbusMaster(const std::string& port, size_t baudrate) :
            m_port(port), m_baudrate(baudrate)
    {
        m_sc = std::make_unique<io::SerialConnector>(m_port, m_baudrate);
    }

    void ModbusMaster::Setup()
    {
        m_sc->Open();
    }

    int16_t ModbusMaster::ReadAnalogInput(uint8_t addr, uint16_t reg)
    {
        std::vector<int16_t> data = ReadAnalogInput(addr, reg, 1);
        return data[0];
    }

    std::vector<int16_t> ModbusMaster::ReadAnalogInput(uint8_t addr, uint16_t reg, uint16_t count)
    {
        static int step = 0;
        reg -= 1;
        uint8_t lreg = reg & 0xFF;
        uint8_t hreg = (reg >> 8) & 0xFF;
        uint8_t lcount = count & 0xFF;
        uint8_t hcount = (count >> 8) & 0xFF;
        uint8_t buf[8] = {addr, 0x04, hreg, lreg, hcount, lcount};
        uint16_t crc = Modbus_crc16(buf, 6);
        buf[6] = crc & 0xFF;
        buf[7] = (crc >> 8) & 0xFF;

        //for(int i=0;i<8;i++)
        //    printf("%02hhX ", buf[i]);
        //printf("\n");

        std::vector<int16_t> result;
        if (step < 100) {
            step++;
            result.resize(4);
            return result;
        }

        m_sc->Send(buf, 8);

        uint8_t rbuf[512];

        size_t n = m_sc->Receive(rbuf, 2*count + 5);
        if(n > 0) {

            //for (int i = 0; i < 2*count + 5; i++)
            //    printf("%02hhX ", rbuf[i]);
            //printf("\n");

            crc = Modbus_crc16(rbuf, 2*count + 3);
            uint16_t r_crc = rbuf[2*count+3] | (rbuf[2*count+4] << 8);
            //std::cout << crc << " " << r_crc << std::endl;
	    if(crc == r_crc)
            {
		printf("Receive data\n");
     		for(int i=0;i<2*count;i+=2)
                   result.push_back(rbuf[i+4] | (rbuf[i+3] << 8));
            }
            else
	    {
	        printf("CRC error\n");
            }
        }

        return result;
    }

    bool ModbusMaster::WriteSingleAnalogOutput(uint8_t addr, uint16_t reg, uint16_t value)
    {
        reg -= 1;
        uint8_t lreg = reg & 0xFF;
        uint8_t hreg = (reg >> 8) & 0xFF;
        uint8_t lvalue = value & 0xFF;
        uint8_t hvalue = (value >> 8) & 0xFF;
        uint8_t wbuf[8] = {addr, 0x06, hreg, lreg, hvalue, lvalue};
        uint16_t crc = Modbus_crc16(wbuf, 6);
        wbuf[6] = crc & 0xFF;
        wbuf[7] = (crc >> 8) & 0xFF;

        //for(int i=0;i<8;i++)
        //    printf("%02hhX ", wbuf[i]);
        //printf("\n");

        m_sc->Send(wbuf, 8);

        uint8_t rbuf[8];
        size_t n = m_sc->Receive(rbuf, 8);

        //std::cout << n << " " << (int)rbuf[0] << std::endl;

        if(n == 8) {

            //for (int i = 0; i < 8; i++)
            //    printf("%02hhX ", rbuf[i]);
            //printf("\n");

            crc = Modbus_crc16(rbuf, 6);
            uint16_t r_crc = rbuf[6] | (rbuf[7] << 8);
            //std::cout << crc << " " << r_crc << std::endl;
        }

        return true;
    }

    bool ModbusMaster::WriteMultiAnalogOutput(uint8_t addr, uint16_t reg, const std::vector<uint16_t>& values)
    {
        reg -= 1;
        uint8_t lreg = reg & 0xFF;
        uint8_t hreg = (reg >> 8) & 0xFF;
        uint8_t lcount = values.size() & 0xFF;
        uint8_t hcount = (values.size() >> 8) & 0xFF;
        uint8_t wbuf[512] = {addr, 0x10, hreg, lreg, hcount, lcount, static_cast<uint8_t>(2*values.size())};

        for(uint8_t i = 0; i<values.size();i++)
        {
            wbuf[7 + 2*i] = values[i] >> 8;
            wbuf[8 + 2*i] = values[i] & 0xFF;
        }
        uint16_t crc = Modbus_crc16(wbuf, 7 + 2*values.size());
        wbuf[7 + 2*values.size()] = crc & 0xFF;
        wbuf[7 + 2*values.size() + 1] = (crc >> 8) & 0xFF;

        //for(int i=0;i<7 + 2*values.size() + 2;i++)
        //    printf("%02hhX ", wbuf[i]);
        //printf("\n");

        m_sc->Send(wbuf, 7 + 2*values.size() + 2);

        uint8_t rbuf[8];
        size_t n = m_sc->Receive(rbuf, 8);

        //std::cout << n << " " << (int)rbuf[0] << std::endl;

        if(n == 8) {

            //for (int i = 0; i < 8; i++)
            //    printf("%02hhX ", rbuf[i]);
            //printf("\n");

            crc = Modbus_crc16(rbuf, 6);
            uint16_t r_crc = rbuf[6] | (rbuf[7] << 8);
            //std::cout << crc << " " << r_crc << std::endl;
        }
	return true;
    }

    uint16_t ModbusMaster::Modbus_crc16(const unsigned char *buf, size_t len) const
    {
        uint8_t x = 0;
        uint16_t crc = 0xFFFF;

        while( len-- )
        {
            x = (*buf++) ^ crc;
            crc >>= 8;
            crc ^= m_table[x];
        }
        return crc;
    }

}
