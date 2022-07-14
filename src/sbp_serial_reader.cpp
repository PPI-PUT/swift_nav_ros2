//
// Created by tomek on 7/7/22.
//


#include <swift_nav_ros2//sbp_serial_reader.hpp>


namespace swift_nav {

    SbpSerialReader::SbpSerialReader(const char *serial_name, int baudrate) {
        this->m_serial_name = serial_name;
        this->baudrate = baudrate;
        int result = 0;

        printf("Attempting to configure the serial port...\n");

        result = sp_get_port_by_name(m_serial_name, &m_piksi_port);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot find provided serial port!\n");
            exit(EXIT_FAILURE);
        }
        printf("Description: %s\n", sp_get_port_description(m_piksi_port));
        printf("Attempting to open the serial port...\n");

        result = sp_open(m_piksi_port, SP_MODE_READ);
        if (result != SP_OK) {
            char *err = sp_last_error_message();
            fprintf(stderr, "%s\n", err);
            fprintf(stderr, "Cannot open %s for reading!\n", serial_name);
            exit(EXIT_FAILURE);
        }

        result = sp_set_baudrate(m_piksi_port, this->baudrate);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot set port baud rate!\n");
            exit(EXIT_FAILURE);
        }
        printf("Configured the baud rate...\n");

        result = sp_set_flowcontrol(m_piksi_port, SP_FLOWCONTROL_NONE);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot set flow control!\n");
            exit(EXIT_FAILURE);
        }
        printf("Configured the flow control...\n");

        result = sp_set_bits(m_piksi_port, 8);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot set data bits!\n");
            exit(EXIT_FAILURE);
        }
        printf("Configured the number of data bits...\n");

        result = sp_set_parity(m_piksi_port, SP_PARITY_NONE);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot set parity!\n");
            exit(EXIT_FAILURE);
        }

        printf("Configured the parity...\n");

        result = sp_set_stopbits(m_piksi_port, 1);
        if (result != SP_OK) {
            fprintf(stderr, "Cannot set stop bits!\n");
            exit(EXIT_FAILURE);
        }

        printf("Configured the number of stop bits... done.\n");
    }

    s32 SbpSerialReader::read(u8 *buffer, u32 buffer_length) {
        s32 result;

        result = sp_blocking_read(m_piksi_port, buffer, buffer_length, 50);
        return result;
    }

    SbpSerialReader::~SbpSerialReader() {
        sp_free_port(m_piksi_port);
    }

}  // namespace swift_nav
