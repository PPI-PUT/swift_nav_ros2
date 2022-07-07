//
// Created by tomek on 7/7/22.
//

#ifndef BUILD_SBP_SERIAL_READER_HPP
#define BUILD_SBP_SERIAL_READER_HPP

#include <libserialport.h>
#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

namespace swift_nav {
    class SbpSerialReader : public sbp::IReader {
    public:
        SbpSerialReader() = default;
        SbpSerialReader(const char *serial_name);
        s32 read(u8 *buffer, u32 buffer_length) override;
        ~SbpSerialReader();

    private:
        const char *m_serial_name;
        struct sp_port *m_piksi_port;
    };
}  // namespace swift_nav_driver
#endif //BUILD_SBP_SERIAL_READER_HPP
