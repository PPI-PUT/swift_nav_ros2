// Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BUILD_SBP_SERIAL_READER_HPP
#define BUILD_SBP_SERIAL_READER_HPP

#include <libserialport.h>
#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>

namespace swift_nav {
    class SbpSerialReader : public sbp::IReader {
    public:
        SbpSerialReader() = default;
        SbpSerialReader(const char *serial_name, int baudrate);
        s32 read(u8 *buffer, u32 buffer_length) override;
        ~SbpSerialReader();

    private:
        const char *m_serial_name;
        int baudrate;
        struct sp_port *m_piksi_port;
    };
}  // namespace swift_nav
#endif //BUILD_SBP_SERIAL_READER_HPP
