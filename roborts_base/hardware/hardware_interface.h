//
// Created by cxn on 19-2-19.
//

#ifndef PROJECT_HARDWARE_INTERFACE_H
#define PROJECT_HARDWARE_INTERFACE_H


#include <cstdint>

namespace leonard_serial_common {
    /**
     * @brief Abstract class for hardware as an interface
     */
    class HardwareInterface {
    public:
        HardwareInterface() {};

        virtual ~HardwareInterface() = default;

        virtual bool Init() = 0;

        virtual int Read(uint8_t *buf, int len) = 0;

        virtual int Write(const uint8_t *buf, int len) = 0;
    };


}

#endif //PROJECT_HARDWARE_INTERFACE_H
