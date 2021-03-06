#pragma once

namespace apollo {
namespace canbus {
namespace diamond {

// Steering const speed
unsigned char C1[8] = {0x0B, 0x06, 0x20, 0x00, 0x27, 0x10, 0x98, 0x9C};
// frontsteer stop
unsigned char C2[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4D, 0xA3};
// frontsteer positive
unsigned char C3[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4C, 0x60};
// frontsteer negative
unsigned char C4[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0C, 0x61};

// rear const speed
unsigned char C5[8] = {0x0C, 0x06, 0x20, 0x00, 0x27, 0x10, 0x99, 0x2B};
// rear stop
unsigned char C6[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4C, 0x14};
// rear positive
unsigned char C7[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4D, 0xD7};
// rear negative
unsigned char C8[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0D, 0xD6};
// parking_brake_frequency 100%
unsigned char C9[8] = {0x0F, 0x06, 0x10, 0x00, 0x27, 0x10, 0x96, 0x18};
// parking_brake_up
unsigned char C10[8] = {0x0F, 0x06, 0x20, 0x00, 0x00, 0x01, 0x42, 0xE4};
// parking_brake_close
unsigned char C11[8] = {0x0F, 0x06, 0x20, 0x00, 0x00, 0x05, 0x43, 0x27};

// Front steering motor
unsigned char C12[8] = {0x0D, 0x06, 0x20, 0x00, 0x27, 0x10, 0x98, 0xFA};
// Front Fan motor forward rotation 
unsigned char C13[8] = {0x0D, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4C, 0x06};
// Front Fan motor stop
unsigned char C14[8] = {0x0D, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4D, 0xC5};

// Rear steering motor
unsigned char C15[8] = {0x0E, 0x06, 0x20, 0x00, 0x27, 0x10, 0x98, 0xC9};
// Rear Fan motor forward rotation 
unsigned char C16[8] = {0x0E, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4C, 0x35};
// Rear Fan motor stop 
unsigned char C17[8] = {0x0E, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4D, 0xF6};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
