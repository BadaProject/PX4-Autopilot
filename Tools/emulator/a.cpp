#include <iostream>
#include <cstdint>

enum Request
{
    REQUEST_WRITE = 0x58,
    REQUEST_READ = 0x54,
};

uint8_t read_command[40] = {
0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54, //LSIS-XGT
0x00, 0x00, // Reserved[2]
0x00, 0x00, // PLC Info[2]
0xA0, // CPU Info
0x33, // Source of Frame
0x00, 0x00, // Invoke ID[2]
0x14, 0x00, //  // Length[2] : size of  Application Instruction : 20
0x00, // Bit0~3 : slot # of FEnet I/F module, Bit4~7
0x42, // chechsum 0~19, ascii 'B'
REQUEST_READ, 0x00, // 0x54 read
0x14, 0x00,  //Data Type[2] : 연속 읽기의 경우 0x0014
0x00, 0x00, //Reserved[2]
0x01, 0x00, // block length[2] : 0x01 (사용할 블럭의 갯수)
0x08, 0x00, // block name length[2] = 0x08 (블럭 이름의 길이)
0x25, 0x44, 0x42, 0x30, 0x37, 0x30, 0x30, 0x30, // %DB07000
0x00, 0x00  // PCL to PX4 data size ???
};

uint8_t write_command_head_buffer[40] = {
0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
0x00, 0x00, // Reserved[2]
0x00, 0x00, // PLC Info[2]
0xA0, // CPU Info[1]
0x33, // Source of Frame[1] : client->server(PLC)의 경우, 0x33
0x01, 0x00, // Invoke ID[2] : frame id
0x14, 0x00, // Length[2] : size of  Application Instruction  // 20 + data size 계산 필요
0x00, // Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
0xFF, // checksum : 0 ~ 19까지 값으로 계산
REQUEST_WRITE, 0x00, //Request[2]
0x14, 0x00, // Data Type[2] : 연속쓰기의 경우 0x0014
0x00, 0x00, // Reserved[2]
0x01, 0x00, // block length[2] : 0x01 (사용할 블럭의 갯수)
0x08, 0x00, // block name length[2] = 0x08 (블럭 이름의 길이)
0x25, 0x44, 0x42, 0x30, 0x36, 0x30, 0x30, 0x30, // %DB06000
0x00, 0x00 // PX4 to PLC Data Size
};
/**
 * @brief 16비트 워드의 하위 바이트를 8비트로 변환.
 *
 * @param word 16비트 워드.
 * @return uint8_t 하위 바이트.
 */
uint8_t u16ToU8Low(uint16_t word)
{
    return (uint8_t)(word & 0x00ff);
}

/**
 * @brief 16비트 워드의 상위 바이트를 8비트로 변환.
 *
 * @param word 16비트 워드.
 * @return uint8_t 상위 바이트.
 */
uint8_t u16ToU8High(uint16_t word)
{
    return (uint8_t)((word & 0xff00) >> 8);
}
uint8_t getCheckSum(uint8_t data[], uint8_t start, uint8_t length)
{
    uint16_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += *(data + start + i);
    }
    sum = sum & 0x00FF;
    return (uint8_t)sum;
}

int setReadArrayFrame(uint8_t *frame, int data_size)
{
    // Company ID[8] :LSIS-XGT
    *(frame + 0) = 0x4C; // L
    *(frame + 1) = 0x53; // S
    *(frame + 2) = 0x49; // I
    *(frame + 3) = 0x53; // S
    *(frame + 4) = 0x2D; // -
    *(frame + 5) = 0x58; // X
    *(frame + 6) = 0x47; // G
    *(frame + 7) = 0x54; // T

    *(frame + 8) = 0; // Reserved[2]
    *(frame + 9) = 0;

    *(frame + 10) = 0x00; // PLC Info[2] : client -> server(PLC)의 경우, Don't care (0x00)
    *(frame + 11) = 0x00; //

    *(frame + 12) = 0xA0; // CPU Info[1] : 0xA0

    *(frame + 13) = 0x33; // Source of Frame[1] : client->server(PLC)의 경우, 0x33

    *(frame + 14) = 0x00; // Invoke ID[2] : frame id
    *(frame + 15) = 0x00;

    *(frame + 16) = 20; // Length[2] : size of  Application Instruction
    *(frame + 17) = 0;

    *(frame + 18) = 0x00; // Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module

    *(frame + 19) = getCheckSum(frame, 0, 19); // Reserved2(BCC)[1] : byte sum of Application Header

    *(frame + 20) = REQUEST_READ; // Request[2] : 읽기 요청의 경우 0x0058
    *(frame + 21) = 0x00;

    *(frame + 22) = 0x14; // Data Type[2] : 연속 읽기의 경우 0x0014
    *(frame + 23) = 0x00;

    *(frame + 24) = 0x00; // Reserved[2] : Don't care
    *(frame + 25) = 0x00;

    *(frame + 26) = 0x01; // block length[2] : 0x01 (사용할 블럭의 갯수)
    *(frame + 27) = 0x00;

    *(frame + 28) = 0x08; // block name length[2] = 0x08 (블럭 이름의 길이)
    *(frame + 29) = 0x00;

    /** block name[8] : %DB07000 (Address of the variable you want to read, ASCII)
     *  실제 주소에 두배에 해당하는 주소를 사용해야 함. 예를 들어, 3500번지부터 읽고 싶으면, 7000번지를 사용해야 함.
    */
    *(frame + 30) = 0x25; // %
    *(frame + 31) = 0x44; // D
    *(frame + 32) = 0x42; // B
    *(frame + 33) = 0x30; // 0
    *(frame + 34) = 0x37; // 7
    *(frame + 35) = 0x30; // 0
    *(frame + 36) = 0x30; // 0
    *(frame + 37) = 0x30; // 0

    *(frame + 38) = u16ToU8Low(data_size); // data[2] : 데이터의 길이
    *(frame + 39) = u16ToU8High(data_size);

    return 40; // size of frame
}

int main()
{   int a = getCheckSum(read_command, 0, 19);
    std::cout << "check sum of ReadCommand :"<< a << std::endl;

    int b = getCheckSum(write_command_head_buffer, 0, 19);
    std::cout << "check sum of Write Command Head :"<< b << std::endl;

    return 1;
}
