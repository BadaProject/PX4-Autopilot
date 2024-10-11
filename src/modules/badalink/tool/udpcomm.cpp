/*
* UDPSend Class를 생성하며 아래와 같은 속성과 메소드를 가진다.
   * 속성
     * int sockfd: 소켓 파일 디스크립터
     * struct sockaddr_in servaddr: 서버 주소 구조체
     * char buffer[1024]: 데이터를 저장할 버퍼
   * 메소드
     * UDPSend(): 생성자에서 생성한 소켓을 인자로 받는다.
     * ~UDPSend(): 소멸자
     * void sendData(const char* data): 데이터 전송할 곳의 ip는 192.168.2.88 이고 port는 2005이다.
     * void closeSocket(): 소켓 닫기

* UDPReceive class를 생성하며 아래와 같은 속성과 메소드를 가진다.
   * 속성
     * int sockfd: 소켓 파일 디스크립터
     * struct sockaddr_in servaddr: 서버 주소 구조체
     * char buffer[1024]: 데이터를 저장할 버퍼
   * 메소드
     * UDPReceive(): 생성자에서 생성한 소켓을 인자로 받는다.
     * ~UDPReceive(): 소멸자
     * void receiveData(): 데이터 수신

* main 함수에서 다음을 수행한다.
   * udp를 위한 소켓을 생성하고 2006 port로 바인딩한다.
   * UDPSend 객체를 생성할대 생성한 소켓을 인자로 넘긴다.
   * UDPReceive 객체를 생성할대 생성한 소켓을 인자로 넘긴다.
   * thread로 UDPReceive 객체의 receiveData 메소드를 실행한다.
   * thread로 UDPSend 객체의 sendData 메소드를 1초마다 실행시킨다.
   * 종료하기 전에 소켓을 닫는다.
*/

#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdint>
#define PLC_IP "127.0.0.1" // #define PLC_IP "192.168.2.88"
#define PLC_PORT 2005
#define PX4_IP "127.0.0.1" // #define PX4_IP "192.168.2.200"
#define PX4_PORT 2006

enum Request
{
    REQUEST_WRITE = 0x58,
    REQUEST_READ = 0x54,
};

enum RESPONSE
{
    RESPONSE_WRITE = 0x59,
    RESPONSE_READ = 0x55,
};

typedef struct{
uint16_t  auto_control_status;
uint16_t  engine_rpm_status;
uint16_t  clutch_status;
uint16_t  steering_angle_status;
uint16_t  trim_angle_command;
uint16_t  engine_running_status;
uint16_t  bow_thruster_power_status;
uint16_t  bow_thruster_rev_status;
} PlcToPX4Packet;

typedef struct {
  uint16_t emtpy1;
  uint16_t empty2;
  uint16_t engine_thrust;
  uint16_t clutch;
  uint16_t steering_angle;
  uint16_t trim_angle;
  uint16_t empty3;
  uint16_t engine_ignition;
  uint16_t bow_thruster_power;
  uint16_t bow_thruster_rev;
  uint16_t reserved1;
  uint16_t reserved2;
  uint16_t reserved3;
  uint16_t reserved4;
  uint16_t reserved5;
} PX4ToPlcPacket;

class PLCPacket
{
public:

  PLCPacket() {
    memset(buffer, 0, sizeof(buffer));
  }

  uint8_t buffer[100];

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

  uint16_t u8ToU16(uint8_t low, uint8_t high)
  {
    uint16_t ret = high;
    ret = ret << 8;
    ret = ret & 0xff00;
    return ret + (uint16_t)low;
  }

  uint8_t u16ToU8Low(uint16_t word)
  {
    return (uint8_t)(word & 0x00ff);
  }

  uint8_t u16ToU8High(uint16_t word)
  {
    return (uint8_t)((word & 0xff00) >> 8);
  }

  int makeWriteFrame(PX4ToPlcPacket data)
  {
    return setWriteArrayFrame(buffer, &data, sizeof(data));
  }

  int makeReadFrame()
  {
    return setReadArrayFrame(buffer, sizeof(PX4ToPlcPacket));
  }

  int setWriteArrayFrame(uint8_t *frame, PX4ToPlcPacket *data, int data_size)
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
    *(frame + 11) = 0x00;

    *(frame + 12) = 0xA0; // CPU Info[1] : 0xA0

    *(frame + 13) = 0x33; // Source of Frame[1] : client->server(PLC)의 경우, 0x33

    *(frame + 14) = 0x01; // Invoke ID[2] : frame id
    *(frame + 15) = 0x00;

    *(frame + 16) = u16ToU8Low(data_size + 20);  // Length[2] : size of  Application Instruction
    *(frame + 17) = u16ToU8High(data_size + 20);

    *(frame + 18) = 0x00; // Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module

    *(frame + 19) = getCheckSum(frame, 0, 19); // Reserved2(BCC)[1] : byte sum of Application Header

    *(frame + 20) = REQUEST_WRITE; // Request[2] : 쓰기 요청의 경우 0x0058
    *(frame + 21) = 0x00;

    *(frame + 22) = 0x14; // Data Type[2] : 연속쓰기의 경우 0x0014
    *(frame + 23) = 0x00;

    *(frame + 24) = 0x00; // Reserved[2] : Don't care
    *(frame + 25) = 0x00;

    *(frame + 26) = 0x01; // block length[2] : 0x01 (사용할 블럭의 갯수)
    *(frame + 27) = 0x00;

    *(frame + 28) = 0x08; // block name length[2] = 0x08 (블럭 이름의 길이)
    *(frame + 29) = 0x00;

    /** block name[8] : %DB06000 (쓰여질 변수의 주소, ASCII)
     *  실제 주소에 두배에 해당하는 주소를 사용해야 함. 예를 들어, 3000번지부터 쓰고 싶으면, 6000번지를 사용해야 함.
    */
    *(frame + 30) = 0x25; // %
    *(frame + 31) = 0x44; // D
    *(frame + 32) = 0x42; // B
    *(frame + 33) = 0x30; // 0
    *(frame + 34) = 0x30; // 0
    *(frame + 35) = 0x36; // 6
    *(frame + 36) = 0x30; // 0
    *(frame + 37) = 0x32; // 2

    *(frame + 38) = u16ToU8Low(data_size); // data[2] : 데이터의 길이
    *(frame + 39) = u16ToU8High(data_size);

    memcpy(frame + 40, data, data_size); // 쓸 데이터

    return data_size + 40; // size of frame
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

    *(frame + 16) = 0x14; // Length[2] : size of  Application Instruction
    *(frame + 17) = 0x00;

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
    *(frame + 34) = 0x30; // 0
    *(frame + 35) = 0x38; // 8
    *(frame + 36) = 0x30; // 0
    *(frame + 37) = 0x32; // 2

    *(frame + 38) = u16ToU8Low(data_size); // data[2] : 데이터의 길이
    *(frame + 39) = u16ToU8High(data_size);

    return 40; // size of frame
  }

  void parseResponse(uint8_t *recData, int data_size, PlcToPX4Packet* dest)
  {
    int hasError = 0;

    uint16_t sum = 0;
    for (int i = 0; i < 18; i++)
    {
        sum += *(recData + i);
    }
    if ((sum & 0x00ff) != *(recData + 19))
    {
        hasError++;
    }

    if(true)
    {
        if ((*(recData + 20)) == RESPONSE_READ)
        {
            memcpy(dest, recData + 32, u8ToU16(*(recData + 30), *(recData + 31)));
        }
    }
  }
};



class UDPSend {
public:
  int sockfd;
  struct sockaddr_in servaddr;
  char buffer[1024];
  bool toggle = false;

  UDPSend(int socket_fd, const struct sockaddr_in& server_addr) : sockfd(socket_fd), servaddr(server_addr) {
    memset(buffer, 0, sizeof(buffer));
  }

  ~UDPSend() {
    closeSocket();
  }

  void sendData(uint8_t* data, int data_size) {
    struct sockaddr_in destaddr;
    memset(&destaddr, 0, sizeof(destaddr));
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(PLC_PORT);
    inet_pton(AF_INET, PLC_IP, &destaddr.sin_addr);

    sendto(sockfd, data, data_size, 0, (const struct sockaddr*)&destaddr, sizeof(destaddr));
  }

  void closeSocket() {
    close(sockfd);
  }
};

class UDPReceive {
public:
  int sockfd;
  struct sockaddr_in servaddr;
  char buffer[1024];

  UDPReceive(int socket_fd, const struct sockaddr_in& server_addr) : sockfd(socket_fd), servaddr(server_addr) {
    memset(buffer, 0, sizeof(buffer));
  }

  ~UDPReceive() {
    closeSocket();
  }

  void receiveData() {
    socklen_t len = sizeof(servaddr);
    while(true) {
      int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&servaddr, &len);
      if (n > 0) {
        buffer[n] = '\0';
        std::cout << "Received: " << n << " : " << buffer << std::endl;
      }
    }
  }

  void closeSocket() {
    close(sockfd);
  }
};


int main() {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return -1;
  }

  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PX4_PORT);
  servaddr.sin_addr.s_addr = inet_addr(PX4_IP);

  if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "Failed to bind socket" << std::endl;
    return -1;
  }

  UDPSend udp_send(sockfd, servaddr);
  UDPReceive udp_receive(sockfd, servaddr);

  bool sendToggle = false;
  std::thread receive_thread(&UDPReceive::receiveData, &udp_receive);
  std::thread send_thread([&udp_send]() {

    while (true) {
      if (udp_send.toggle) { //write command
        PLCPacket plcPacket;
        PX4ToPlcPacket data;
        data.engine_thrust=1;
        data.clutch=2;
        data.steering_angle=3;
        data.trim_angle=4;
        data.engine_ignition=5;
        data.bow_thruster_power=6;
        data.bow_thruster_rev=7;
        int size = plcPacket.makeWriteFrame(data);
        udp_send.sendData(plcPacket.buffer, size);
        std::cout<<"Send Write Command!"<<std::endl;
      }
      else { //read command
        PLCPacket plcPacket;
        int size = plcPacket.makeReadFrame();
        udp_send.sendData(plcPacket.buffer, size);
        std::cout<<"Send Read Command!"<<std::endl;
      }
      udp_send.toggle = !udp_send.toggle;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  receive_thread.join();
  send_thread.join();

  return 0;
}
