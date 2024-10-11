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


class UDPSend {
public:
  int sockfd;
  struct sockaddr_in servaddr;
  char buffer[1024];

  UDPSend(int socket_fd, const struct sockaddr_in& server_addr) : sockfd(socket_fd), servaddr(server_addr) {
    memset(buffer, 0, sizeof(buffer));
  }

  ~UDPSend() {
    closeSocket();
  }

  void sendData(const char* data) {
    struct sockaddr_in destaddr;
    memset(&destaddr, 0, sizeof(destaddr));
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(2005);
    inet_pton(AF_INET, "192.168.2.88", &destaddr.sin_addr);

    sendto(sockfd, data, strlen(data), 0, (const struct sockaddr*)&destaddr, sizeof(destaddr));
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
    int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&servaddr, &len);
    if (n > 0) {
      buffer[n] = '\0';
      std::cout << "Received: " << buffer << std::endl;
    }
  }

  void closeSocket() {
    close(sockfd);
  }
};

void print_hello(){
  std::cout << "Hello from thread" << std::endl;
}

int main() {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return -1;
  }

  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(2006);
  servaddr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "Failed to bind socket" << std::endl;
    return -1;
  }

  UDPSend udp_send(sockfd, servaddr);
  UDPReceive udp_receive(sockfd, servaddr);

  std::thread t(print_hello);
  t.join();

  std::thread t2([&udp_send]() {
    udp_send.sendData("Hello from UDP Send");
  });
  t2.join();

  //send_thread 생성하여 udp_send의 sendData 메소드를 실행한다.
  //1초마다 실행시킨다.
  // std::thread send_thread([&udp_send](){
  //   while (true) {
  //     udp_send.sendData("Hello from UDP Send");
  //     std::this_thread::sleep_for(std::chrono::seconds(1));
  //   }
  // });

  // receive_thread.join();
  // send_thread.join();

  return 0;
}
