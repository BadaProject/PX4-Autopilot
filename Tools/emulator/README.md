# Emulator
## 준비물
- 공유기 or 스위치 허브
- Pixhawk or PX4 emulator
- PLC or PLC emulator

## PLC <-> PX4
* [network 설정](https://docs.google.com/presentation/d/1yWAXLv5-QHHBHnp6CrYQVKMTulQLzgeF3px3wkjxai0/edit#slide=id.g2fb059a45f9_0_5)

## 설정 파일
* config.json 파일

## PLC Emulator 실행
```bash
$ python3 plc.py
```

## PX4 Emulator 실행
```bash
$ python3 px4.py
```
## ToDo
* systemcmds/actuator_test 분석
   * 발생시키는 msg
* 구현
   * actuator_outputs msg 수신하기
   * 어떤 fields를 사용하는지 : steering, throttle
   * 해당 값을 전달하기

## Design
* 명령 전송 단계
   * uorb에서 actuator msg 얻기
   * actuator msg로 PX4ToPLC 구조체 채우기
   * PX4ToPLC를 UDP로 PLC로 전송하기

* 읽기 전송 단계
   * r
* actuator msg -> plc packet -> make_udp packet -> udp send
```c++
uint8_t send_buffer[1024];
uint8_t receive_buffer[1024];

uint8_t template_write_command[40];
uint8_t template_read_command[40] = {0x4C, };

make_command(bool isWrite, Msg msg);


void make_command(bool isWrite, Msg msg)
{
	if(isWrite){
		msg
	}
	else{
		send_buffer[0] =
	}
}
```
