# Custom MAVLink in PX4

이 문서는 PX4의 `src/modules/mavlink` 모듈에서 사용하는
`src/modules/mavlink/mavlink` 디렉터리가 어디에서 오는지, 빌드 과정에서 무엇이
생성되는지, 그리고 custom MAVLink 헤더가 필요할 때 어떤 방식으로 관리하는 것이
좋은지 정리한다.

## `src/modules/mavlink/mavlink`는 빌드 산출물인가?

아니다. `src/modules/mavlink/mavlink`는 빌드 과정에서 생성되는 디렉터리가 아니라
git submodule로 가져오는 MAVLink 원본 저장소다.

현재 `.gitmodules`에는 다음 submodule로 등록되어 있다.

```ini
[submodule "src/modules/mavlink/mavlink"]
	path = src/modules/mavlink/mavlink
	url = https://github.com/BadaProject/mavlink.git
	branch = custom_boat
```

실제 checkout된 submodule은 다음처럼 확인할 수 있다.

```sh
git submodule status --recursive src/modules/mavlink/mavlink
```

이 레포 기준 확인 결과는 다음과 같다.

```text
e8d867d355ab401e9290075cae85b983f34eb539 src/modules/mavlink/mavlink
fcaa2c7d25e3169dc66155929c338487941555e9 src/modules/mavlink/mavlink/pymavlink
```

즉 PX4 본체는 `src/modules/mavlink/mavlink`를 특정 commit에 고정된 submodule로
참조하고, 그 안의 `pymavlink`도 recursive submodule로 함께 필요하다.

## 빌드 과정에서 생성되는 것은 무엇인가?

빌드 과정에서 생성되는 것은 `src/modules/mavlink/mavlink` 자체가 아니라 MAVLink C
헤더다.

`src/modules/mavlink/CMakeLists.txt`는 다음 일을 한다.

1. `MAVLINK_GIT_DIR`를 `src/modules/mavlink/mavlink`로 설정한다.
2. `px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")`로 submodule
   존재 여부와 commit 상태를 확인한다.
3. `mavgen.py`를 실행해 빌드 디렉터리에 MAVLink C 헤더를 생성한다.

생성 위치는 source tree가 아니라 build tree다.

```text
build/<target>/mavlink/<dialect>/<dialect>.h
build/<target>/mavlink/uAvionix/uAvionix.h
```

예를 들어 `CONFIG_MAVLINK_DIALECT="bada"`인 보드라면 primary dialect 헤더는
대략 다음 위치에 생성된다.

```text
build/<target>/mavlink/bada/bada.h
```

그리고 `mavlink_c` interface library가 이 build tree include path를 사용한다.

```cmake
target_include_directories(mavlink_c
	INTERFACE
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
)
```

따라서 정리하면 다음과 같다.

| 항목 | 출처 | 생성/관리 방식 |
| --- | --- | --- |
| `src/modules/mavlink/mavlink` | `https://github.com/BadaProject/mavlink.git`, `custom_boat` branch | git submodule |
| `src/modules/mavlink/mavlink/pymavlink` | MAVLink submodule 내부 | recursive git submodule |
| `build/<target>/mavlink/<dialect>/*.h` | MAVLink XML + `mavgen.py` | PX4 빌드 중 생성 |

## `src/modules/mavlink/mavlink`를 가져오는 방법

PX4를 처음 clone할 때는 recursive clone을 사용하는 것이 가장 단순하다.

```sh
git clone --recursive <PX4-repository-url>
```

이미 clone한 뒤라면 다음 명령으로 MAVLink submodule만 가져올 수 있다.

```sh
git submodule sync --recursive -- src/modules/mavlink/mavlink
git submodule update --init --recursive -- src/modules/mavlink/mavlink
```

전체 submodule을 한 번에 맞추려면 다음을 사용한다.

```sh
git submodule sync --recursive
git submodule update --init --recursive
```

PX4 CMake configure/build 과정에서도 `px4_add_git_submodule()`이
`Tools/check_submodules.sh src/modules/mavlink/mavlink`를 호출한다. submodule이
없으면 자동으로 `git submodule update --init --recursive`를 시도하고, commit이
기대값과 다르면 사용자에게 업데이트 여부를 묻는다.

CI, VSCode CMake, CLion 환경에서는 상호작용 없이 submodule sync/update를 수행하도록
되어 있다.

## MAVLink dialect 선택

PX4에서 primary MAVLink dialect는 보드 설정의 `CONFIG_MAVLINK_DIALECT`로 결정된다.

예:

```text
CONFIG_MAVLINK_DIALECT="bada"
```

빌드 시에는 다음 XML이 입력으로 사용된다.

```text
src/modules/mavlink/mavlink/message_definitions/v1.0/bada.xml
```

그리고 항상 `uAvionix` dialect도 추가로 생성된다.

```text
src/modules/mavlink/mavlink/message_definitions/v1.0/uAvionix.xml
```

## Custom MAVLink 헤더가 필요한 경우

### 현재 적용 방식: MAVLink submodule을 fork로 관리

PX4 firmware에서 custom message를 안정적으로 사용하기 위해 MAVLink 저장소를 fork하고
custom dialect XML을 fork 안에서 관리한 뒤 PX4의 submodule commit을 그 fork의 특정
commit으로 고정한다. 현재 이 저장소는 `https://github.com/BadaProject/mavlink.git`의
`custom_boat` branch를 사용한다.

권장 흐름은 다음과 같다.

1. `mavlink/mavlink`를 fork한다.
2. fork의 `message_definitions/v1.0/` 아래에 custom dialect XML을 추가하거나 기존
   dialect를 확장한다.
3. 필요한 경우 custom dialect가 `common.xml` 또는 `development.xml`을 include하도록
   구성한다.
4. PX4의 `src/modules/mavlink/mavlink` submodule remote를 fork로 바꾼다.
5. submodule을 원하는 commit으로 checkout한다.
6. PX4 상위 repo에서 submodule pointer 변경을 commit한다.
7. 보드 설정의 `CONFIG_MAVLINK_DIALECT`를 custom dialect 이름으로 설정한다.

예시:

```sh
cd src/modules/mavlink/mavlink
git remote set-url origin <your-mavlink-fork-url>
git fetch origin
git checkout <custom-mavlink-commit>
cd -
git add src/modules/mavlink/mavlink .gitmodules
```

보드 설정 예시:

```text
CONFIG_MAVLINK_DIALECT="bada"
```

이 경우 PX4 빌드가 다음 XML을 사용해 C 헤더를 생성한다.

```text
src/modules/mavlink/mavlink/message_definitions/v1.0/bada.xml
```

### 임시 개발 방식: submodule 안에서 직접 수정

빠른 실험만 필요하다면 `src/modules/mavlink/mavlink/message_definitions/v1.0/` 안의 XML을
직접 수정하고 빌드할 수 있다. 하지만 이 방식은 다음 이유로 장기 관리에는 적합하지
않다.

- submodule 내부 변경은 PX4 상위 repo에서 일반 파일 변경처럼 보이지 않는다.
- 다른 개발자가 같은 헤더를 재현하려면 submodule 내부 commit이나 patch를 별도로 알아야 한다.
- `git submodule update` 과정에서 작업 내용이 사라지거나 detached HEAD 상태에서 관리가
  꼬일 수 있다.

임시 수정 후에도 공유가 필요하다면 반드시 MAVLink fork에 commit을 만들고 PX4 상위
repo의 submodule pointer를 그 commit으로 고정하는 것이 좋다.

### 생성된 헤더를 직접 커밋하는 방식은 비추천

`build/<target>/mavlink/<dialect>/` 아래 생성된 C 헤더를 PX4 source tree에 복사해
직접 커밋하는 방식은 추천하지 않는다.

이유는 다음과 같다.

- PX4 빌드는 XML에서 헤더를 재생성하는 구조다.
- 생성 헤더는 target/build directory에 종속된다.
- XML과 헤더가 불일치하면 companion, GCS, PX4 간 message CRC/ID 불일치가 생길 수 있다.
- MAVLink message 정의의 source of truth가 XML이 아니라 복사된 헤더로 흩어진다.

custom message가 필요하면 XML을 관리하고, PX4와 companion/GCS가 같은 XML 또는 같은
MAVLink fork commit에서 헤더/라이브러리를 생성하도록 맞추는 것이 안전하다.

## 권장 체크리스트

custom MAVLink를 적용할 때는 아래를 같이 확인한다.

| 항목 | 확인 내용 |
| --- | --- |
| Submodule URL | `.gitmodules`의 `src/modules/mavlink/mavlink` URL이 의도한 MAVLink fork인지 확인 |
| Submodule commit | `git submodule status --recursive src/modules/mavlink/mavlink`로 고정 commit 확인 |
| Dialect XML | `message_definitions/v1.0/<dialect>.xml`이 존재하는지 확인 |
| Board config | 대상 보드의 `.px4board`에 `CONFIG_MAVLINK_DIALECT="<dialect>"` 설정 |
| Generated header | 빌드 후 `build/<target>/mavlink/<dialect>/<dialect>.h` 생성 여부 확인 |
| Companion/GCS | PX4와 동일한 dialect XML 또는 동일 MAVLink fork commit 사용 |
| Message ID/CRC | custom message ID 충돌과 CRC mismatch가 없는지 확인 |

## 결론

- `src/modules/mavlink/mavlink`는 빌드되는 디렉터리가 아니라 git submodule이다.
- PX4 빌드가 생성하는 것은 build tree 아래의 MAVLink C 헤더다.
- custom MAVLink가 필요하면 생성된 헤더를 직접 들고 오는 것보다 MAVLink fork의 XML을
  관리하고, PX4 submodule pointer와 `CONFIG_MAVLINK_DIALECT`를 명시적으로 고정하는
  방식을 추천한다.
