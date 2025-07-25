## 2. 🌸 Bloom for you
> ***키움의 과정을 선물하는*** 꽃 선물 서비스  
> - 개발기간 : 2025.06.09-06.20(2주)  
> - 개발인원 : 4명(팀원)  

**[프로젝트 개요]**
> ***"선물의 종류는 많지만 감정은 없다"는*** 문제의식에서 출발  
> 선물의 ***목적과 기간***에 적합한 꽃을 추천  
> 재배 과정에서 음성을 기록하여 ***진심***을 전달  
> ***재배 실패***에 대한 부담을, 로봇암 기반의 자동화 시스템으로 해결  
>> - ***준태***: YOLO 모델 생성, 꽃 추천 및 씨앗 심기   
>> - 석환: 기능 모듈화, 전체 시나리오 관리 및 총괄 지휘  
>> - 예은: 기능 모듈화, 포장 및 자동급수  
>> - 요한: 음성 메시지 기록 및 Front-end  

**[담당 역할]**
> YOLO 모델 생성, 상황별 꽃 추천 및 해당 씨앗 심기 기능 구현  
>> - ***YOLO*** 모델을 학습시켜, 프로젝트에 최적화된 객체 탐지 모델 구축  
>> - ***OpenAI API***를 활용하여, 상황별 꽃 추천 기능  
>> - ***두산 로봇암***을 사용하여, 씨앗 심기 기능  
>> - [프로젝트 상세 설명](https://github.com/juntae02/bloom_for_you)  
<br />

# 🌸 Bloom for you - 꽃 선물 서비스

<img src="./bloom_for_you/resource/git_readme/congrats.png" alt="Project Banner" width="200"/>

> 특별한 사람을 위해 "키움의 과정을 선물하는" 꽃 선물 서비스, Bloom for you 🎁  
> OpenAI API를 활용한 상황별 꽃 선택 서비스, 로봇팔과 사진 기록을 통해 키우는 과정을 기록 및 선물

---

## 🎥 시연 영상
- 영상 링크: [Draw](https://youtu.be/ftucMoKNOY0)
👉 클릭해서 시연 영상 보기

---

## 🛠️ 기술 스택

- 🤖 **Robot 제어**: ROS2, MoveIt
- 🌐 **Frontend**: 플라스크, Kivy
- 🧠 **API**: OpenAI API
- 🧰 **협업 툴**: GitHub, Notion, Draw.io

---

## 🚀 주요 기능

- 🌺 데이터베이스에 저장된 꽃말과 재배기간을 확인해 LLM과 대화로 꽃 추천  
- 🤖 추천한 꽃을 로봇팔이 자동으로 씨앗 심기, 재배, 꽃 포장  
- 🗣️ 재배 과정 중 기록된 음성 메시지를 STT를 통해 요약 정리  
- 📦 꽃 포장 시 음성 메시지를 확인할 수 있는 서버 접속용 QR 코드 제공  
- 🧩 전체 과정을 통합적으로 제어하는 메인 코드 구성  

---

## 🗂️ 프로젝트 구조

```bash
📦 Bloom for you
├── 📁 Function Modules: 이름에 "_" 로 끝나는 모듈은 다른 기능 모듈의 구성 요소로 사용되는 내부 모듈
│   ├── ⚙️ config.py: 로봇팔 제어 및 서버 관련 기본 config 설정
│   ├── 🧠 keyword_extraction.py: 저장된 프롬프트와 사용자 대화 입력을 통해 키워드 추출
│   ├── 🤖 robot.py: 로봇팔의 움직임 및 그리퍼 제어 모듈
│   ├── 🗣️ stt.py / tts.py: 음성과 문자 변환 모듈
│   └── 🕵️ yolo.py: YOLO를 통해 물체 이름과 번호 출력
│
├── 🧪 test: 모듈 사용법 전달을 위한 테스트 코드
│   ├── 🤖 test_robot: 로봇 테스트 코드
│   └── 🕵️ test_yolo: YOLO 테스트 코드
│
├── 🧠 주요 Node
│   ├── 🧭 중앙 제어
│       ├── 🎛️ scenario_manager: 전체 시나리오를 커맨드를 받아서 제어
│       ├── 🎙️ speech_to_command: 음성 메시지 입력으로 커맨드 scenario_manager에 전달
│   ├── 🌱 씨앗 선택
│       ├── 🌸 flower_recommender: 음성 메시지를 통한 꽃 추천
│       ├── 🌾 seed_planting: 로봇팔 제어를 통한 씨앗 심기
│   ├── 🌿 재배
│       ├── 🎧 voice_memory: 재배 과정 중 음성 메시지 녹음
│       └── 💧 watering_scheduler: 주기적으로 물 주기 기능 실행
│   ├── 🎁 포장
│       ├── 📦 flower_wrapping: 로봇팔 제어를 통한 꽃 포장
│       └── 📝 make_letter_card: 저장된 음성 메시지 요약 + qr코드 입력된 card 생성
│   ├── ⚙️ 백그라운드 실행
│       ├── 🦾 set_robot: 커맨드 + 타겟 입력 시 pick and place
│       ├── 🌐 set_server: qr 코드로 접속하는 서버 실행
│       └── 🕵️ set_yolo: YOLO 모델 set up
│
└── 📄 README.md   
```


## 🧑‍💻 Contributor

| 이름 | 역할 | GitHub |
|------|------|--------|
| 위석환 | 중앙제어 / 팀 조율 | [@clarobit](https://github.com/clarobit) |
| 지예은 | Function Modules 설계 / 재배(물 주기) / 포장(꽃 포장) | [@yeyeyeyeyeyeun](https://github.com/yeyeyeyeyeyeun) |
| 박준태 | 씨앗 선택(꽃 추천, 씨앗 심기)  | [@juntae02](https://github.com/juntae02) |
| 김요한 | 재배(음성 메시지 녹음) / 포장(card 생성) / 백그라운드 실행(서버 실행) | [@KIMYOHAN60](https://github.com/KIMYOHAN60) |


- 발표 자료  
> 언어는 영어로 설정해야 폰트가 변형되지 않음  
>> - [🌸 Bloom for you](https://www.canva.com/design/DAGt2hxdEvg/8CEbznRIRrQc8xgAosH4Jg/edit?utm_content=DAGt2hxdEvg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)
