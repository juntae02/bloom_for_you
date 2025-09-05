## 🌸 Bloom for you
- **키움의 과정을 선물하는** 꽃 선물 서비스 :  
&nbsp;&nbsp;선물의 목적과 기간에 적합한 꽃을 추천하고 재배 과정에서 음성을 기록하여 진심을 전달합니다.  
또한, 로봇암 기반의 자동화 시스템으로 재배 실패 부담을 해소합니다.
- 개발 기간 : 2025.06.09-06.20(2주)  
- 개발 인원 : 4명(팀원)  
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;해당 프로젝트는 **"선물의 종류는 많지만 감정은 없다"라는** 문제의식에서 출발했습니다. 
> 기존의 선물은 순간적인 감동에 그치는 경우가 많습니다. 저희는 꽃을 단순히 '주는 것'에 그치지 않고, **꽃의 성장 과정 자체를 선물의 일부로 만들어** 지속적인 **기쁨과 진심**을 전달하는 것을 목표로 했습니다. 
> 또한, 식물 재배의 어려움과 실패에 대한 부담을 **로봇 기술**로 해결함으로써, **누구나 꽃을 쉽게 기를 수 있도록 돕는 것**을 목표로 했습니다.
<br />

## 📌 주요 기능
- **상황별 꽃 추천** : 대화를 통해 선물의 목적과 기간에 최적화된 꽃을 추천하는 기능
- **자동화된 씨앗 심기** : 로봇암을 활용하여 씨앗 심기 과정을 자동화한 기능
- **음성 메시지 기록** : 재배 과정에서 음성을 기록하고 포장 시 QR코드를 생성하는 기능
- **자동 관수 시스템** : 꽃의 관수 주기에 맞춰 물을 자동으로 공급하는 기능 
- **YOLO 기반 객체 탐지** : 프로젝트 맞춤 객체 탐지 모델을 활용해 객체 인식 및 처리 기능
<br />

## 🎥 시연 영상
- [🎞️ Demo (개인 시연 영상)](https://www.youtube.com/watch?v=E3JtXJ45bFk)  
- [🎞️ Demo (전체 시연 영상)](https://www.youtube.com/watch?v=jA9iK2Lapts)  
👉 클릭하시면 시연 영상을 시청하실 수 있습니다.
- [🖥️ 발표 자료 (Canva)](https://www.canva.com/design/DAGt2hxdEvg/8CEbznRIRrQc8xgAosH4Jg/edit?utm_content=DAGt2hxdEvg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ Canva에서는 언어를 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어** : Doosan Robotics M-series M0609, realsense 카메라
- **개발 언어** : Python 
- **프론트엔드** : Kivy
- **백엔드** : Flask, rclpy
- **AI / 데이터 처리** : YOLO, OpenAI API
- **협업 툴** : GitHub, Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
&nbsp;&nbsp;직접 수집 및 가공한 데이터로 YOLO 객체 탐지 **모델을 학습**하고, **꽃 추천** 및 **씨앗 심기 자동화** 기능 구현을 담당했습니다.

- ***YOLO 기반 객체 탐지 모델 생성 및 학습*** :  
  &nbsp;&nbsp;프로젝트에서 사용할 객체(화분, 꽃, 씨앗 등)를 **직접 촬영하고 라벨링**한 후, 커스텀 데이터셋을 구성하여 **YOLO11n 모델**을 학습시켰습니다. 
  학습은 **ultralytics 라이브러리**를 활용했고, 이미지 크기, 에폭 수, 배치 크기 등을 조정하여 **프로젝트 환경에 맞는 .pt 모델을 생성**했습니다.  
  👉 [모델 생성 및 학습](https://github.com/juntae02/bloom_for_you/tree/main/yolo_models)
---

- ***상황별 꽃 추천 기능*** :  
&nbsp;&nbsp;OpenAI API 기반의 키워드 추출 **프롬프트**를 설계하여 대화에서 선물의 **목적과 남은 기간**을 효율적으로 추출했습니다. 
추출된 키워드를 **JSON 데이터와 매칭**하여 적절한 꽃을 **추천하는** 로직을 구현했습니다. 
또한, **Kivy GUI**에서 사용자와 상호작용하도록 개발했습니다.  
  👉 [flower_recommender.py (추천 기능)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L205-L250)  
  👉 [flower_recommender.py (GUI)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L95-L203)  
  👉 [추천 프롬프트 텍스트](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/prompt/recommender_prompt.txt)  
  👉 [추천 JSON 데이터](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/flower_recommendations.json)
---

- ***자동화된 씨앗 심기 기능*** :  
&nbsp;&nbsp;**ROS2 토픽 통신**을 통해 꽃 정보를 전달받아 로봇 동작을 제어했습니다. 
YOLO 탐지로 씨앗과 화분 위치를 파악하고, **'씨앗 집기 -> 운반 및 정밀 배치 -> 재배 구역 이송'** 로직을 구현했습니다. 
**씨앗 미탐지 예외 처리** 및 **순응 제어**를 적용하여 안정적인 동작을 보장했습니다.  
  👉 [seed_planting.py(씨앗 Pick&Place)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L76-L156)  
  👉 [seed_planting.py(화분 정밀 배치)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L158-L184)
<br />

## 🤔 트러블슈팅 및 해결

- **문제 상황 1: 꽃이 화분에 심어져 있는 경우에 꽃이 탐지되지 않는 현상**
  - **상황** : 꽃이 화분에 심어져 있는 경우, 꽃은 탐지되지 않고 화분만 인식되는 문제가 발생했습니다.
    
  - **원인** : 초기에는 빠른 추론 속도를 위해 **YOLOv8n 모델**을 사용했습니다. 하지만 **경량화된 구조**로 인하여
    작은 꽃과 큰 화분을 정확히 분리하는 데 한계가 있었습니다.
    
  - **해결** : 더 정밀한 특징 추출과 표현 능력을 가진 **YOLO11n 모델**로 교체했습니다. 
    그 결과, **mAP@0.5:0.95 기준으로 약 2% 성능 향상**을 달성했고 동일한 이미지에서도 **화분과 꽃을 분리 탐지**할 수 있었습니다.  
  👉 [트러블슈팅 해결 과정](https://github.com/juntae02/bloom_for_you/tree/main/yolo_models)
---

- **문제 상황 2: 씨앗 집기 실패 현상**
  - **상황** : 로봇이 씨앗을 인식하고 집는 과정에서 간헐적으로 **씨앗을 놓치거나 아무것도 잡지 못한 채 동작을 종료**하는 문제가 발생했습니다.
    
  - **원인** : **첫 번째로는 강한 역광**으로 인해 YOLO가 씨앗을 인식하지 못하는 경우에 발생했습니다. 
    **두 번째로는 씨앗이 모두 소진되는 경우처럼** 실제로 존재하지 않는 경우에 발생했습니다. 
    **마지막으로 씨앗의 크기 편차**로 인해 YOLO 모델이 중심 좌표를 정확히 탐지했음에도 그립 실패가 발생했습니다.
   
  - **해결** : grip 이후에 **get_status()[1] 값**을 확인하여, **실제로 물체가 잡혔는지 여부를 판별**하는 로직을 추가했습니다. 실패 시에는 **3번의 재시도**를 수행하고, 
    **모두 실패할 경우에는 call_manager() 함수**를 호출하여 사용자에게 상황을 알리는 로직을 구현했습니다. 
    그 결과, 씨앗이 잡히지 않는 상황에서도 **시스템이 비정상 중단되지 않았습니다**. 그리고 **관리자를 호출**을 통해 예외 상황을 안전하게 처리했습니다.  
  👉 [씨앗 집기 코드](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L93-L138)  
  👉 [관리자 호출 코드](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L201-L218)
<br />

## 💡 과정 속에서 배운 점
&nbsp;&nbsp;이번 프로젝트를 통해 로봇과 AI 기술을 결합해 **사용자에게 가치를 전달**하는 경험을 했습니다. 특히, **불확실성(역광, 크기 편차 등)을** 가진 환경에서 
로봇 시스템을 안정적으로 운영하기 위해 모델을 고도화하고 예외 상황을 처리하는 기술적 역량을 길렀습니다.
<br />

## 🤝 팀원 정보
- ***준태*** : YOLO 모델 생성, 꽃 추천 및 씨앗 심기(본인)   
- 석환 : 기능 모듈화, 전체 시나리오 관리 및 총괄 지휘  
- 예은 : 기능 모듈화, 자동 관수 및 꽃 포장 
- 요한 : Front-end, 음성 메시지 기록 및 QR코드 생성 
<br />

## 📚 참고 및 출처
- 본 프로젝트의 일부 코드는 **두산 로보틱스 부트캠프 교육과정**에서 제공된  
  `Tutorial.zip` 자료를 기반으로 작성되었습니다.
- `STT.py`, `wakeup_word.py`, `keyword_extraction.py` 파일은 해당 자료에서 가져와 **일부 기능을 수정하여 사용**하였습니다.
- 그 외의 파일은 프로젝트 목적에 맞게 **직접 구현된 코드**입니다.
- 원본 전체 코드는 포함되어 있지 않으며, 본 저장소에는 **필요한 일부 파일만 포함**되어 있습니다.

- `bloom_for_you/function_modules/onrobot_.py` 파일은 아래 공개 저장소의 코드를 기반으로 **일부 수정하여 사용**하였습니다:  
  > 🔗 https://github.com/takuya-ki/onrobot-rg

- 원본 코드는 [MIT License](https://github.com/takuya-ki/onrobot-rg/blob/main/LICENSE)를 따르며, 본 저장소 내 수정된 코드 역시 동일한 라이선스를 존중하여 사용하고 있습니다.

> 본 저장소는 **교육 및 학습 목적**으로만 사용됩니다.
