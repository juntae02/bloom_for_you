## 🧠 YOLO 모델 생성 및 학습
  &nbsp;&nbsp;프로젝트에 최적화된 객체 탐지 모델을 구축하기 위해, YOLO 모델을 생성 및 학습했습니다.  
  이 모델은 시나리오 상에서 중요한 역할을 하는 **화분, 꽃, 씨앗 등 한정된 객체**를 정확하게 탐지하는 데 기여했습니다.  
  👉 [YOLO 모델](https://github.com/juntae02/bloom_for_you/tree/main/bloom_for_you/resource/models_and_json)
<br />

## 🤔 트러블슈팅
- **문제 상황** :  
  &nbsp;&nbsp;실제 테스트에서 화분에 꽃이 심어진 경우, **화분은 인식하지만 꽃은 인식되지 않는 문제**가 발생했습니다.
    
- **원인 분석** :  
  &nbsp;&nbsp;한정된 환경은 빠른 추론 속도와 낮은 리소스 사용량이 요구된다고 판단하여, 경량 모델인 YOLOv8n를 사용하여 모델을 학습했습니다.  
  하지만 해당 모델은 화분과 꽃이 경계가 겹치는 경우, 상대적으로 작은 객체인 꽃이 묻혀 정확한 분리 탐지가 어렵다는 한계가 있다고 판단했습니다.  
  
- **해결 방안** :  
  &nbsp;&nbsp;더 정밀한 특징 추출과 표현 능력을 가진 YOLO11n 모델로 전환했습니다.  
  그 결과 동일한 이미지에서도 화분과 꽃을 모두 동시에 인식할 수 있었습니다.  

- 🔍 아래는 두 모델의 인식 결과 비교 이미지입니다 :  
  <img src="./comparison/compare_visual.jpg" alt="인식 결과 비교 이미지" width="500" height="500"/>

- 📊 성능 개선 결과 :  
  &nbsp;&nbsp;mAP@0.5:0.95 기준으로 약 2% 향상된 성능을 보였으며
  <img src="./comparison/compare_mAP50-95.jpg" alt="성능 비교 그래프" width="500" height="500"/>  
  👉 [YOLOv8n 성능 결과](https://github.com/juntae02/bloom_for_you/blob/main/yolo_models/yolov8n/results/results_yolov8n.png)  
  👉 [YOLO11n 성능 결과](https://github.com/juntae02/bloom_for_you/blob/main/yolo_models/yolo11n/results/results_yolo11n.png)  
<br />
