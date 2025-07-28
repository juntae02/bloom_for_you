## 🧠 YOLO 모델 생성 및 학습
> &nbsp;&nbsp;프로젝트에 최적화된 객체 탐지 모델을 구축하기 위해, YOLO 모델을 생성 및 학습했습니다.  
> 해당 모델은 시나리오 상에서 중요한 역할을 하는 **화분, 꽃, 씨앗 등 한정된 객체**를 정확하게 탐지하는 데 기여했습니다.  

👉 [YOLO 모델](https://github.com/juntae02/bloom_for_you/tree/main/bloom_for_you/resource/models_and_json)
<br />

## 🤔 트러블슈팅
- **문제 상황** :  
  &nbsp;&nbsp;실제 테스트에서 화분에 꽃이 심어진 경우, **화분은 인식하지만 꽃은 인식되지 않는 문제**가 발생했습니다.
    
- **원인 분석** :  
  &nbsp;&nbsp;한정된 환경에서는 빠른 추론 속도와 낮은 리소스 사용량이 요구된다고 판단하여, 초기에는 경량 모델인 **YOLOv8n**을 사용하여 학습했습니다.  
  하지만 YOLOv8n은 **화분과 꽃의 경계가 겹치는 경우**, 상대적으로 작은 객체인 꽃이 화분에 묻혀, **정확한 분리 탐지**가 어렵다고 판단했습니다.  
  
- **해결 방안** :  

  &nbsp;&nbsp;더 정밀한 특징 추출과 표현 능력을 가진 **YOLO11n** 모델로 전환하여 학습했습니다.  
  그 결과 동일한 이미지에서도 **화분과 꽃을 모두 동시에 인식**할 수 있었습니다.  
<br />

## 🔍 아래는 두 모델의 인식 결과 비교 이미지입니다 :  
  <img src="./comparison/compare_visual.jpg" alt="인식 결과 비교 이미지" width="500" height="500"/>

## 📊 성능 개선 결과 :  
  &nbsp;&nbsp;**mAP@0.5:0.95** 기준으로 **약 2%의 성능 향상**이 있었으며,  
  이는 **작은 객체, 경계 겹침** 등 다양한 조건에서도 탐지 정확도가 개선되었음을 보여줍니다.  
  <img src="./comparison/compare_mAP50-95.jpg" alt="성능 비교 그래프" width="500" height="500"/>  
  | 항목               | YOLOv8n            | YOLO11n                  |  
  | ----------------- | ------------------ | ------------------------ |  
  |  📌 mAP@0.5:0.95  | 86.2%              | **88.1%**                |  
  |  ⚙️ 특징           | 낮은 리소스 사용      | 높은 탐지 정확도            |  
  👉 [YOLOv8n 성능 결과](https://github.com/juntae02/bloom_for_you/blob/main/yolo_models/yolov8n/results)  
  👉 [YOLO11n 성능 결과](https://github.com/juntae02/bloom_for_you/blob/main/yolo_models/yolo11n/results)  
<br />
