int64 id                      # 식별 번호
int64 command                 # 입력 명령
int64 zone_number             # 플랜트 존 위치 번호
string flower_name            # 꽃 이름
string flower_meaning         # 꽃말
int64 growth_duration_days    # 성장까지 필요한 기간 (일 단위)
int64 watering_cycle          # 물 주는 주기 (일 단위)
int64 growth_state            # 꽃 성장 상태

# command 설명
# 0: 비워둠
# 1: 꽃 추천
# 2: 씨앗 심기
# 3: 씨앗 심기 완료

# 10: 음성 녹음 시작
# 11: 음성 녹음 저장 완료

# 20: 포장 시작
# 21: 카드 포장 시작
# 22: 포장 완료

# 30: 물주기
# 31: 물주기 완료